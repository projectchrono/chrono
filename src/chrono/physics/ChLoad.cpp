// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLoad.h"

namespace chrono {

void ChLoadJacobians::SetVariables(std::vector<ChVariables*> mvariables) {
    KRM.SetVariables(mvariables);
    auto nscalar_coords = KRM.Get_K().cols();
    K.setZero(nscalar_coords, nscalar_coords);
    R.setZero(nscalar_coords, nscalar_coords);
    M.setZero(nscalar_coords, nscalar_coords);
}

// -----------------------------------------------------------------------------

ChLoadBase::ChLoadBase() : jacobians(nullptr) {}

ChLoadBase::~ChLoadBase() {
    delete jacobians;
}

void ChLoadBase::Update(double time) {
    // current state speed & position
    ChState mstate_x(LoadGet_ndof_x(), 0);
    LoadGetStateBlock_x(mstate_x);
    ChStateDelta mstate_w(LoadGet_ndof_w(), 0);
    LoadGetStateBlock_w(mstate_w);
    // compute the applied load, at current state
    ComputeQ(&mstate_x, &mstate_w);
    // compute the jacobian, at current state
    if (IsStiff()) {
        if (!jacobians)
            CreateJacobianMatrices();
        ComputeJacobian(&mstate_x, &mstate_w, jacobians->K, jacobians->R, jacobians->M);
    }
};

void ChLoadBase::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    if (jacobians) {
        mdescriptor.InsertKblock(&jacobians->KRM);
    }
}

void ChLoadBase::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    if (jacobians) {
        jacobians->KRM.Get_K().setZero();
        jacobians->KRM.Get_K() += jacobians->K * Kfactor;
        jacobians->KRM.Get_K() += jacobians->R * Rfactor;
        jacobians->KRM.Get_K() += jacobians->M * Mfactor;
    }
}

// -----------------------------------------------------------------------------

ChLoadCustom::ChLoadCustom(std::shared_ptr<ChLoadable> mloadable) : loadable(mloadable) {
    load_Q.setZero(LoadGet_ndof_w());
}

int ChLoadCustom::LoadGet_ndof_x() {
    return loadable->LoadableGet_ndof_x();
}
int ChLoadCustom::LoadGet_ndof_w() {
    return loadable->LoadableGet_ndof_w();
}
void ChLoadCustom::LoadGetStateBlock_x(ChState& mD) {
    loadable->LoadableGetStateBlock_x(0, mD);
}
void ChLoadCustom::LoadGetStateBlock_w(ChStateDelta& mD) {
    loadable->LoadableGetStateBlock_w(0, mD);
}
void ChLoadCustom::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    loadable->LoadableStateIncrement(0, x_new, x, 0, dw);
}
int ChLoadCustom::LoadGet_field_ncoords() {
    return loadable->Get_field_ncoords();
}

void ChLoadCustom::ComputeJacobian(ChState* state_x,       // state position to evaluate jacobians
                                   ChStateDelta* state_w,  // state speed to evaluate jacobians
                                   ChMatrixRef mK,         // result dQ/dx
                                   ChMatrixRef mR,         // result dQ/dv
                                   ChMatrixRef mM)         // result dQ/da
{
    double Delta = 1e-8;

    int mrows_w = LoadGet_ndof_w();
    int mrows_x = LoadGet_ndof_x();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
    Q0 = load_Q;

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);
    ChState state_x_inc(mrows_x, nullptr);
    ChStateDelta state_delta(mrows_w, nullptr);

    // Compute K=-dQ(x,v)/dx by backward differentiation
    state_delta.setZero(mrows_w, nullptr);

    for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        LoadStateIncrement(*state_x, state_delta,
                                 state_x_inc);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;
        ComputeQ(&state_x_inc, state_w);  // Q1 = Q(x+Dx, v)
        Q1 = load_Q;
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
        jacobians->K.block(0, i, mrows_w, 1) = Jcolumn;
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = load_Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        jacobians->R.block(0, i, mrows_w, 1) = Jcolumn;
    }
}

void ChLoadCustom::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int rowQ = 0;
    for (int i = 0; i < loadable->GetSubBlocks(); ++i) {
        if (loadable->IsSubBlockActive(i)) {
            unsigned int moffset = loadable->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                R(row + moffset) += load_Q(rowQ) * c;
                ++rowQ;
            }
        }
    }
}

void ChLoadCustom::LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    if (!this->jacobians)
        return;
    // fetch w as a contiguous vector
    ChVectorDynamic<> grouped_w(this->LoadGet_ndof_w());
    ChVectorDynamic<> grouped_cMv(this->LoadGet_ndof_w());
    unsigned int rowQ = 0;
    for (int i = 0; i < loadable->GetSubBlocks(); ++i) {
        if (loadable->IsSubBlockActive(i)) {
            unsigned int moffset = loadable->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                grouped_w(rowQ) = w(row + moffset);
                ++rowQ;
            }
        }
    }
    // do computation R=c*M*v
    grouped_cMv = c * this->jacobians->M * grouped_w;
    rowQ = 0;
    for (int i = 0; i < loadable->GetSubBlocks(); ++i) {
        if (loadable->IsSubBlockActive(i)) {
            unsigned int moffset = loadable->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                R(row + moffset) += grouped_cMv(rowQ) * c;
                ++rowQ;
            }
        }
    }
}

void ChLoadCustom::LoadIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& err, const double c) {
    if (!this->jacobians)
        return;
    // do computation Md=c*diag(M)
    unsigned int rowQ = 0;
    for (int i = 0; i < loadable->GetSubBlocks(); ++i) {
        if (loadable->IsSubBlockActive(i)) {
            unsigned int moffset = loadable->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                Md(row + moffset) += c * this->jacobians->M(rowQ, rowQ);
                ++rowQ;
            }
        }
    }
    err = this->jacobians->M.sum() - this->jacobians->M.diagonal().sum();
}

void ChLoadCustom::CreateJacobianMatrices() {
    if (!jacobians) {
        // create jacobian structure
        jacobians = new ChLoadJacobians;
        // set variables forsparse KRM block
        std::vector<ChVariables*> mvars;
        loadable->LoadableGetVariables(mvars);
        jacobians->SetVariables(mvars);
    }
}

// -----------------------------------------------------------------------------

ChLoadCustomMultiple::ChLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& mloadables)
    : loadables(mloadables) {
    load_Q.setZero(LoadGet_ndof_w());
}

ChLoadCustomMultiple::ChLoadCustomMultiple(std::shared_ptr<ChLoadable> mloadableA,
                                           std::shared_ptr<ChLoadable> mloadableB) {
    loadables.push_back(mloadableA);
    loadables.push_back(mloadableB);
    load_Q.setZero(LoadGet_ndof_w());
}

ChLoadCustomMultiple::ChLoadCustomMultiple(std::shared_ptr<ChLoadable> mloadableA,
                                           std::shared_ptr<ChLoadable> mloadableB,
                                           std::shared_ptr<ChLoadable> mloadableC) {
    loadables.push_back(mloadableA);
    loadables.push_back(mloadableB);
    loadables.push_back(mloadableC);
    load_Q.setZero(LoadGet_ndof_w());
}

int ChLoadCustomMultiple::LoadGet_ndof_x() {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i)
        ndoftot += loadables[i]->LoadableGet_ndof_x();
    return ndoftot;
}

int ChLoadCustomMultiple::LoadGet_ndof_w() {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i)
        ndoftot += loadables[i]->LoadableGet_ndof_w();
    return ndoftot;
}

void ChLoadCustomMultiple::LoadGetStateBlock_x(ChState& mD) {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i) {
        loadables[i]->LoadableGetStateBlock_x(ndoftot, mD);
        ndoftot += loadables[i]->LoadableGet_ndof_x();
    }
}

void ChLoadCustomMultiple::LoadGetStateBlock_w(ChStateDelta& mD) {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i) {
        loadables[i]->LoadableGetStateBlock_w(ndoftot, mD);
        ndoftot += loadables[i]->LoadableGet_ndof_w();
    }
}

void ChLoadCustomMultiple::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    int ndoftotx = 0;
    int ndoftotw = 0;
    for (int i = 0; i < loadables.size(); ++i) {
        loadables[i]->LoadableStateIncrement(ndoftotx, x_new, x, ndoftotw, dw);
        ndoftotx += loadables[i]->LoadableGet_ndof_x();
        ndoftotw += loadables[i]->LoadableGet_ndof_w();
    }
}

int ChLoadCustomMultiple::LoadGet_field_ncoords() {
    return loadables[0]->Get_field_ncoords();
}

void ChLoadCustomMultiple::ComputeJacobian(ChState* state_x,       // state position to evaluate jacobians
                                           ChStateDelta* state_w,  // state speed to evaluate jacobians
                                           ChMatrixRef mK,         // result dQ/dx
                                           ChMatrixRef mR,         // result dQ/dv
                                           ChMatrixRef mM)         // result dQ/da
{
    double Delta = 1e-8;

    int mrows_w = LoadGet_ndof_w();
    int mrows_x = LoadGet_ndof_x();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
    Q0 = load_Q;

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);
    ChState state_x_inc(mrows_x, nullptr);
    ChStateDelta state_delta(mrows_w, nullptr);

    // Compute K=-dQ(x,v)/dx by backward differentiation
    state_delta.setZero(mrows_w, nullptr);

    for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        LoadStateIncrement(*state_x, state_delta,
                                 state_x_inc);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;
        ComputeQ(&state_x_inc, state_w);  // Q1 = Q(x+Dx, v)
        Q1 = load_Q;
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
        jacobians->K.block(0, i, mrows_w, 1) = Jcolumn;
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = load_Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        jacobians->R.block(0, i, mrows_w, 1) = Jcolumn;
    }
}

void ChLoadCustomMultiple::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int mQoffset = 0;
    for (int k = 0; k < loadables.size(); ++k) {
        for (int i = 0; i < loadables[k]->GetSubBlocks(); ++i) {
            if (loadables[k]->IsSubBlockActive(i)) {
                unsigned int mblockoffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    R(row + mblockoffset) += load_Q(row + mQoffset) * c;
                }
            }
            mQoffset += loadables[k]->GetSubBlockSize(i);
        }
    }
    // GetLog() << " debug: R=" << R << "\n";
}

void ChLoadCustomMultiple::LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    if (!this->jacobians)
        return;
    // fetch w as a contiguous vector
    ChVectorDynamic<> grouped_w(this->LoadGet_ndof_w());
    ChVectorDynamic<> grouped_cMv(this->LoadGet_ndof_w());
    unsigned int rowQ = 0;
    for (int k = 0; k < loadables.size(); ++k) {
        for (int i = 0; i < loadables[k]->GetSubBlocks(); ++i) {
            if (loadables[k]->IsSubBlockActive(i)) {
                unsigned int moffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    grouped_w(rowQ) = w(row + moffset);
                    ++rowQ;
                }
            }
        }
    }
    // do computation R=c*M*v
    grouped_cMv = c * this->jacobians->M * grouped_w;
    rowQ = 0;
    for (int k = 0; k < loadables.size(); ++k) {
        for (int i = 0; i < loadables[k]->GetSubBlocks(); ++i) {
            if (loadables[k]->IsSubBlockActive(i)) {
                unsigned int moffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    R(row + moffset) += grouped_cMv(rowQ) * c;
                    ++rowQ;
                }
            }
        }
    }
}

void ChLoadCustomMultiple::LoadIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& err, double c) {
    if (!this->jacobians)
        return;
    // do computation Md=c*diag(M)
    unsigned int rowQ = 0;
    for (int k = 0; k < loadables.size(); ++k) {
        for (int i = 0; i < loadables[k]->GetSubBlocks(); ++i) {
            if (loadables[k]->IsSubBlockActive(i)) {
                unsigned int moffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    Md(row + moffset) += c * this->jacobians->M(rowQ,rowQ);
                    ++rowQ;
                }
            }
        }
    }
    err = this->jacobians->M.sum() - this->jacobians->M.diagonal().sum();
}

void ChLoadCustomMultiple::CreateJacobianMatrices() {
    if (!jacobians) {
        // create jacobian structure
        jacobians = new ChLoadJacobians;
        // set variables for sparse KRM block appending them to mvars list
        std::vector<ChVariables*> mvars;
        for (int i = 0; i < loadables.size(); ++i)
            loadables[i]->LoadableGetVariables(mvars);
        jacobians->SetVariables(mvars);
    }
}

}  // end namespace chrono
