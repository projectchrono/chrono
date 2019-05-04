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
    int nscalar_coords = KRM.Get_K()->GetColumns();
    K.Reset(nscalar_coords, nscalar_coords);
    R.Reset(nscalar_coords, nscalar_coords);
    M.Reset(nscalar_coords, nscalar_coords);
}

// -----------------------------------------------------------------------------

ChLoadBase::ChLoadBase() : jacobians(nullptr) {}

ChLoadBase::~ChLoadBase() {
    if (jacobians)
        delete jacobians;
}

void ChLoadBase::Update(double time) {
    // current state speed & position
    ChState mstate_x(this->LoadGet_ndof_x(), 0);
    this->LoadGetStateBlock_x(mstate_x);
    ChStateDelta mstate_w(this->LoadGet_ndof_w(), 0);
    this->LoadGetStateBlock_w(mstate_w);
    // compute the applied load, at current state
    this->ComputeQ(&mstate_x, &mstate_w);
    // compute the jacobian, at current state
    if (this->IsStiff()) {
        if (!this->jacobians)
            this->CreateJacobianMatrices();
        this->ComputeJacobian(&mstate_x, &mstate_w, this->jacobians->K, this->jacobians->R, this->jacobians->M);
    }
};

void ChLoadBase::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    if (this->jacobians) {
        mdescriptor.InsertKblock(&this->jacobians->KRM);
    }
}

void ChLoadBase::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    if (this->jacobians) {
        this->jacobians->KRM.Get_K()->FillElem(0);
        this->jacobians->KRM.Get_K()->MatrInc(this->jacobians->K * Kfactor);
        this->jacobians->KRM.Get_K()->MatrInc(this->jacobians->R * Rfactor);
        this->jacobians->KRM.Get_K()->MatrInc(this->jacobians->M * Mfactor);
    }
}

// -----------------------------------------------------------------------------

ChLoadCustom::ChLoadCustom(std::shared_ptr<ChLoadable> mloadable) : loadable(mloadable) {
    load_Q.Reset(this->LoadGet_ndof_w());
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
                                   ChMatrix<>& mK,         // result dQ/dx
                                   ChMatrix<>& mR,         // result dQ/dv
                                   ChMatrix<>& mM)         // result dQ/da
{
    double Delta = 1e-8;

    int mrows_w = this->LoadGet_ndof_w();
    int mrows_x = this->LoadGet_ndof_x();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    this->ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
    Q0 = this->load_Q;

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);
    ChState state_x_inc(mrows_x, nullptr);
    ChStateDelta state_delta(mrows_w, nullptr);

    // Compute K=-dQ(x,v)/dx by backward differentiation
    state_delta.Reset(mrows_w, nullptr);

    for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        this->LoadStateIncrement(*state_x, state_delta,
                                 state_x_inc);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;
        this->ComputeQ(&state_x_inc, state_w);  // Q1 = Q(x+Dx, v)
        Q1 = this->load_Q;
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
        this->jacobians->K.PasteMatrix(Jcolumn, 0, i);
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        this->ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = this->load_Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        this->jacobians->R.PasteMatrix(Jcolumn, 0, i);
    }
}

void ChLoadCustom::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int rowQ = 0;
    for (int i = 0; i < this->loadable->GetSubBlocks(); ++i) {
        unsigned int moffset = this->loadable->GetSubBlockOffset(i);
        for (unsigned int row = 0; row < this->loadable->GetSubBlockSize(i); ++row) {
            R(row + moffset) += this->load_Q(rowQ) * c;
            ++rowQ;
        }
    }
}

void ChLoadCustom::CreateJacobianMatrices() {
    if (!this->jacobians) {
        // create jacobian structure
        this->jacobians = new ChLoadJacobians;
        // set variables forsparse KRM block
        std::vector<ChVariables*> mvars;
        loadable->LoadableGetVariables(mvars);
        this->jacobians->SetVariables(mvars);
    }
}

// -----------------------------------------------------------------------------

ChLoadCustomMultiple::ChLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& mloadables)
    : loadables(mloadables) {
    load_Q.Reset(this->LoadGet_ndof_w());
}

ChLoadCustomMultiple::ChLoadCustomMultiple(std::shared_ptr<ChLoadable> mloadableA,
                                           std::shared_ptr<ChLoadable> mloadableB) {
    loadables.push_back(mloadableA);
    loadables.push_back(mloadableB);
    load_Q.Reset(this->LoadGet_ndof_w());
}

ChLoadCustomMultiple::ChLoadCustomMultiple(std::shared_ptr<ChLoadable> mloadableA,
                                           std::shared_ptr<ChLoadable> mloadableB,
                                           std::shared_ptr<ChLoadable> mloadableC) {
    loadables.push_back(mloadableA);
    loadables.push_back(mloadableB);
    loadables.push_back(mloadableC);
    load_Q.Reset(this->LoadGet_ndof_w());
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
                                           ChMatrix<>& mK,         // result dQ/dx
                                           ChMatrix<>& mR,         // result dQ/dv
                                           ChMatrix<>& mM)         // result dQ/da
{
    double Delta = 1e-8;

    int mrows_w = this->LoadGet_ndof_w();
    int mrows_x = this->LoadGet_ndof_x();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    this->ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
    Q0 = this->load_Q;

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);
    ChState state_x_inc(mrows_x, nullptr);
    ChStateDelta state_delta(mrows_w, nullptr);

    // Compute K=-dQ(x,v)/dx by backward differentiation
    state_delta.Reset(mrows_w, nullptr);

    for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        this->LoadStateIncrement(*state_x, state_delta,
                                 state_x_inc);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;
        this->ComputeQ(&state_x_inc, state_w);  // Q1 = Q(x+Dx, v)
        Q1 = this->load_Q;
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
        this->jacobians->K.PasteMatrix(Jcolumn, 0, i);
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        this->ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = this->load_Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        this->jacobians->R.PasteMatrix(Jcolumn, 0, i);
    }
}

void ChLoadCustomMultiple::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int mQoffset = 0;
    for (int k = 0; k < loadables.size(); ++k) {
        std::vector<ChVariables*> kvars;
        loadables[k]->LoadableGetVariables(kvars);
        for (int i = 0; i < loadables[k]->GetSubBlocks(); ++i) {
            if (kvars[i]->IsActive()) {
                unsigned int mblockoffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    R(row + mblockoffset) += this->load_Q(row + mQoffset) * c;
                }
            }
            mQoffset += loadables[k]->GetSubBlockSize(i);
        }
    }
    // GetLog() << " debug: R=" << R << "\n";
}

void ChLoadCustomMultiple::CreateJacobianMatrices() {
    if (!this->jacobians) {
        // create jacobian structure
        this->jacobians = new ChLoadJacobians;
        // set variables for sparse KRM block appending them to mvars list
        std::vector<ChVariables*> mvars;
        for (int i = 0; i < loadables.size(); ++i)
            loadables[i]->LoadableGetVariables(mvars);
        this->jacobians->SetVariables(mvars);
    }
}

}  // end namespace chrono
