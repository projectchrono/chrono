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
    auto nscalar_coords = KRM.GetMatrix().cols();
    K.setZero(nscalar_coords, nscalar_coords);
    R.setZero(nscalar_coords, nscalar_coords);
    M.setZero(nscalar_coords, nscalar_coords);
}

// -----------------------------------------------------------------------------

ChLoadBase::ChLoadBase() : m_jacobians(nullptr) {}

ChLoadBase::~ChLoadBase() {
    delete m_jacobians;
}

void ChLoadBase::Update(double time, bool update_assets) {
    ChObj::Update(time, update_assets);

    // current state speed & position
    ChState mstate_x(LoadGetNumCoordsPosLevel(), 0);
    LoadGetStateBlock_x(mstate_x);
    ChStateDelta mstate_w(LoadGetNumCoordsVelLevel(), 0);
    LoadGetStateBlock_w(mstate_w);

    // compute the applied load, at current state
    ComputeQ(&mstate_x, &mstate_w);

    // compute the jacobian, at current state
    if (IsStiff()) {
        if (!m_jacobians)
            CreateJacobianMatrices();
        ComputeJacobian(&mstate_x, &mstate_w);
    }
}

void ChLoadBase::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (m_jacobians) {
        descriptor.InsertKRMBlock(&m_jacobians->KRM);
    }
}

void ChLoadBase::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (m_jacobians) {
        m_jacobians->KRM.GetMatrix().setZero();
        m_jacobians->KRM.GetMatrix() += m_jacobians->K * Kfactor;
        m_jacobians->KRM.GetMatrix() += m_jacobians->R * Rfactor;
        m_jacobians->KRM.GetMatrix() += m_jacobians->M * Mfactor;
    }
}

// -----------------------------------------------------------------------------

int ChLoad::LoadGetNumCoordsPosLevel() {
    return loader->GetLoadable()->GetLoadableNumCoordsPosLevel();
}

int ChLoad::LoadGetNumCoordsVelLevel() {
    return loader->GetLoadable()->GetLoadableNumCoordsVelLevel();
}

void ChLoad::LoadGetStateBlock_x(ChState& mD) {
    loader->GetLoadable()->LoadableGetStateBlockPosLevel(0, mD);
}

void ChLoad::LoadGetStateBlock_w(ChStateDelta& mD) {
    loader->GetLoadable()->LoadableGetStateBlockVelLevel(0, mD);
}

void ChLoad::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    loader->GetLoadable()->LoadableStateIncrement(0, x_new, x, 0, dw);
}

int ChLoad::LoadGetNumFieldCoords() {
    return loader->GetLoadable()->GetNumFieldCoords();
}

void ChLoad::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    loader->ComputeQ(state_x, state_w);
}

void ChLoad::ComputeJacobian(ChState* state_x, ChStateDelta* state_w) {
    double Delta = 1e-8;

    int mrows_w = LoadGetNumCoordsVelLevel();
    int mrows_x = LoadGetNumCoordsPosLevel();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    loader->ComputeQ(state_x, state_w);  // Q0 = Q(x, v)
    Q0 = loader->Q;

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);
    ChState state_x_inc(mrows_x, nullptr);
    ChStateDelta state_delta(mrows_w, nullptr);

    // Compute K=-dQ(x,v)/dx by backward differentiation
    state_delta.setZero(mrows_w, nullptr);

    for (int i = 0; i < mrows_w; ++i) {
        state_delta(i) += Delta;
        LoadStateIncrement(*state_x, state_delta,
                           state_x_inc);          // exponential: state_x_inc=state_x+Delta
        loader->ComputeQ(&state_x_inc, state_w);  // Q1 = Q(x+Dx, v)
        Q1 = loader->Q;
        state_delta(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
        m_jacobians->K.block(0, i, mrows_w, 1) = Jcolumn;
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        loader->ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = loader->Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        m_jacobians->R.block(0, i, mrows_w, 1) = Jcolumn;
    }
}

void ChLoad::LoadIntLoadResidual_F(ChVectorDynamic<>& R, double c) {
    unsigned int rowQ = 0;
    for (unsigned int i = 0; i < loader->GetLoadable()->GetNumSubBlocks(); ++i) {
        if (loader->GetLoadable()->IsSubBlockActive(i)) {
            unsigned int moffset = loader->GetLoadable()->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loader->GetLoadable()->GetSubBlockSize(i); ++row) {
                R(row + moffset) += loader->Q(rowQ) * c;
                ++rowQ;
            }
        }
    }
}

void ChLoad::LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, double c) {
    if (!m_jacobians)
        return;
    // fetch w as a contiguous vector
    ChVectorDynamic<> grouped_w(LoadGetNumCoordsVelLevel());
    ChVectorDynamic<> grouped_cMv(LoadGetNumCoordsVelLevel());
    unsigned int rowQ = 0;
    for (unsigned int i = 0; i < loader->GetLoadable()->GetNumSubBlocks(); ++i) {
        if (loader->GetLoadable()->IsSubBlockActive(i)) {
            unsigned int moffset = loader->GetLoadable()->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loader->GetLoadable()->GetSubBlockSize(i); ++row) {
                grouped_w(rowQ) = w(row + moffset);
                ++rowQ;
            }
        }
    }
    // do computation R=c*M*v
    grouped_cMv = c * m_jacobians->M * grouped_w;
    rowQ = 0;
    for (unsigned int i = 0; i < loader->GetLoadable()->GetNumSubBlocks(); ++i) {
        if (loader->GetLoadable()->IsSubBlockActive(i)) {
            unsigned int moffset = loader->GetLoadable()->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loader->GetLoadable()->GetSubBlockSize(i); ++row) {
                R(row + moffset) += grouped_cMv(rowQ) * c;
                ++rowQ;
            }
        }
    }
}

void ChLoad::LoadIntLoadLumpedMass_Md(ChVectorDynamic<>& Md, double& err, double c) {
    if (!m_jacobians)
        return;
    // do computation Md=c*diag(M)
    unsigned int rowQ = 0;
    for (unsigned int i = 0; i < loader->GetLoadable()->GetNumSubBlocks(); ++i) {
        if (loader->GetLoadable()->IsSubBlockActive(i)) {
            unsigned int moffset = loader->GetLoadable()->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loader->GetLoadable()->GetSubBlockSize(i); ++row) {
                Md(row + moffset) += c * m_jacobians->M(rowQ, rowQ);
                ++rowQ;
            }
        }
    }
    err = m_jacobians->M.sum() - m_jacobians->M.diagonal().sum();
}

void ChLoad::CreateJacobianMatrices() {
    if (!m_jacobians) {
        // create jacobian structure
        m_jacobians = new ChLoadJacobians;
        // set variables forsparse KRM block
        std::vector<ChVariables*> mvars;
        loader->GetLoadable()->LoadableGetVariables(mvars);
        m_jacobians->SetVariables(mvars);
    }
}

// -----------------------------------------------------------------------------

ChLoadCustom::ChLoadCustom(std::shared_ptr<ChLoadable> loadable_object) : loadable(loadable_object) {
    load_Q.setZero(LoadGetNumCoordsVelLevel());
}

int ChLoadCustom::LoadGetNumCoordsPosLevel() {
    return loadable->GetLoadableNumCoordsPosLevel();
}
int ChLoadCustom::LoadGetNumCoordsVelLevel() {
    return loadable->GetLoadableNumCoordsVelLevel();
}
void ChLoadCustom::LoadGetStateBlock_x(ChState& mD) {
    loadable->LoadableGetStateBlockPosLevel(0, mD);
}
void ChLoadCustom::LoadGetStateBlock_w(ChStateDelta& mD) {
    loadable->LoadableGetStateBlockVelLevel(0, mD);
}
void ChLoadCustom::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    loadable->LoadableStateIncrement(0, x_new, x, 0, dw);
}
int ChLoadCustom::LoadGetNumFieldCoords() {
    return loadable->GetNumFieldCoords();
}

void ChLoadCustom::ComputeJacobian(ChState* state_x, ChStateDelta* state_w) {
    double Delta = 1e-8;

    int mrows_w = LoadGetNumCoordsVelLevel();
    int mrows_x = LoadGetNumCoordsPosLevel();

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
        m_jacobians->K.block(0, i, mrows_w, 1) = Jcolumn;
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = load_Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        m_jacobians->R.block(0, i, mrows_w, 1) = Jcolumn;
    }
}

void ChLoadCustom::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int rowQ = 0;
    for (unsigned int i = 0; i < loadable->GetNumSubBlocks(); ++i) {
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
    if (!m_jacobians)
        return;
    // fetch w as a contiguous vector
    ChVectorDynamic<> grouped_w(LoadGetNumCoordsVelLevel());
    grouped_w.setZero();
    unsigned int rowQ = 0;
    for (unsigned int i = 0; i < loadable->GetNumSubBlocks(); ++i) {
        if (loadable->IsSubBlockActive(i)) {
            unsigned int moffset = loadable->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                grouped_w(rowQ) = w(row + moffset);
                ++rowQ;
            }
        }
    }
    // do computation R=c*M*v
    ChVectorDynamic<> grouped_cMv(LoadGetNumCoordsVelLevel());
    grouped_cMv = c * m_jacobians->M * grouped_w;
    rowQ = 0;
    for (unsigned int i = 0; i < loadable->GetNumSubBlocks(); ++i) {
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
    if (!m_jacobians)
        return;
    // do computation Md=c*diag(M)
    unsigned int rowQ = 0;
    for (unsigned int i = 0; i < loadable->GetNumSubBlocks(); ++i) {
        if (loadable->IsSubBlockActive(i)) {
            unsigned int moffset = loadable->GetSubBlockOffset(i);
            for (unsigned int row = 0; row < loadable->GetSubBlockSize(i); ++row) {
                Md(row + moffset) += c * m_jacobians->M(rowQ, rowQ);
                ++rowQ;
            }
        }
    }
    err = m_jacobians->M.sum() - m_jacobians->M.diagonal().sum();
}

void ChLoadCustom::CreateJacobianMatrices() {
    if (!m_jacobians) {
        // create jacobian structure
        m_jacobians = new ChLoadJacobians;
        // set variables forsparse KRM block
        std::vector<ChVariables*> mvars;
        loadable->LoadableGetVariables(mvars);
        m_jacobians->SetVariables(mvars);
    }
}

// -----------------------------------------------------------------------------

ChLoadCustomMultiple::ChLoadCustomMultiple(std::vector<std::shared_ptr<ChLoadable>>& loadable_objects)
    : loadables(loadable_objects) {
    load_Q.setZero(LoadGetNumCoordsVelLevel());
}

ChLoadCustomMultiple::ChLoadCustomMultiple(std::shared_ptr<ChLoadable> loadableA,
                                           std::shared_ptr<ChLoadable> loadableB) {
    loadables.push_back(loadableA);
    loadables.push_back(loadableB);
    load_Q.setZero(LoadGetNumCoordsVelLevel());
}

ChLoadCustomMultiple::ChLoadCustomMultiple(std::shared_ptr<ChLoadable> loadableA,
                                           std::shared_ptr<ChLoadable> loadableB,
                                           std::shared_ptr<ChLoadable> loadableC) {
    loadables.push_back(loadableA);
    loadables.push_back(loadableB);
    loadables.push_back(loadableC);
    load_Q.setZero(LoadGetNumCoordsVelLevel());
}

int ChLoadCustomMultiple::LoadGetNumCoordsPosLevel() {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i)
        ndoftot += loadables[i]->GetLoadableNumCoordsPosLevel();
    return ndoftot;
}

int ChLoadCustomMultiple::LoadGetNumCoordsVelLevel() {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i)
        ndoftot += loadables[i]->GetLoadableNumCoordsVelLevel();
    return ndoftot;
}

void ChLoadCustomMultiple::LoadGetStateBlock_x(ChState& mD) {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i) {
        loadables[i]->LoadableGetStateBlockPosLevel(ndoftot, mD);
        ndoftot += loadables[i]->GetLoadableNumCoordsPosLevel();
    }
}

void ChLoadCustomMultiple::LoadGetStateBlock_w(ChStateDelta& mD) {
    int ndoftot = 0;
    for (int i = 0; i < loadables.size(); ++i) {
        loadables[i]->LoadableGetStateBlockVelLevel(ndoftot, mD);
        ndoftot += loadables[i]->GetLoadableNumCoordsVelLevel();
    }
}

void ChLoadCustomMultiple::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    int ndoftotx = 0;
    int ndoftotw = 0;
    for (int i = 0; i < loadables.size(); ++i) {
        loadables[i]->LoadableStateIncrement(ndoftotx, x_new, x, ndoftotw, dw);
        ndoftotx += loadables[i]->GetLoadableNumCoordsPosLevel();
        ndoftotw += loadables[i]->GetLoadableNumCoordsVelLevel();
    }
}

int ChLoadCustomMultiple::LoadGetNumFieldCoords() {
    return loadables[0]->GetNumFieldCoords();
}

void ChLoadCustomMultiple::ComputeJacobian(ChState* state_x, ChStateDelta* state_w) {
    double Delta = 1e-8;

    int mrows_w = LoadGetNumCoordsVelLevel();
    int mrows_x = LoadGetNumCoordsPosLevel();

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
        m_jacobians->K.block(0, i, mrows_w, 1) = Jcolumn;
    }
    // Compute R=-dQ(x,v)/dv by backward differentiation
    for (int i = 0; i < mrows_w; ++i) {
        (*state_w)(i) += Delta;
        ComputeQ(state_x, state_w);  // Q1 = Q(x, v+Dv)
        Q1 = load_Q;
        (*state_w)(i) -= Delta;

        Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
        m_jacobians->R.block(0, i, mrows_w, 1) = Jcolumn;
    }
}

void ChLoadCustomMultiple::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    unsigned int mQoffset = 0;
    for (unsigned int k = 0; k < (unsigned int)loadables.size(); ++k) {
        for (unsigned int i = 0; i < loadables[k]->GetNumSubBlocks(); ++i) {
            if (loadables[k]->IsSubBlockActive(i)) {
                unsigned int mblockoffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    R(row + mblockoffset) += load_Q(row + mQoffset) * c;
                }
            }
            mQoffset += loadables[k]->GetSubBlockSize(i);
        }
    }
    // std::cout << " debug: R=" << R << std::endl;
}

void ChLoadCustomMultiple::LoadIntLoadResidual_Mv(ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {
    if (!m_jacobians)
        return;
    // fetch w as a contiguous vector
    ChVectorDynamic<> grouped_w(LoadGetNumCoordsVelLevel());
    grouped_w.setZero();
    unsigned int rowQ = 0;
    for (unsigned int k = 0; k < (unsigned int)loadables.size(); ++k) {
        for (unsigned int i = 0; i < loadables[k]->GetNumSubBlocks(); ++i) {
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
    ChVectorDynamic<> grouped_cMv(LoadGetNumCoordsVelLevel());
    grouped_cMv = c * m_jacobians->M * grouped_w;
    rowQ = 0;
    for (unsigned int k = 0; k < (unsigned int)loadables.size(); ++k) {
        for (unsigned int i = 0; i < loadables[k]->GetNumSubBlocks(); ++i) {
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
    if (!m_jacobians)
        return;
    // do computation Md=c*diag(M)
    unsigned int rowQ = 0;
    for (int k = 0; k < loadables.size(); ++k) {
        for (unsigned int i = 0; i < loadables[k]->GetNumSubBlocks(); ++i) {
            if (loadables[k]->IsSubBlockActive(i)) {
                unsigned int moffset = loadables[k]->GetSubBlockOffset(i);
                for (unsigned int row = 0; row < loadables[k]->GetSubBlockSize(i); ++row) {
                    Md(row + moffset) += c * m_jacobians->M(rowQ, rowQ);
                    ++rowQ;
                }
            }
        }
    }
    err = m_jacobians->M.sum() - m_jacobians->M.diagonal().sum();
}

void ChLoadCustomMultiple::CreateJacobianMatrices() {
    if (!m_jacobians) {
        // create jacobian structure
        m_jacobians = new ChLoadJacobians;
        // set variables for sparse KRM block appending them to mvars list
        std::vector<ChVariables*> mvars;
        for (int i = 0; i < loadables.size(); ++i)
            loadables[i]->LoadableGetVariables(mvars);
        m_jacobians->SetVariables(mvars);
    }
}

}  // end namespace chrono
