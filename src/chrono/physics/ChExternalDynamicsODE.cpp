// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChExternalDynamicsODE.h"

namespace chrono {

// Perturbation for finite-difference Jacobian approximation
const double ChExternalDynamicsODE::m_FD_delta = 1e-8;

ChExternalDynamicsODE::ChExternalDynamicsODE() {}
ChExternalDynamicsODE::~ChExternalDynamicsODE() {
    delete m_variables;
}

void ChExternalDynamicsODE::Initialize() {
    m_nstates = GetNumStates();
    m_states.resize(m_nstates);
    m_rhs.resize(m_nstates);

    SetInitialConditions(m_states);

    m_variables = new ChVariablesGenericDiagonalMass(m_nstates);
    m_variables->GetMassDiagonal().Constant(m_nstates, 1);

    if (IsStiff()) {
        m_jac.resize(m_nstates, m_nstates);

        std::vector<ChVariables*> vars;
        vars.push_back(m_variables);
        m_KRM.SetVariables(vars);
    }
}

ChVectorDynamic<> ChExternalDynamicsODE::GetInitialStates() {
    ChVectorDynamic<> y0(m_nstates);
    SetInitialConditions(y0);
    return y0;
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsODE::ComputeJac(double time) {
    m_jac.setZero();

    // Invoke Jacobian function
    bool has_jac = CalculateJac(time, m_states, m_rhs, m_jac);

    // If Jacobian not provided, estimate with finite differences
    if (!has_jac) {
        ChVectorDynamic<> rhs1(m_nstates);
        ChVectorDynamic<> Jcolumn(m_nstates);
        for (int i = 0; i < m_nstates; i++) {
            m_states(i) += m_FD_delta;
            CalculateRHS(time, m_states, rhs1);
            Jcolumn = (rhs1 - m_rhs) * (1 / m_FD_delta);
            m_jac.col(i) = Jcolumn;
            m_states(i) -= m_FD_delta;
        }
    }
}

void ChExternalDynamicsODE::Update(double time, bool update_assets) {
    // Update time and assets
    ChPhysicsItem::Update(time, update_assets);

    // Compute forcing terms at current states
    CalculateRHS(time, m_states, m_rhs);

    // Compute Jacobian (if needed)
    if (IsStiff()) {
        ComputeJac(time);
    }
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsODE::InjectVariables(ChSystemDescriptor& descriptor) {
    m_variables->SetDisabled(!IsActive());
    descriptor.InsertVariables(m_variables);
}

void ChExternalDynamicsODE::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (IsStiff()) {
        descriptor.InsertKRMBlock(&m_KRM);
    }
}

void ChExternalDynamicsODE::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                        ChState& x,                // state vector, position part
                                        const unsigned int off_v,  // offset in v state vector
                                        ChStateDelta& v,           // state vector, speed part
                                        double& T                  // time
) {
    if (!IsActive())
        return;

    x.segment(off_x, m_nstates).setZero();
    v.segment(off_v, m_nstates) = m_states;
    T = GetChTime();
}

void ChExternalDynamicsODE::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                         const ChState& x,          // state vector, position part
                                         const unsigned int off_v,  // offset in v state vector
                                         const ChStateDelta& v,     // state vector, speed part
                                         const double T,            // time
                                         bool full_update           // perform complete update
) {
    if (!IsActive())
        return;

    // Important: set the internal states first, as they will be used in Update.
    m_states = v.segment(off_v, m_nstates);

    Update(T, full_update);
}

void ChExternalDynamicsODE::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    if (!IsActive())
        return;

    a.segment(off_a, m_nstates) = m_rhs;
}

void ChExternalDynamicsODE::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // Nothing to do here
}

void ChExternalDynamicsODE::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                           ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                           const double c           // a scaling factor
) {
    if (!IsActive())
        return;

    // Add forcing term for internal variables
    R.segment(off, m_nstates) += c * m_rhs;
}

void ChExternalDynamicsODE::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                            ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                            const ChVectorDynamic<>& v,  // the v vector
                                            const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    R.segment(off, m_nstates) += c * v.segment(off, m_nstates);
}

void ChExternalDynamicsODE::IntLoadLumpedMass_Md(const unsigned int off,
                                              ChVectorDynamic<>& Md,
                                              double& err,
                                              const double c) {
    if (!IsActive())
        return;

    Md.segment(off, m_nstates).array() += c * 1.0;
}

void ChExternalDynamicsODE::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                         const ChStateDelta& v,
                                         const ChVectorDynamic<>& R,
                                         const unsigned int off_L,  // offset in L, Qc
                                         const ChVectorDynamic<>& L,
                                         const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_variables->State() = v.segment(off_v, m_nstates);
    m_variables->Force() = R.segment(off_v, m_nstates);
}

void ChExternalDynamicsODE::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                           ChStateDelta& v,
                                           const unsigned int off_L,  // offset in L
                                           ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    v.segment(off_v, m_nstates) = m_variables->State();
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsODE::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (IsStiff()) {
        // Recall to flip sign to load R = -dQ/dv (K is zero here)
        m_KRM.GetMatrix() = Mfactor * ChMatrixDynamic<>::Identity(m_nstates, m_nstates) - Rfactor * m_jac;
    }
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsODE::VariablesFbReset() {
    m_variables->Force().setZero();
}

void ChExternalDynamicsODE::VariablesFbLoadForces(double factor) {
    m_variables->Force() = m_rhs;
}

void ChExternalDynamicsODE::VariablesQbLoadSpeed() {
    m_variables->State() = m_states;
}

void ChExternalDynamicsODE::VariablesQbSetSpeed(double step) {
    m_states = m_variables->State();
}

void ChExternalDynamicsODE::VariablesFbIncrementMq() {
    m_variables->AddMassTimesVector(m_variables->Force(), m_variables->State());
}

void ChExternalDynamicsODE::VariablesQbIncrementPosition(double dt_step) {
    //// RADU: correct?
    // nothing to do here
}

void ChExternalDynamicsODE::ConstraintsFbLoadForces(double factor) {
    // nothing to do here
}

}  // end namespace chrono
