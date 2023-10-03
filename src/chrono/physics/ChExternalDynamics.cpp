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

#include "chrono/physics/ChExternalDynamics.h"

namespace chrono {

// Perturbation for finite-difference Jacobian approximation
const double ChExternalDynamics::m_FD_delta = 1e-8;

ChExternalDynamics::ChExternalDynamics(std::shared_ptr<ODE> ode) : m_ode(ode) {
    m_nstates = ode->GetNumStates();
    m_states.resize(m_nstates);
    m_rhs.resize(m_nstates);

    ode->SetInitialConditions(m_states, *this);

    m_variables = new ChVariablesGenericDiagonalMass(m_nstates);
    m_variables->GetMassDiagonal().Constant(m_nstates, 1);

    if (ode->IsStiff()) {
        m_jac.resize(m_nstates, m_nstates);

        std::vector<ChVariables*> vars;
        vars.push_back(m_variables);
        m_KRM.SetVariables(vars);
    }
}

ChExternalDynamics::~ChExternalDynamics() {
    delete m_variables;
}

ChExternalDynamics* ChExternalDynamics::Clone() const {
    return new ChExternalDynamics(*this);
}

// -----------------------------------------------------------------------------

void ChExternalDynamics::ComputeQ(double time, ChVectorDynamic<>& rhs) {
    m_ode->CalculateRHS(time, m_states, rhs, *this);
}

void ChExternalDynamics::ComputeJac(double time) {
    m_jac.setZero();

    // Invoke user-provided Jacobian function
    bool has_jac = m_ode->CalculateJac(time, m_states, m_rhs, m_jac, *this);

    // If Jacobian not provided, estimate with finite differences
    if (!has_jac) {
        ChVectorDynamic<> rhs1(m_nstates);
        ChVectorDynamic<> Jcolumn(m_nstates);
        for (int i = 0; i < m_nstates; i++) {
            m_states(i) += m_FD_delta;
            ComputeQ(time, rhs1);
            Jcolumn = (rhs1 - m_rhs) * (1 / m_FD_delta);
            m_jac.col(i) = Jcolumn;
            m_states(i) -= m_FD_delta;
        }
    }
}

void ChExternalDynamics::Update(double time, bool update_assets) {
    ChTime = time;

    // Compute forcing terms at current states
    ComputeQ(time, m_rhs);

    // Compute Jacobian (if needed)
    if (m_ode->IsStiff()) {
        ComputeJac(time);
    }

    // Update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

// -----------------------------------------------------------------------------

void ChExternalDynamics::InjectVariables(ChSystemDescriptor& descriptor) {
    // The base class does not actually inject any variables
    ChPhysicsItem::InjectVariables(descriptor);

    m_variables->SetDisabled(!IsActive());
    descriptor.InsertVariables(m_variables);
}

void ChExternalDynamics::InjectKRMmatrices(ChSystemDescriptor& descriptor) {
    // The base class does nothing
    ChPhysicsItem::InjectVariables(descriptor);

    if (m_ode->IsStiff()) {
        descriptor.InsertKblock(&m_KRM);
    }
}

void ChExternalDynamics::IntStateGather(const unsigned int off_x,  // offset in x state vector
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

void ChExternalDynamics::IntStateScatter(const unsigned int off_x,  // offset in x state vector
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

void ChExternalDynamics::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    if (!IsActive())
        return;

    a.segment(off_a, m_nstates) = m_rhs;
}

void ChExternalDynamics::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // Nothing to do here
}

void ChExternalDynamics::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                           ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                           const double c           // a scaling factor
) {
    if (!IsActive())
        return;

    // Add forcing term for internal variables
    R.segment(off, m_nstates) += c * m_rhs;
}

void ChExternalDynamics::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                            ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                            const ChVectorDynamic<>& v,  // the v vector
                                            const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    R.segment(off, m_nstates) += c * v.segment(off, m_nstates);
}

void ChExternalDynamics::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                         const ChStateDelta& v,
                                         const ChVectorDynamic<>& R,
                                         const unsigned int off_L,  // offset in L, Qc
                                         const ChVectorDynamic<>& L,
                                         const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    m_variables->Get_qb() = v.segment(off_v, m_nstates);
    m_variables->Get_fb() = R.segment(off_v, m_nstates);
}

void ChExternalDynamics::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                           ChStateDelta& v,
                                           const unsigned int off_L,  // offset in L
                                           ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    v.segment(off_v, m_nstates) = m_variables->Get_qb();
}

// -----------------------------------------------------------------------------

void ChExternalDynamics::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    if (m_ode->IsStiff()) {
        // Recall to flip sign to load K = -dQ/dx and R = -dQ/dv
        m_KRM.Get_K() = -Rfactor * m_jac;
    }
}

// -----------------------------------------------------------------------------

void ChExternalDynamics::VariablesFbReset() {
    m_variables->Get_fb().setZero();
}

void ChExternalDynamics::VariablesFbLoadForces(double factor) {
    m_variables->Get_fb() = m_rhs;
}

void ChExternalDynamics::VariablesQbLoadSpeed() {
    m_variables->Get_qb() = m_states;
}

void ChExternalDynamics::VariablesQbSetSpeed(double step) {
    m_states = m_variables->Get_qb();
}

void ChExternalDynamics::VariablesFbIncrementMq() {
    m_variables->Compute_inc_Mb_v(m_variables->Get_fb(), m_variables->Get_qb());
}

void ChExternalDynamics::VariablesQbIncrementPosition(double dt_step) {
    //// RADU: correct?
    // nothing to do here
}

void ChExternalDynamics::ConstraintsFbLoadForces(double factor) {
    // nothing to do here
}

}  // end namespace chrono
