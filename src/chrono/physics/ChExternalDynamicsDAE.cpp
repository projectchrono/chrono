// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChExternalDynamicsDAE.h"

namespace chrono {

// Perturbation for finite-difference Jacobian approximation
const double ChExternalDynamicsDAE::m_FD_delta = 1e-8;

ChExternalDynamicsDAE::ChExternalDynamicsDAE() {}
ChExternalDynamicsDAE::~ChExternalDynamicsDAE() {
    delete m_variables;
}

void ChExternalDynamicsDAE::Initialize() {
    m_ny = GetNumStates();
    m_nyd = GetNumStateDerivatives();
    m_y.resize(m_ny);
    m_yd.resize(m_nyd);
    m_ydd.resize(m_nyd);
    m_F.resize(m_nyd);

    SetInitialConditions(m_y, m_yd);
    m_ydd.setZero();

    m_variables = new ChVariablesGeneric(m_nyd);
    if (InExplicitForm()) {
        m_variables->GetMass().setIdentity(m_nyd, m_nyd);
        m_variables->GetInvMass().setIdentity(m_nyd, m_nyd);
    } else {
        m_M.resize(m_nyd, m_nyd);
        CalculateMassMatrix(m_M);

        m_variables->GetMass() = m_M;
        m_variables->GetInvMass() = m_M.inverse();
    }

    if (IsStiff()) {
        m_J.resize(m_nyd, m_nyd);

        std::vector<ChVariables*> vars;
        vars.push_back(m_variables);
        m_KRM.SetVariables(vars);
    }

    m_nc = GetNumAlgebraicConstraints();
    m_c.resize(m_nc);
    m_multipliers.resize(m_nc, 0);
    m_Jc.resize(m_nc, m_nyd);

    if (IsRheonomous()) {
        m_ct.resize(m_nc);
    }

    m_constraints.resize(m_nc);
    for (auto& constraint : m_constraints)
        constraint.SetVariables({m_variables});
}

ChVectorDynamic<> ChExternalDynamicsDAE::GetInitialStates() {
    ChVectorDynamic<> y0(m_ny);
    ChVectorDynamic<> yd0(m_nyd);
    SetInitialConditions(y0, yd0);
    return y0;
}

ChVectorDynamic<> ChExternalDynamicsDAE::GetInitialStateDerivatives() {
    ChVectorDynamic<> y0(m_ny);
    ChVectorDynamic<> yd0(m_nyd);
    SetInitialConditions(y0, yd0);
    return yd0;
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsDAE::Update(double time, bool update_assets) {
    ChTime = time;

    OnUpdate(time, m_y, m_yd);

    // Compute forcing terms at current states
    CalculateForce(time, m_y, m_yd, m_F);

    // Compute constraint violations
    CalculateConstraintViolation(time, m_y, m_c);

    // Compute constraint Jacobian
    CalculateConstraintJacobian(time, m_y, m_c, m_Jc);

    // Compute constraint time derivatives
    if (IsRheonomous()) {
        ComputeConstraintDerivative(time);
    }

    // Update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsDAE::InjectVariables(ChSystemDescriptor& descriptor) {
    m_variables->SetDisabled(!IsActive());
    descriptor.InsertVariables(m_variables);
}

void ChExternalDynamicsDAE::InjectConstraints(ChSystemDescriptor& descriptor) {
    for (auto& constraint : m_constraints)
        descriptor.InsertConstraint(&constraint);
}

void ChExternalDynamicsDAE::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (IsStiff()) {
        descriptor.InsertKRMBlock(&m_KRM);
    }
}

void ChExternalDynamicsDAE::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                           ChState& x,                // state vector, position part
                                           const unsigned int off_v,  // offset in v state vector
                                           ChStateDelta& v,           // state vector, speed part
                                           double& T                  // time
) {
    if (!IsActive())
        return;

    x.segment(off_x, m_ny) = m_y;
    v.segment(off_v, m_nyd) = m_yd;
    T = GetChTime();
}

void ChExternalDynamicsDAE::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                            const ChState& x,          // state vector, position part
                                            const unsigned int off_v,  // offset in v state vector
                                            const ChStateDelta& v,     // state vector, speed part
                                            const double T,            // time
                                            bool full_update           // perform complete update
) {
    if (!IsActive())
        return;

    // Important: set the internal states first, as they will be used in Update.
    m_y = x.segment(off_x, m_ny);
    m_yd = v.segment(off_v, m_nyd);

    Update(T, full_update);
}

void ChExternalDynamicsDAE::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a, m_nyd) = m_ydd;
}

void ChExternalDynamicsDAE::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    m_ydd = a.segment(off_a, m_nyd);
}

void ChExternalDynamicsDAE::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    for (unsigned int i = 0; i < m_nc; i++)
        L(off_L + i) = m_multipliers[i];
}

void ChExternalDynamicsDAE::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    for (unsigned int i = 0; i < m_nc; i++)
        m_multipliers[i] = L(off_L + i);
}

void ChExternalDynamicsDAE::IntStateIncrement(const unsigned int off_x,  // offset in state vector
                                              ChState& x_new,            // result, incremented state vector
                                              const ChState& x,          // current state vector
                                              const unsigned int off_v,  // offset in state vector increment
                                              const ChStateDelta& Dv     // state vector increment
) {
    if (m_ny == m_nyd) {
        ChPhysicsItem::IntStateIncrement(off_x, x_new, x, off_v, Dv);
        return;
    }

    ChVectorDynamic<> new_state(m_ny);
    IncrementState(x.segment(off_x, m_ny), Dv.segment(off_v, m_nyd), new_state);
    x_new.segment(off_x, m_ny) = new_state;
}

void ChExternalDynamicsDAE::IntStateGetIncrement(const unsigned int off_x,  // offset in x state vector
                                                 const ChState& x_new,      // state vector, final position part
                                                 const ChState& x,          // state vector, initial position part
                                                 const unsigned int off_v,  // offset in v state vector
                                                 ChStateDelta& Dv           // result, state vector increment
) {
    if (m_ny == m_nyd) {
        ChPhysicsItem::IntStateGetIncrement(off_x, x_new, x, off_v, Dv);
        return;
    }

    ChVectorDynamic<> increment(m_nyd);
    CalculateStateIncrement(x.segment(off_x, m_ny), x_new.segment(off_x, m_ny), increment);
    Dv.segment(off_v, m_nyd) = increment;
}

void ChExternalDynamicsDAE::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                              ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                              const double c           // a scaling factor
) {
    if (!IsActive())
        return;

    // Add forcing term for internal variables
    R.segment(off, m_nyd) += c * m_F;
}

void ChExternalDynamicsDAE::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                               ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                               const ChVectorDynamic<>& v,  // the v vector
                                               const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    if (InExplicitForm()) {
        R.segment(off, m_nyd) += c * v.segment(off, m_nyd);
        return;
    }

    ChVectorDynamic<> Mv(m_nyd);
    bool has_Mv = CalculateMassTimesVector(v.segment(off, m_nyd), Mv);
    if (has_Mv) {
        R.segment(off, m_nyd) += c * Mv;
    } else {
        R.segment(off, m_nyd) += c * m_M * v.segment(off, m_nyd);
    }
}

void ChExternalDynamicsDAE::IntLoadLumpedMass_Md(
    const unsigned int off,  // offset in Md vector
    ChVectorDynamic<>& Md,   // result: Md += c * diag(M), diagonal of the lumped mass matrix
    double& err,             // result: not touched if lumping does not introduce errors
    const double c           // a scaling factor
) {
    if (!IsActive())
        return;

    if (InExplicitForm()) {
        Md.segment(off, m_nyd).array() += c * 1.0;
    } else {
        Md.segment(off, m_nyd) += c * m_M.diagonal();
    }
}

void ChExternalDynamicsDAE::IntLoadConstraint_C(const unsigned int off,  // offset in Qc residual
                                                ChVectorDynamic<>& Qc,   // result: the Qc residual, Qc += c*C
                                                const double c,          // a scaling factor
                                                bool do_clamp,           // apply clamping to c*C?
                                                double recovery_clamp    // value for min/max clamping of c*C
) {
    for (unsigned int i = 0; i < m_nc; i++)
        Qc(off + i) += c * m_c[i];
}

void ChExternalDynamicsDAE::IntLoadConstraint_Ct(const unsigned int off,  // offset in Qc residual
                                                 ChVectorDynamic<>& Qc,   // result: the Qc residual, Qc += c*Ct
                                                 const double c           // a scaling factor
) {
    if (!IsRheonomous())
        return;

    Qc.segment(off, m_nc) += c * m_ct;
}

void ChExternalDynamicsDAE::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                const ChVectorDynamic<>& L,  // the L vector
                                                const double c               // a scaling factor

) {
    for (unsigned int i = 0; i < m_nc; i++)
        m_constraints[i].AddJacobianTransposedTimesScalarInto(R, L(off_L + i) * c);
}

void ChExternalDynamicsDAE::IntToDescriptor(
    const unsigned int off_v,    // offset in v, R
    const ChStateDelta& v,       // vector copied into the q 'unknowns' term of the variables
    const ChVectorDynamic<>& R,  // vector copied into the F 'force' term of the variables
    const unsigned int off_L,    // offset in L, Qc
    const ChVectorDynamic<>& L,  // vector copied into the L 'lagrangian ' term of the constraints
    const ChVectorDynamic<>& Qc  // vector copied into the Qb 'constraint' term of the constraints
) {
    if (!IsActive())
        return;

    m_variables->State() = v.segment(off_v, m_nyd);
    m_variables->Force() = R.segment(off_v, m_nyd);

    for (unsigned int i = 0; i < m_nc; i++) {
        m_constraints[i].SetLagrangeMultiplier(L(off_L + i));
        m_constraints[i].SetRightHandSide(Qc(off_L + i));
    }
}

void ChExternalDynamicsDAE::IntFromDescriptor(
    const unsigned int off_v,  // offset in v
    ChStateDelta& v,           // vector to where the q 'unknowns' term of the variables will be copied
    const unsigned int off_L,  // offset in L
    ChVectorDynamic<>& L       // vector to where L 'lagrangian ' term of the constraints will be copied
) {
    if (!IsActive())
        return;

    v.segment(off_v, m_nyd) = m_variables->State();

    for (unsigned int i = 0; i < m_nc; i++) {
        L(off_L + i) = m_constraints[i].GetLagrangeMultiplier();
    }
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsDAE::LoadConstraintJacobians() {
    for (unsigned int i = 0; i < m_nc; i++)
        m_constraints[i].Get_Cq_N(0) = m_Jc.row(i);
}

// -----------------------------------------------------------------------------

void ChExternalDynamicsDAE::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (!IsStiff())
        return;

    double time = GetSystem()->GetChTime();
    ComputeForceJacobian(time, Kfactor, Rfactor);

    // Recall to flip sign when loading J = Kfactor * dF/dy + Rfactor * dF/dyd
    if (InExplicitForm()) {
        m_KRM.GetMatrix() = Mfactor * ChMatrixDynamic<>::Identity(m_nyd, m_nyd) - m_J;
    } else {
        m_KRM.GetMatrix() = Mfactor * m_M - m_J;
    }
}

// -----------------------------------------------------------------------------

/*
void ChExternalDynamicsDAE::VariablesFbReset() {
}

void ChExternalDynamicsDAE::VariablesFbLoadForces(double factor) {
}

void ChExternalDynamicsDAE::VariablesQbLoadSpeed() {
}

void ChExternalDynamicsDAE::VariablesQbSetSpeed(double step) {
}

void ChExternalDynamicsDAE::VariablesFbIncrementMq() {
}

void ChExternalDynamicsDAE::VariablesQbIncrementPosition(double dt_step) {
}

void ChExternalDynamicsDAE::ConstraintsBiReset() {
}

void ChExternalDynamicsDAE::ConstraintsFbLoadForces(double factor) {
}
*/

// -----------------------------------------------------------------------------

void ChExternalDynamicsDAE::ComputeForceJacobian(double time, double alpha, double beta) {
    m_J.setZero();

    // Invoke Jacobian function
    bool has_jac = CalculateForceJacobians(time, m_y, m_yd, m_F, alpha, beta, m_J);

    // If Jacobian not provided, approximate with finite differences (here, m_ny = m_nyd)
    if (!has_jac) {
        ChVectorDynamic<> F(m_nyd);
        for (unsigned int i = 0; i < m_nyd; i++) {
            m_y(i) += m_FD_delta;
            CalculateForce(time, m_y, m_yd, F);
            m_J.col(i) += (F - m_F) * (alpha / m_FD_delta);
            m_y(i) -= m_FD_delta;
        }
        for (unsigned int i = 0; i < m_nyd; i++) {
            m_yd(i) += m_FD_delta;
            CalculateForce(time, m_y, m_yd, F);
            m_J.col(i) += (F - m_F) * (beta / m_FD_delta);
            m_yd(i) -= m_FD_delta;
        }
    }
}

void ChExternalDynamicsDAE::ComputeConstraintDerivative(double time) {
    // Invoke constraint derivative function
    bool has_ct = CalculateConstraintDerivative(time, m_y, m_c, m_ct);

    // If derivative function not provided, approximate with finite differences
    if (!has_ct) {
        ChVectorDynamic<> c(m_nc);
        time += m_FD_delta;
        CalculateConstraintViolation(time, m_y, c);
        m_ct = (c - m_c) / m_FD_delta;
        time -= m_FD_delta;
    }
}

}  // end namespace chrono
