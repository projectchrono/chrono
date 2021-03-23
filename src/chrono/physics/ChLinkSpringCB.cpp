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
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkSpringCB.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkSpringCB)

ChLinkSpringCB::ChLinkSpringCB()
    : m_force_fun(nullptr), m_ode_fun(nullptr), m_nstates(0), m_rest_length(0), m_force(0), m_variables(nullptr) {}

ChLinkSpringCB::ChLinkSpringCB(const ChLinkSpringCB& other) : ChLinkMarkers(other) {
    m_rest_length = other.m_rest_length;
    m_force = other.m_force;
    m_force_fun = other.m_force_fun;
    m_ode_fun = other.m_ode_fun;
    m_nstates = other.m_nstates;
    m_states = other.m_states;
    m_rhs = other.m_rhs;
    if (other.m_variables) {
        m_variables = new ChVariablesGenericDiagonalMass(other.m_variables->Get_ndof());
        (*m_variables) = (*other.m_variables);
    }
}

ChLinkSpringCB* ChLinkSpringCB::Clone() const {
    return new ChLinkSpringCB(*this);
}

ChLinkSpringCB::~ChLinkSpringCB() {
    delete m_variables;
}

void ChLinkSpringCB::RegisterODE(ODE* functor) {
    m_ode_fun = functor;
    m_nstates = functor->GetNumStates();
    m_states.resize(m_nstates);
    m_rhs.resize(m_nstates);
    functor->SetInitialConditions(m_states, this);
    m_variables = new ChVariablesGenericDiagonalMass(m_nstates);
    m_variables->GetMassDiagonal().Constant(m_nstates, 1);
}

void ChLinkSpringCB::Initialize(std::shared_ptr<ChBody> body1,
                                std::shared_ptr<ChBody> body2,
                                bool pos_are_relative,
                                ChVector<> pos1,
                                ChVector<> pos2,
                                bool auto_rest_length,
                                double rest_length) {
    // First, initialize as all links with markers.
    // In this case, create the two markers also!.
    ChLinkMarkers::Initialize(body1, body2, CSYSNORM);

    if (pos_are_relative) {
        marker1->Impose_Rel_Coord(ChCoordsys<>(pos1, QUNIT));
        marker2->Impose_Rel_Coord(ChCoordsys<>(pos2, QUNIT));
    } else {
        marker1->Impose_Abs_Coord(ChCoordsys<>(pos1, QUNIT));
        marker2->Impose_Abs_Coord(ChCoordsys<>(pos2, QUNIT));
    }

    ChVector<> AbsDist = marker1->GetAbsCoord().pos - marker2->GetAbsCoord().pos;
    dist = AbsDist.Length();

    m_rest_length = auto_rest_length ? dist : rest_length;
}

// -----------------------------------------------------------------------------

void ChLinkSpringCB::UpdateForces(double time) {
    // Allow the base class to update itself (possibly adding its own forces)
    ChLinkMarkers::UpdateForces(time);

    // Invoke the provided functor to evaluate force
    m_force = m_force_fun ? (*m_force_fun)(time, m_rest_length, dist, dist_dt, this) : 0;

    // Add to existing force.
    C_force += m_force * relM.pos.GetNormalized();
}

void ChLinkSpringCB::Update(double time, bool update_assets) {
    ChLinkMarkers::Update(time, update_assets);

    // Evaluate ODE right-hand side at current state
    if (m_ode_fun) {
        m_ode_fun->CalculateRHS(time, m_states, m_rhs, this);
    }
}

// -----------------------------------------------------------------------------

void ChLinkSpringCB::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                    ChState& x,                // state vector, position part
                                    const unsigned int off_v,  // offset in v state vector
                                    ChStateDelta& v,           // state vector, speed part
                                    double& T                  // time
) {
    if (!m_variables)
        return;

    x.segment(off_x, m_nstates).setZero();
    v.segment(off_v, m_nstates) = m_states;
    T = GetChTime();
}

void ChLinkSpringCB::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                     const ChState& x,          // state vector, position part
                                     const unsigned int off_v,  // offset in v state vector
                                     const ChStateDelta& v,     // state vector, speed part
                                     const double T,            // time
                                     bool full_update           // perform complete update
) {
    Update(T, full_update);

    if (!m_variables)
        return;

    m_states = v.segment(off_v, m_nstates);
}

void ChLinkSpringCB::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    if (!m_variables)
        return;

    a.segment(off_a, m_nstates) = m_rhs;
}

void ChLinkSpringCB::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    //// RADU: correct?!?
    // do nothing here
}

void ChLinkSpringCB::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                       ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                       const double c           // a scaling factor
) {
    ChLinkMarkers::IntLoadResidual_F(off, R, c);

    if (!m_variables)
        return;

    // add forcing term for internal states
    R.segment(off, m_nstates) += c * m_rhs;
}

void ChLinkSpringCB::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                        ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                        const ChVectorDynamic<>& v,  // the v vector
                                        const double c               // a scaling factor
) {
    if (!m_variables)
        return;

    R.segment(off, m_nstates) += c * v.segment(off, m_nstates);
}

void ChLinkSpringCB::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,  // offset in L, Qc
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!m_variables)
        return;

    m_variables->Get_qb() = v.segment(off_v, m_nstates);
    m_variables->Get_fb() = R.segment(off_v, m_nstates);
}

void ChLinkSpringCB::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                       ChStateDelta& v,
                                       const unsigned int off_L,  // offset in L
                                       ChVectorDynamic<>& L) {
    if (!m_variables)
        return;

    v.segment(off_v, m_nstates) = m_variables->Get_qb();
}

// -----------------------------------------------------------------------------

void ChLinkSpringCB::InjectVariables(ChSystemDescriptor& mdescriptor) {
    ChLinkMarkers::InjectVariables(mdescriptor);

    if (!m_variables)
        return;

    m_variables->SetDisabled(!IsActive());
    mdescriptor.InsertVariables(m_variables);
}

void ChLinkSpringCB::VariablesFbReset() {
    if (!m_variables)
        return;

    m_variables->Get_fb().setZero();
}

void ChLinkSpringCB::VariablesFbLoadForces(double factor) {
    if (!m_variables)
        return;

    m_variables->Get_fb() = m_rhs;
}

void ChLinkSpringCB::VariablesQbLoadSpeed() {
    if (!m_variables)
        return;

    m_variables->Get_qb() = m_states;
}

void ChLinkSpringCB::VariablesQbSetSpeed(double step) {
    if (!m_variables)
        return;

    m_states = m_variables->Get_qb();
}

void ChLinkSpringCB::VariablesFbIncrementMq() {
    if (!m_variables)
        return;

    m_variables->Compute_inc_Mb_v(m_variables->Get_fb(), m_variables->Get_qb());
}

void ChLinkSpringCB::VariablesQbIncrementPosition(double dt_step) {
    //// RADU: correct?
    // do nothing here
}

// -----------------------------------------------------------------------------

void ChLinkSpringCB::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkSpringCB>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(m_rest_length);
}

void ChLinkSpringCB::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkSpringCB>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(m_rest_length);
}

}  // end namespace chrono
