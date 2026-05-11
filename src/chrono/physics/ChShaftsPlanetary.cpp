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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsPlanetary)

ChShaftsPlanetary::ChShaftsPlanetary()
    : m_r1(1),
      m_r2(1),
      m_r3(1),
      m_torque_react(0),
      m_shaft1(NULL),
      m_shaft2(NULL),
      m_shaft3(NULL),
      m_avoid_phase_drift(true),
      m_phase1(0),
      m_phase2(0),
      m_phase3(0),
      m_active(true) {}

ChShaftsPlanetary::ChShaftsPlanetary(const ChShaftsPlanetary& other) : ChPhysicsItem(other), m_active(true) {
    m_r1 = other.m_r1;
    m_r2 = other.m_r2;
    m_r3 = other.m_r3;

    m_torque_react = other.m_torque_react;
    m_shaft1 = NULL;
    m_shaft2 = NULL;
    m_shaft3 = NULL;

    m_avoid_phase_drift = other.m_avoid_phase_drift;
    m_phase1 = other.m_phase1;
    m_phase2 = other.m_phase2;
    m_phase3 = other.m_phase3;
}

bool ChShaftsPlanetary::Initialize(std::shared_ptr<ChShaft> shaft_1,  // first shaft to join (carrier wheel)
                                   std::shared_ptr<ChShaft> shaft_2,  // second shaft to join (wheel)
                                   std::shared_ptr<ChShaft> shaft_3   // third shaft to join (wheel)
) {
    m_shaft1 = shaft_1.get();
    m_shaft2 = shaft_2.get();
    m_shaft3 = shaft_3.get();

    assert(m_shaft1 && m_shaft2 && m_shaft3);
    assert(m_shaft1 != m_shaft2);
    assert(m_shaft1 != m_shaft3);
    assert(m_shaft3 != m_shaft2);
    assert((m_shaft1->GetSystem() == m_shaft2->GetSystem()) && (m_shaft1->GetSystem() == m_shaft3->GetSystem()));

    m_phase1 = m_shaft1->GetPos();
    m_phase2 = m_shaft2->GetPos();
    m_phase3 = m_shaft3->GetPos();

    m_constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables(), &shaft_3->Variables());

    SetSystem(m_shaft1->GetSystem());

    return true;
}

void ChShaftsPlanetary::Update(double time, UpdateFlags update_flags) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_flags);

    // update class data
    // ...
}

void ChShaftsPlanetary::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                            ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                            const ChVectorDynamic<>& L,  // the L vector
                                            const double c               // a scaling factor
) {
    if (!m_active)
        return;

    m_constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftsPlanetary::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                            ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                            const double c,            // a scaling factor
                                            const double c_vel,        // the scaling factor if the m_constraint is at speed level
                                            bool do_clamp,             // apply clamping to c*C?
                                            double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!m_active)
        return;

    double res = m_r1 * (m_shaft1->GetPos() - m_phase1) + m_r2 * (m_shaft2->GetPos() - m_phase2) + m_r3 * (m_shaft3->GetPos() - m_phase3);
    if (!m_avoid_phase_drift)
        res = 0;

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsPlanetary::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                        const ChStateDelta& v,
                                        const ChVectorDynamic<>& R,
                                        const unsigned int off_L,  // offset in L, Qc
                                        const ChVectorDynamic<>& L,
                                        const ChVectorDynamic<>& Qc) {
    if (!m_active)
        return;

    m_constraint.SetLagrangeMultiplier(L(off_L));
    m_constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftsPlanetary::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                          ChStateDelta& v,
                                          const unsigned int off_L,  // offset in L
                                          ChVectorDynamic<>& L) {
    if (!m_active)
        return;

    L(off_L) = m_constraint.GetLagrangeMultiplier();
}

void ChShaftsPlanetary::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = m_torque_react;
}

void ChShaftsPlanetary::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    m_torque_react = L(off_L);
}

void ChShaftsPlanetary::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!m_active)
        return;

    descriptor.InsertConstraint(&m_constraint);
}

void ChShaftsPlanetary::ConstraintsBiReset() {
    m_constraint.SetRightHandSide(0.);
}

void ChShaftsPlanetary::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!m_active)
        return;

    double res = 0;  // no residual

    m_constraint.SetRightHandSide(m_constraint.GetRightHandSide() + factor * res);
}

void ChShaftsPlanetary::ConstraintsBiLoad_Ct(double factor) {
    // nothing
}

void ChShaftsPlanetary::LoadConstraintJacobians() {
    // compute jacobians
    m_constraint.Get_Cq_a()(0) = m_r1;
    m_constraint.Get_Cq_b()(0) = m_r2;
    m_constraint.Get_Cq_c()(0) = m_r3;
}

void ChShaftsPlanetary::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    m_torque_react = m_constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsPlanetary::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsPlanetary>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(m_r1);
    archive_out << CHNVP(m_r2);
    archive_out << CHNVP(m_r3);
    archive_out << CHNVP(m_avoid_phase_drift);
    archive_out << CHNVP(m_phase1);
    archive_out << CHNVP(m_phase2);
    archive_out << CHNVP(m_phase3);
    archive_out << CHNVP(m_shaft1);  //// TODO  serialize with shared ptr
    archive_out << CHNVP(m_shaft2);  //// TODO  serialize with shared ptr
    archive_out << CHNVP(m_shaft3);  //// TODO  serialize with shared ptr
}

void ChShaftsPlanetary::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsPlanetary>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(m_r1);
    archive_in >> CHNVP(m_r2);
    archive_in >> CHNVP(m_r3);
    archive_in >> CHNVP(m_avoid_phase_drift);
    archive_in >> CHNVP(m_phase1);
    archive_in >> CHNVP(m_phase2);
    archive_in >> CHNVP(m_phase3);
    archive_in >> CHNVP(m_shaft1);  //// TODO  serialize with shared ptr
    archive_in >> CHNVP(m_shaft2);  //// TODO  serialize with shared ptr
    archive_in >> CHNVP(m_shaft3);  //// TODO  serialize with shared ptr

    m_constraint.SetVariables(&m_shaft1->Variables(), &m_shaft2->Variables(), &m_shaft3->Variables());
}

}  // end namespace chrono
