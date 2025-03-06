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
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsClutch)

ChShaftsClutch::ChShaftsClutch() : maxT(1), minT(-1), modulation(1), torque_react(0), active(true) {}

ChShaftsClutch::ChShaftsClutch(const ChShaftsClutch& other) : ChShaftsCouple(other), active(true) {
    maxT = other.maxT;
    minT = other.minT;
    modulation = other.modulation;
    torque_react = other.torque_react;
}

bool ChShaftsClutch::Initialize(std::shared_ptr<ChShaft> shaft_1, std::shared_ptr<ChShaft> shaft_2) {
    // parent class initialization
    if (!ChShaftsCouple::Initialize(shaft_1, shaft_2))
        return false;

    constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsClutch::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(time, update_assets);

    // update class data
    // ...
}

void ChShaftsClutch::SetTorqueLimit(double ml, double mu) {
    minT = ml;
    maxT = mu;
}

void ChShaftsClutch::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsClutch::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsClutch::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
) {
    if (!active)
        return;

    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftsClutch::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!active)
        return;

    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsClutch::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!active)
        return;

    double dt = system->GetStep();  //// TODO: check this if ever using variable-step integrators
    constraint.SetBoxedMinMax(dt * minT * modulation, dt * maxT * modulation);
}

void ChShaftsClutch::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,  // offset in L, Qc
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    if (!active)
        return;

    constraint.SetLagrangeMultiplier(L(off_L));
    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftsClutch::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                       ChStateDelta& v,
                                       const unsigned int off_L,  // offset in L
                                       ChVectorDynamic<>& L) {
    if (!active)
        return;

    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftsClutch::InjectConstraints(ChSystemDescriptor& descriptor) {
    if (!active)
        return;

    descriptor.InsertConstraint(&constraint);
}

void ChShaftsClutch::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftsClutch::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!active)
        return;

    double res = 0;  // no residual

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftsClutch::ConstraintsBiLoad_Ct(double factor) {
    // nothing
}

void ChShaftsClutch::ConstraintsFbLoadForces(double factor) {
    // no forces

    // compute jacobians
    double m_dt = factor;

    constraint.SetBoxedMinMax(m_dt * minT * modulation, m_dt * maxT * modulation);
}

void ChShaftsClutch::LoadConstraintJacobians() {
    constraint.Get_Cq_a()(0) = 1.0;
    constraint.Get_Cq_b()(0) = -1.0;
}

void ChShaftsClutch::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsClutch::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsClutch>();

    // serialize parent class
    ChShaftsCouple::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(maxT);
    archive_out << CHNVP(minT);
    archive_out << CHNVP(modulation);
}

void ChShaftsClutch::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsClutch>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(maxT);
    archive_in >> CHNVP(minT);
    archive_in >> CHNVP(modulation);
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables());
}

}  // end namespace chrono
