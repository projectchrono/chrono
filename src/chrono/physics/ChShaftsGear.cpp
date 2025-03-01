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
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsGear)

ChShaftsGear::ChShaftsGear() : ratio(1), torque_react(0), avoid_phase_drift(true), phase1(0), phase2(0), violation(0) {}

ChShaftsGear::ChShaftsGear(const ChShaftsGear& other) : ChShaftsCouple(other), violation(0) {
    ratio = other.ratio;
    torque_react = other.torque_react;
    avoid_phase_drift = other.avoid_phase_drift;
    phase1 = other.phase1;
    phase2 = other.phase2;
}

bool ChShaftsGear::Initialize(std::shared_ptr<ChShaft> shaft_1, std::shared_ptr<ChShaft> shaft_2) {
    // Parent initialization
    if (!ChShaftsCouple::Initialize(shaft_1, shaft_2))
        return false;

    phase1 = shaft1->GetPos();
    phase2 = shaft2->GetPos();

    constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables());

    SetSystem(shaft1->GetSystem());
    return true;
}

void ChShaftsGear::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(time, update_assets);

    // update class data
    violation = ratio * (shaft1->GetPos() - phase1) - 1.0 * (shaft2->GetPos() - phase2);
}

// STATE BOOKKEEPING FUNCTIONS

void ChShaftsGear::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsGear::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsGear::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                       ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                       const ChVectorDynamic<>& L,  // the L vector
                                       const double c               // a scaling factor
) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftsGear::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                       ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                       const double c,            // a scaling factor
                                       bool do_clamp,             // apply clamping to c*C?
                                       double recovery_clamp      // value for min/max clamping of c*C
) {
    double res = this->ratio * (this->shaft1->GetPos() - phase1) + -1.0 * (this->shaft2->GetPos() - phase2);
    if (!avoid_phase_drift)
        res = 0;

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsGear::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                   const ChStateDelta& v,
                                   const ChVectorDynamic<>& R,
                                   const unsigned int off_L,  // offset in L, Qc
                                   const ChVectorDynamic<>& L,
                                   const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));

    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftsGear::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                     ChStateDelta& v,
                                     const unsigned int off_L,  // offset in L
                                     ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftsGear::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    descriptor.InsertConstraint(&constraint);
}

void ChShaftsGear::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftsGear::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftsGear::LoadConstraintJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()(0) = ratio;
    constraint.Get_Cq_b()(0) = -1;
}

void ChShaftsGear::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsGear::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsGear>();

    // serialize parent class
    ChShaftsCouple::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(ratio);
    archive_out << CHNVP(avoid_phase_drift);
    archive_out << CHNVP(phase1);
    archive_out << CHNVP(phase2);
}

void ChShaftsGear::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsGear>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(ratio);
    archive_in >> CHNVP(avoid_phase_drift);
    archive_in >> CHNVP(phase1);
    archive_in >> CHNVP(phase2);
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables());
}

}  // end namespace chrono
