// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

ChShaftsGear::ChShaftsGear() : ratio(1), torque_react(0) {}

ChShaftsGear::ChShaftsGear(const ChShaftsGear& other) : ChShaftsCouple(other) {
    ratio = other.ratio;
    torque_react = other.torque_react;
}

bool ChShaftsGear::Initialize(std::shared_ptr<ChShaft> mshaft1, std::shared_ptr<ChShaft> mshaft2) {
    // Parent initialization
    if (!ChShaftsCouple::Initialize(mshaft1, mshaft2))
        return false;

    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    SetSystem(shaft1->GetSystem());
    return true;
}

void ChShaftsGear::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

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
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsGear::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                       ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                       const double c,            // a scaling factor
                                       bool do_clamp,             // apply clamping to c*C?
                                       double recovery_clamp      // value for min/max clamping of c*C
                                       ) {
    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = ChMin(ChMax(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsGear::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                   const ChStateDelta& v,
                                   const ChVectorDynamic<>& R,
                                   const unsigned int off_L,  // offset in L, Qc
                                   const ChVectorDynamic<>& L,
                                   const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsGear::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                     ChStateDelta& v,
                                     const unsigned int off_L,  // offset in L
                                     ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

// SOLVER INTERFACES

void ChShaftsGear::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsGear::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsGear::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsGear::ConstraintsLoadJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()->SetElement(0, 0, ratio);
    constraint.Get_Cq_b()->SetElement(0, 0, -1);
}

void ChShaftsGear::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsGear::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsGear>();

    // serialize parent class
    ChShaftsCouple::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(ratio);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsGear::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsGear>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(ratio);
}

}  // end namespace chrono
