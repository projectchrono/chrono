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
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsPlanetary)

ChShaftsPlanetary::ChShaftsPlanetary()
    : r1(1), r2(1), r3(1), torque_react(0), shaft1(NULL), shaft2(NULL), shaft3(NULL) {}

ChShaftsPlanetary::ChShaftsPlanetary(const ChShaftsPlanetary& other) : ChPhysicsItem(other) {
    r1 = other.r1;
    r2 = other.r2;
    r3 = other.r3;

    torque_react = other.torque_react;
    shaft1 = NULL;
    shaft2 = NULL;
    shaft3 = NULL;
}

bool ChShaftsPlanetary::Initialize(std::shared_ptr<ChShaft> mshaft1,  // first  shaft to join (carrier wheel)
                                   std::shared_ptr<ChShaft> mshaft2,  // second shaft to join (wheel)
                                   std::shared_ptr<ChShaft> mshaft3   // third  shaft to join (wheel)
                                   ) {
    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();
    ChShaft* mm3 = mshaft3.get();
    assert(mm1 && mm2 && mm3);
    assert(mm1 != mm2);
    assert(mm1 != mm3);
    assert(mm3 != mm2);
    assert((mm1->GetSystem() == mm2->GetSystem()) && (mm1->GetSystem() == mm3->GetSystem()));

    shaft1 = mm1;
    shaft2 = mm2;
    shaft3 = mm3;

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables(), &mm3->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsPlanetary::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsPlanetary::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                            ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                            const ChVectorDynamic<>& L,  // the L vector
                                            const double c               // a scaling factor
                                            ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsPlanetary::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
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

void ChShaftsPlanetary::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                        const ChStateDelta& v,
                                        const ChVectorDynamic<>& R,
                                        const unsigned int off_L,  // offset in L, Qc
                                        const ChVectorDynamic<>& L,
                                        const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsPlanetary::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                          ChStateDelta& v,
                                          const unsigned int off_L,  // offset in L
                                          ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

// SOLVER INTERFACES

void ChShaftsPlanetary::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsPlanetary::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsPlanetary::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsPlanetary::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsPlanetary::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsPlanetary::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftsPlanetary::ConstraintsLoadJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()->SetElement(0, 0, r1);
    constraint.Get_Cq_b()->SetElement(0, 0, r2);
    constraint.Get_Cq_c()->SetElement(0, 0, r3);
}

void ChShaftsPlanetary::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsPlanetary::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsPlanetary>();

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(r1);
    marchive << CHNVP(r2);
    marchive << CHNVP(r3);
    // marchive << CHNVP(shaft1); //***TODO*** serialize with shared ptr
    // marchive << CHNVP(shaft2); //***TODO*** serialize with shared ptr
    // marchive << CHNVP(shaft3); //***TODO*** serialize with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsPlanetary::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsPlanetary>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(r1);
    marchive >> CHNVP(r2);
    marchive >> CHNVP(r3);
    // marchive >> CHNVP(shaft1); //***TODO*** serialize with shared ptr
    // marchive >> CHNVP(shaft2); //***TODO*** serialize with shared ptr
    // marchive >> CHNVP(shaft3); //***TODO*** serialize with shared ptr
}

}  // end namespace chrono
