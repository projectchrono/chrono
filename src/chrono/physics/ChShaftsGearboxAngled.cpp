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
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsGearboxAngled)

ChShaftsGearboxAngled::ChShaftsGearboxAngled()
    : t0(1), torque_react(0), shaft1(NULL), shaft2(NULL), body(NULL), shaft_dir1(VECT_X), shaft_dir2(VECT_X) {
    SetTransmissionRatio(1);
}

ChShaftsGearboxAngled::ChShaftsGearboxAngled(const ChShaftsGearboxAngled& other) : ChPhysicsItem(other) {
    t0 = other.t0;
    shaft_dir1 = other.shaft_dir1;
    shaft_dir2 = other.shaft_dir2;

    torque_react = other.torque_react;
    shaft1 = NULL;
    shaft2 = NULL;
    body = NULL;
}

bool ChShaftsGearboxAngled::Initialize(
    std::shared_ptr<ChShaft> mshaft1,    // first (input) shaft to join
    std::shared_ptr<ChShaft> mshaft2,    // second  (output) shaft to join
    std::shared_ptr<ChBodyFrame> mbody,  // 3D body to use as truss (also carrier, if rotates as in planetary gearboxes)
    ChVector<>& mdir1,  // the direction of the first shaft on 3D body defining the gearbox truss
    ChVector<>& mdir2   // the direction of the first shaft on 3D body defining the gearbox truss
    ) {
    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();
    ChBodyFrame* mm3 = mbody.get();
    assert(mm1 && mm2 && mm3);
    assert(mm1 != mm2);
    assert((mm1->GetSystem() == mm2->GetSystem()));

    shaft1 = mm1;
    shaft2 = mm2;
    body = mm3;
    shaft_dir1 = mdir1;
    shaft_dir2 = mdir2;

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables(), &mm3->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsGearboxAngled::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsGearboxAngled::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                const ChVectorDynamic<>& L,  // the L vector
                                                const double c               // a scaling factor
                                                ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsGearboxAngled::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
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

void ChShaftsGearboxAngled::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                            const ChStateDelta& v,
                                            const ChVectorDynamic<>& R,
                                            const unsigned int off_L,  // offset in L, Qc
                                            const ChVectorDynamic<>& L,
                                            const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsGearboxAngled::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                              ChStateDelta& v,
                                              const unsigned int off_L,  // offset in L
                                              ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

// SOLVER INTERFACES

void ChShaftsGearboxAngled::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsGearboxAngled::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsGearboxAngled::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsGearboxAngled::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsGearboxAngled::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsGearboxAngled::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftsGearboxAngled::ConstraintsLoadJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()->SetElement(0, 0, t0);
    constraint.Get_Cq_b()->SetElement(0, 0, -1.0);

    // ChVector<> jacw = body->TransformDirectionParentToLocal(t0*shaft_dir1 - shaft_dir2);
    ChVector<> jacw = (t0 * shaft_dir1 - shaft_dir2);

    constraint.Get_Cq_c()->ElementN(0) = 0;
    constraint.Get_Cq_c()->ElementN(1) = 0;
    constraint.Get_Cq_c()->ElementN(2) = 0;
    constraint.Get_Cq_c()->ElementN(3) = jacw.x();
    constraint.Get_Cq_c()->ElementN(4) = jacw.y();
    constraint.Get_Cq_c()->ElementN(5) = jacw.z();
}

void ChShaftsGearboxAngled::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsGearboxAngled::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsGearboxAngled>();

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(t0);
    marchive << CHNVP(shaft_dir1);
    marchive << CHNVP(shaft_dir2);
    // marchive << CHNVP(shaft1); //***TODO*** serialize with shared ptr
    // marchive << CHNVP(shaft2); //***TODO*** serialize with shared ptr
    // marchive << CHNVP(body); //***TODO*** serialize with shared ptr
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsGearboxAngled::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsGearboxAngled>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(t0);
    marchive >> CHNVP(shaft_dir1);
    marchive >> CHNVP(shaft_dir2);
    // marchive >> CHNVP(shaft1); //***TODO*** serialize with shared ptr
    // marchive >> CHNVP(shaft2); //***TODO*** serialize with shared ptr
    // marchive >> CHNVP(body); //***TODO*** serialize with shared ptr
}

}  // end namespace chrono
