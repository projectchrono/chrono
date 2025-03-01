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
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsGearbox)

ChShaftsGearbox::ChShaftsGearbox()
    : r1(1), r2(1), r3(1), torque_react(0), shaft1(NULL), shaft2(NULL), body(NULL), shaft_dir(VECT_X) {
    SetTransmissionRatio(0.5);
}

ChShaftsGearbox::ChShaftsGearbox(const ChShaftsGearbox& other) : ChPhysicsItem(other) {
    r1 = other.r1;
    r2 = other.r2;
    r3 = other.r3;

    torque_react = other.torque_react;
    shaft1 = 0;
    shaft2 = 0;
    body = 0;

    shaft_dir = other.shaft_dir;
}

void ChShaftsGearbox::SetTransmissionRatio(double t0) {
    r3 = (1. - t0);
    r1 = t0;
    r2 = -1.0;
}

bool ChShaftsGearbox::Initialize(std::shared_ptr<ChShaft> shaft_1,    // first (input) shaft to join
                                 std::shared_ptr<ChShaft> shaft_2,    // second  (output) shaft to join
                                 std::shared_ptr<ChBodyFrame> truss,  // 3D body to use as truss
                                 ChVector3d& mdir                     // direction of the shaft on 3D body
) {
    shaft1 = shaft_1.get();
    shaft2 = shaft_2.get();
    body = truss.get();
    shaft_dir = mdir;

    assert(shaft1 && shaft2 && body);
    assert(shaft1 != shaft2);
    assert((shaft1->GetSystem() == shaft2->GetSystem()));

    constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables(), &truss->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsGearbox::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

void ChShaftsGearbox::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsGearbox::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsGearbox::IntLoadResidual_CqL(const unsigned int off_L,
                                          ChVectorDynamic<>& R,
                                          const ChVectorDynamic<>& L,
                                          const double c) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftsGearbox::IntLoadConstraint_C(const unsigned int off_L,
                                          ChVectorDynamic<>& Qc,
                                          const double c,
                                          bool do_clamp,
                                          double recovery_clamp) {
    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsGearbox::IntToDescriptor(const unsigned int off_v,
                                      const ChStateDelta& v,
                                      const ChVectorDynamic<>& R,
                                      const unsigned int off_L,
                                      const ChVectorDynamic<>& L,
                                      const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));

    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftsGearbox::IntFromDescriptor(const unsigned int off_v,
                                        ChStateDelta& v,
                                        const unsigned int off_L,
                                        ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftsGearbox::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    descriptor.InsertConstraint(&constraint);
}

void ChShaftsGearbox::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftsGearbox::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftsGearbox::LoadConstraintJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()(0) = r1;
    constraint.Get_Cq_b()(0) = r2;

    // ChVector3d jacw = body->TransformDirectionParentToLocal(shaft_dir);
    ChVector3d jacw = shaft_dir;

    constraint.Get_Cq_c()(0) = 0;
    constraint.Get_Cq_c()(1) = 0;
    constraint.Get_Cq_c()(2) = 0;
    constraint.Get_Cq_c()(3) = jacw.x() * r3;
    constraint.Get_Cq_c()(4) = jacw.y() * r3;
    constraint.Get_Cq_c()(5) = jacw.z() * r3;
}

void ChShaftsGearbox::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsGearbox::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsGearbox>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(r1);
    archive_out << CHNVP(r2);
    archive_out << CHNVP(r3);
    archive_out << CHNVP(shaft_dir);
    archive_out << CHNVP(shaft1);  //// TODO  serialize with shared ptr
    archive_out << CHNVP(shaft2);  //// TODO  serialize with shared ptr
    archive_out << CHNVP(body);    //// TODO  serialize with shared ptr
}

void ChShaftsGearbox::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsGearbox>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(r1);
    archive_in >> CHNVP(r2);
    archive_in >> CHNVP(r3);
    archive_in >> CHNVP(shaft_dir);
    archive_in >> CHNVP(shaft1);  //// TODO  serialize with shared ptr
    archive_in >> CHNVP(shaft2);  //// TODO  serialize with shared ptr
    archive_in >> CHNVP(body);    //// TODO  serialize with shared ptr
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables(), &body->Variables());
}

}  // end namespace chrono
