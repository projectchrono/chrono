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
    std::shared_ptr<ChShaft> shaft_1,    // first (input) shaft to join
    std::shared_ptr<ChShaft> shaft_2,    // second  (output) shaft to join
    std::shared_ptr<ChBodyFrame> truss,  // truss body (also carrier, if rotating as in planetary gearboxes)
    ChVector3d& dir_1,                   // direction of the first shaft on the gearbox truss
    ChVector3d& dir_2                    // direction of the second shaft on the gearbox truss
) {
    shaft1 = shaft_1.get();
    shaft2 = shaft_2.get();
    body = truss.get();
    shaft_dir1 = dir_1;
    shaft_dir2 = dir_2;

    assert(shaft1 && shaft2 && body);
    assert(shaft1 != shaft2);
    assert((shaft1->GetSystem() == shaft2->GetSystem()));

    constraint.SetVariables(&shaft_1->Variables(), &shaft_2->Variables(), &truss->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsGearboxAngled::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

void ChShaftsGearboxAngled::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                                ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                                const ChVectorDynamic<>& L,  // the L vector
                                                const double c               // a scaling factor
) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
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
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsGearboxAngled::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                            const ChStateDelta& v,
                                            const ChVectorDynamic<>& R,
                                            const unsigned int off_L,  // offset in L, Qc
                                            const ChVectorDynamic<>& L,
                                            const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));

    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftsGearboxAngled::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                              ChStateDelta& v,
                                              const unsigned int off_L,  // offset in L
                                              ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftsGearboxAngled::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsGearboxAngled::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsGearboxAngled::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    descriptor.InsertConstraint(&constraint);
}

void ChShaftsGearboxAngled::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftsGearboxAngled::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftsGearboxAngled::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftsGearboxAngled::LoadConstraintJacobians() {
    // compute jacobians
    constraint.Get_Cq_a()(0) = t0;
    constraint.Get_Cq_b()(0) = -1.0;

    // ChVector3d jacw = body->TransformDirectionParentToLocal(t0*shaft_dir1 - shaft_dir2);
    ChVector3d jacw = (t0 * shaft_dir1 - shaft_dir2);

    constraint.Get_Cq_c()(0) = 0;
    constraint.Get_Cq_c()(1) = 0;
    constraint.Get_Cq_c()(2) = 0;
    constraint.Get_Cq_c()(3) = jacw.x();
    constraint.Get_Cq_c()(4) = jacw.y();
    constraint.Get_Cq_c()(5) = jacw.z();
}

void ChShaftsGearboxAngled::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftsGearboxAngled::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsGearboxAngled>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(t0);
    archive_out << CHNVP(shaft_dir1);
    archive_out << CHNVP(shaft_dir2);
    archive_out << CHNVP(shaft1);  //// TODO  serialize with shared ptr
    archive_out << CHNVP(shaft2);  //// TODO  serialize with shared ptr
    archive_out << CHNVP(body);    //// TODO  serialize with shared ptr
}

void ChShaftsGearboxAngled::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsGearboxAngled>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(t0);
    archive_in >> CHNVP(shaft_dir1);
    archive_in >> CHNVP(shaft_dir2);
    archive_in >> CHNVP(shaft1);  //// TODO  serialize with shared ptr
    archive_in >> CHNVP(shaft2);  //// TODO  serialize with shared ptr
    archive_in >> CHNVP(body);    //// TODO  serialize with shared ptr
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables(), &body->Variables());
}

}  // end namespace chrono
