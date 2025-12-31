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

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChShaftBodyConstraint.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftBodyRotation)

ChShaftBodyRotation::ChShaftBodyRotation() : torque_react(0), shaft(NULL), body(NULL), shaft_dir(VECT_Z) {}

ChShaftBodyRotation::ChShaftBodyRotation(const ChShaftBodyRotation& other) : ChPhysicsItem(other) {
    torque_react = other.torque_react;
    shaft_dir = other.shaft_dir;
    shaft = NULL;
    body = NULL;
}

bool ChShaftBodyRotation::Initialize(std::shared_ptr<ChShaft> mshaft,
                                     std::shared_ptr<ChBodyFrame> mbody,
                                     const ChVector3d& mdir) {
    ChShaft* mm1 = mshaft.get();
    ChBodyFrame* mm2 = mbody.get();
    assert(mm1 && mm2);

    shaft = mm1;
    body = mm2;
    shaft_dir = Vnorm(mdir);

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    SetSystem(shaft->GetSystem());
    return true;
}

void ChShaftBodyRotation::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

void ChShaftBodyRotation::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -torque_react;
}

void ChShaftBodyRotation::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = -L(off_L);
}

void ChShaftBodyRotation::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                              ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                              const ChVectorDynamic<>& L,  ///< the L vector
                                              const double c               ///< a scaling factor
) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftBodyRotation::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                              ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                              const double c,            ///< a scaling factor
                                              bool do_clamp,             ///< apply clamping to c*C?
                                              double recovery_clamp      ///< value for min/max clamping of c*C
) {
    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftBodyRotation::IntToDescriptor(const unsigned int off_v,
                                          const ChStateDelta& v,
                                          const ChVectorDynamic<>& R,
                                          const unsigned int off_L,
                                          const ChVectorDynamic<>& L,
                                          const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));

    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftBodyRotation::IntFromDescriptor(const unsigned int off_v,
                                            ChStateDelta& v,
                                            const unsigned int off_L,
                                            ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftBodyRotation::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    descriptor.InsertConstraint(&constraint);
}

void ChShaftBodyRotation::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftBodyRotation::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftBodyRotation::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftBodyRotation::LoadConstraintJacobians() {
    // compute jacobians
    // ChVector3d jacw = body->TransformDirectionParentToLocal(shaft_dir);
    ChVector3d jacw = shaft_dir;

    constraint.Get_Cq_a()(0) = -1;

    constraint.Get_Cq_b()(0) = 0;
    constraint.Get_Cq_b()(1) = 0;
    constraint.Get_Cq_b()(2) = 0;
    constraint.Get_Cq_b()(3) = jacw.x();
    constraint.Get_Cq_b()(4) = jacw.y();
    constraint.Get_Cq_b()(5) = jacw.z();
}

void ChShaftBodyRotation::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = -constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftBodyRotation::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftBodyRotation>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(shaft_dir);
    archive_out << CHNVP(shaft);  //// TODO  serialize, with shared ptr
    archive_out << CHNVP(body);   //// TODO  serialize, with shared ptr
}

void ChShaftBodyRotation::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftBodyRotation>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(shaft_dir);
    archive_in >> CHNVP(shaft);  //// TODO  serialize, with shared ptr
    archive_in >> CHNVP(body);   //// TODO  serialize, with shared ptr
    constraint.SetVariables(&shaft->Variables(), &body->Variables());
}

//--------------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftBodyTranslation)

ChShaftBodyTranslation::ChShaftBodyTranslation()
    : force_react(0), shaft(NULL), body(NULL), shaft_dir(VECT_Z), shaft_pos(VNULL) {}

ChShaftBodyTranslation::ChShaftBodyTranslation(const ChShaftBodyTranslation& other) : ChPhysicsItem(other) {
    force_react = other.force_react;
    shaft_dir = other.shaft_dir;
    shaft_pos = other.shaft_pos;
    shaft = NULL;
    body = NULL;
}

bool ChShaftBodyTranslation::Initialize(std::shared_ptr<ChShaft> mshaft,
                                        std::shared_ptr<ChBodyFrame> mbody,
                                        const ChVector3d& mdir,
                                        const ChVector3d& mpos) {
    ChShaft* mm1 = mshaft.get();
    ChBodyFrame* mm2 = mbody.get();
    assert(mm1 && mm2);

    shaft = mm1;
    body = mm2;
    shaft_dir = Vnorm(mdir);
    shaft_pos = mpos;

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    SetSystem(shaft->GetSystem());
    return true;
}

void ChShaftBodyTranslation::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    // ...
}

void ChShaftBodyTranslation::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -force_react;
}

void ChShaftBodyTranslation::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    force_react = -L(off_L);
}

void ChShaftBodyTranslation::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                                 ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                                 const ChVectorDynamic<>& L,  ///< the L vector
                                                 const double c               ///< a scaling factor
) {
    constraint.AddJacobianTransposedTimesScalarInto(R, L(off_L) * c);
}

void ChShaftBodyTranslation::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                                 ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                                 const double c,            ///< a scaling factor
                                                 bool do_clamp,             ///< apply clamping to c*C?
                                                 double recovery_clamp      ///< value for min/max clamping of c*C
) {
    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftBodyTranslation::IntToDescriptor(const unsigned int off_v,
                                             const ChStateDelta& v,
                                             const ChVectorDynamic<>& R,
                                             const unsigned int off_L,
                                             const ChVectorDynamic<>& L,
                                             const ChVectorDynamic<>& Qc) {
    constraint.SetLagrangeMultiplier(L(off_L));

    constraint.SetRightHandSide(Qc(off_L));
}

void ChShaftBodyTranslation::IntFromDescriptor(const unsigned int off_v,
                                               ChStateDelta& v,
                                               const unsigned int off_L,
                                               ChVectorDynamic<>& L) {
    L(off_L) = constraint.GetLagrangeMultiplier();
}

void ChShaftBodyTranslation::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    descriptor.InsertConstraint(&constraint);
}

void ChShaftBodyTranslation::ConstraintsBiReset() {
    constraint.SetRightHandSide(0.);
}

void ChShaftBodyTranslation::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.SetRightHandSide(constraint.GetRightHandSide() + factor * res);
}

void ChShaftBodyTranslation::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftBodyTranslation::LoadConstraintJacobians() {
    // compute jacobians
    ChVector3d jacx = body->TransformDirectionLocalToParent(shaft_dir);
    ChVector3d jacw = Vcross(shaft_pos, shaft_dir);

    constraint.Get_Cq_a()(0) = -1.0;

    constraint.Get_Cq_b()(0) = jacx.x();
    constraint.Get_Cq_b()(1) = jacx.y();
    constraint.Get_Cq_b()(2) = jacx.z();
    constraint.Get_Cq_b()(3) = jacw.x();
    constraint.Get_Cq_b()(4) = jacw.y();
    constraint.Get_Cq_b()(5) = jacw.z();
}

void ChShaftBodyTranslation::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    force_react = -constraint.GetLagrangeMultiplier() * factor;
}

void ChShaftBodyTranslation::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftBodyRotation>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(shaft_dir);
    archive_out << CHNVP(shaft_pos);
    archive_out << CHNVP(shaft);  //// TODO  serialize, with shared ptr
    archive_out << CHNVP(body);   //// TODO  serialize, with shared ptr
}

void ChShaftBodyTranslation::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftBodyRotation>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(shaft_dir);
    archive_in >> CHNVP(shaft_pos);
    archive_in >> CHNVP(shaft);  //// TODO  serialize, with shared ptr
    archive_in >> CHNVP(body);   //// TODO  serialize, with shared ptr
    constraint.SetVariables(&shaft->Variables(), &body->Variables());
}

}  // end namespace chrono
