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

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChShaftsBody.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsBody)

ChShaftsBody::ChShaftsBody() : torque_react(0), shaft(NULL), body(NULL), shaft_dir(VECT_Z) {}

ChShaftsBody::ChShaftsBody(const ChShaftsBody& other) : ChPhysicsItem(other) {
    torque_react = other.torque_react;
    shaft_dir = other.shaft_dir;
    shaft = NULL;
    body = NULL;
}

bool ChShaftsBody::Initialize(std::shared_ptr<ChShaft> mshaft,
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

void ChShaftsBody::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

void ChShaftsBody::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -torque_react;
}

void ChShaftsBody::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = -L(off_L);
}

void ChShaftsBody::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                       ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                       const ChVectorDynamic<>& L,  ///< the L vector
                                       const double c               ///< a scaling factor
) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsBody::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
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

void ChShaftsBody::IntToDescriptor(const unsigned int off_v,
                                   const ChStateDelta& v,
                                   const ChVectorDynamic<>& R,
                                   const unsigned int off_L,
                                   const ChVectorDynamic<>& L,
                                   const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsBody::IntFromDescriptor(const unsigned int off_v,
                                     ChStateDelta& v,
                                     const unsigned int off_L,
                                     ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

void ChShaftsBody::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsBody::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsBody::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsBody::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftsBody::ConstraintsLoadJacobians() {
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

void ChShaftsBody::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = -constraint.Get_l_i() * factor;
}

void ChShaftsBody::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsBody>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(shaft_dir);
    archive_out << CHNVP(shaft);  //// TODO  serialize, with shared ptr
    archive_out << CHNVP(body);   //// TODO  serialize, with shared ptr
}

void ChShaftsBody::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsBody>();

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
CH_FACTORY_REGISTER(ChShaftsBodyTranslation)

ChShaftsBodyTranslation::ChShaftsBodyTranslation()
    : force_react(0), shaft(NULL), body(NULL), shaft_dir(VECT_Z), shaft_pos(VNULL) {}

ChShaftsBodyTranslation::ChShaftsBodyTranslation(const ChShaftsBodyTranslation& other) : ChPhysicsItem(other) {
    force_react = other.force_react;
    shaft_dir = other.shaft_dir;
    shaft_pos = other.shaft_pos;
    shaft = NULL;
    body = NULL;
}

bool ChShaftsBodyTranslation::Initialize(std::shared_ptr<ChShaft> mshaft,
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

void ChShaftsBodyTranslation::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    // ...
}

void ChShaftsBodyTranslation::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = -force_react;
}

void ChShaftsBodyTranslation::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    force_react = -L(off_L);
}

void ChShaftsBodyTranslation::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                                  ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                                  const ChVectorDynamic<>& L,  ///< the L vector
                                                  const double c               ///< a scaling factor
) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsBodyTranslation::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
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

void ChShaftsBodyTranslation::IntToDescriptor(const unsigned int off_v,
                                              const ChStateDelta& v,
                                              const ChVectorDynamic<>& R,
                                              const unsigned int off_L,
                                              const ChVectorDynamic<>& L,
                                              const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsBodyTranslation::IntFromDescriptor(const unsigned int off_v,
                                                ChStateDelta& v,
                                                const unsigned int off_L,
                                                ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

void ChShaftsBodyTranslation::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsBodyTranslation::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsBodyTranslation::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsBodyTranslation::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftsBodyTranslation::ConstraintsLoadJacobians() {
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

void ChShaftsBodyTranslation::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    force_react = -constraint.Get_l_i() * factor;
}

void ChShaftsBodyTranslation::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsBody>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(shaft_dir);
    archive_out << CHNVP(shaft_pos);
    archive_out << CHNVP(shaft);  //// TODO  serialize, with shared ptr
    archive_out << CHNVP(body);   //// TODO  serialize, with shared ptr
}

void ChShaftsBodyTranslation::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsBody>();

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
