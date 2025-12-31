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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChLinkMotorRotationDriveline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationDriveline)

ChLinkMotorRotationDriveline::ChLinkMotorRotationDriveline() {
    this->c_rz = false;
    SetupLinkMask();

    innershaft1 = chrono_types::make_shared<ChShaft>();
    innershaft2 = chrono_types::make_shared<ChShaft>();
    innerconstraint1 = chrono_types::make_shared<ChShaftBodyRotation>();
    innerconstraint2 = chrono_types::make_shared<ChShaftBodyRotation>();
}

ChLinkMotorRotationDriveline::ChLinkMotorRotationDriveline(const ChLinkMotorRotationDriveline& other)
    : ChLinkMotorRotation(other) {
    innershaft1 = other.innershaft1;
    innershaft2 = other.innershaft2;
    innerconstraint1 = other.innerconstraint1;
    innerconstraint2 = other.innerconstraint2;
}

ChLinkMotorRotationDriveline::~ChLinkMotorRotationDriveline() {}

void ChLinkMotorRotationDriveline::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                              std::shared_ptr<ChBodyFrame> mbody2,
                                              ChFrame<> mabsframe) {
    this->Initialize(mbody1, mbody2, false, mabsframe, mabsframe);
}

void ChLinkMotorRotationDriveline::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                              std::shared_ptr<ChBodyFrame> mbody2,
                                              bool pos_are_relative,
                                              ChFrame<> mframe1,
                                              ChFrame<> mframe2) {
    ChLinkMotorRotation::Initialize(mbody1, mbody2, pos_are_relative, mframe1, mframe2);
    innerconstraint1->Initialize(innershaft1, mbody1, VECT_Z);
    innerconstraint2->Initialize(innershaft2, mbody2, VECT_Z);
}

void ChLinkMotorRotationDriveline::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                              std::shared_ptr<ChBodyFrame> mbody2,
                                              bool pos_are_relative,
                                              const ChVector3d& mpt1,
                                              const ChVector3d& mpt2,
                                              const ChVector3d& mnorm1,
                                              const ChVector3d& mnorm2) {
    ChLinkMotorRotation::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, mnorm1, mnorm2);
    innerconstraint1->Initialize(innershaft1, mbody1, VECT_Z);
    innerconstraint2->Initialize(innershaft2, mbody2, VECT_Z);
}

void ChLinkMotorRotationDriveline::Setup() {
    if (innershaft1->IsActive()) {
        innershaft1->SetOffset_x(this->offset_x + 0);
        innershaft1->SetOffset_w(this->offset_w + 0);
    }
    if (innershaft2->IsActive()) {
        innershaft2->SetOffset_x(this->offset_x + 1);
        innershaft2->SetOffset_w(this->offset_w + 1);
    }
    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->SetOffset_L(this->offset_L + nc + 0);
    innerconstraint2->SetOffset_L(this->offset_L + nc + 1);
}

void ChLinkMotorRotationDriveline::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorRotation::Update(time, update_assets);

    // Update the direction of 1D-3D ChShaftBody constraints:
    ChVector3d abs_shaftdir = this->GetFrame2Abs().TransformDirectionLocalToParent(VECT_Z);
    ChVector3d shaftdir_b1 = this->m_body1->TransformDirectionParentToLocal(abs_shaftdir);
    ChVector3d shaftdir_b2 = this->m_body2->TransformDirectionParentToLocal(abs_shaftdir);

    innerconstraint1->SetShaftDirection(shaftdir_b1);
    innerconstraint2->SetShaftDirection(shaftdir_b2);
}

unsigned int ChLinkMotorRotationDriveline::GetNumCoordsPosLevel() {
    return 2 + ChLinkMotorRotation::GetNumCoordsPosLevel();
}

unsigned int ChLinkMotorRotationDriveline::GetNumConstraints() {
    return 2 + ChLinkMotorRotation::GetNumConstraints();
}

unsigned int ChLinkMotorRotationDriveline::GetNumConstraintsBilateral() {
    return 2 + ChLinkMotorRotation::GetNumConstraintsBilateral();
}

void ChLinkMotorRotationDriveline::IntStateGather(const unsigned int off_x,
                                                  ChState& x,
                                                  const unsigned int off_v,
                                                  ChStateDelta& v,
                                                  double& T) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateGather(off_x, x, off_v, v, T);

    innershaft1->IntStateGather(off_x + 0, x, off_v + 0, v, T);
    innershaft2->IntStateGather(off_x + 1, x, off_v + 1, v, T);
}

void ChLinkMotorRotationDriveline::IntStateScatter(const unsigned int off_x,
                                                   const ChState& x,
                                                   const unsigned int off_v,
                                                   const ChStateDelta& v,
                                                   const double T,
                                                   bool full_update) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateScatter(off_x, x, off_v, v, T, full_update);

    innershaft1->IntStateScatter(off_x + 0, x, off_v + 0, v, T, full_update);
    innershaft2->IntStateScatter(off_x + 1, x, off_v + 1, v, T, full_update);

    Update(T, full_update);
}

void ChLinkMotorRotationDriveline::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateGatherAcceleration(off_a, a);

    innershaft1->IntStateScatterAcceleration(off_a + 0, a);
    innershaft2->IntStateScatterAcceleration(off_a + 1, a);
}

void ChLinkMotorRotationDriveline::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateScatterAcceleration(off_a, a);

    innershaft1->IntStateScatterAcceleration(off_a + 0, a);
    innershaft2->IntStateScatterAcceleration(off_a + 1, a);
}

void ChLinkMotorRotationDriveline::IntStateIncrement(const unsigned int off_x,
                                                     ChState& x_new,
                                                     const ChState& x,
                                                     const unsigned int off_v,
                                                     const ChStateDelta& Dv) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateIncrement(off_x, x_new, x, off_v, Dv);

    innershaft1->IntStateIncrement(off_x + 0, x_new, x, off_v + 0, Dv);
    innershaft2->IntStateIncrement(off_x + 1, x_new, x, off_v + 1, Dv);
}

void ChLinkMotorRotationDriveline::IntStateGetIncrement(const unsigned int off_x,
                                                        const ChState& x_new,
                                                        const ChState& x,
                                                        const unsigned int off_v,
                                                        ChStateDelta& Dv) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateGetIncrement(off_x, x_new, x, off_v, Dv);

    innershaft1->IntStateGetIncrement(off_x + 0, x_new, x, off_v + 0, Dv);
    innershaft2->IntStateGetIncrement(off_x + 1, x_new, x, off_v + 1, Dv);
}

void ChLinkMotorRotationDriveline::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateGatherReactions(off_L, L);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->IntStateGatherReactions(off_L + nc + 0, L);
    innerconstraint2->IntStateGatherReactions(off_L + nc + 1, L);
}

void ChLinkMotorRotationDriveline::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateScatterReactions(off_L, L);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->IntStateScatterReactions(off_L + nc + 0, L);
    innerconstraint2->IntStateScatterReactions(off_L + nc + 1, L);
}

void ChLinkMotorRotationDriveline::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadResidual_F(off, R, c);

    innershaft1->IntLoadResidual_F(off + 0, R, c);
    innershaft2->IntLoadResidual_F(off + 1, R, c);
}

void ChLinkMotorRotationDriveline::IntLoadResidual_Mv(const unsigned int off,
                                                      ChVectorDynamic<>& R,
                                                      const ChVectorDynamic<>& w,
                                                      const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadResidual_Mv(off, R, w, c);

    innershaft1->IntLoadResidual_Mv(off + 0, R, w, c);
    innershaft2->IntLoadResidual_Mv(off + 1, R, w, c);
}

void ChLinkMotorRotationDriveline::IntLoadLumpedMass_Md(const unsigned int off,
                                                        ChVectorDynamic<>& Md,
                                                        double& err,
                                                        const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadLumpedMass_Md(off, Md, err, c);

    innershaft1->IntLoadLumpedMass_Md(off + 0, Md, err, c);
    innershaft2->IntLoadLumpedMass_Md(off + 1, Md, err, c);
}

void ChLinkMotorRotationDriveline::IntLoadResidual_CqL(const unsigned int off_L,
                                                       ChVectorDynamic<>& R,
                                                       const ChVectorDynamic<>& L,
                                                       const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadResidual_CqL(off_L, R, L, c);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->IntLoadResidual_CqL(off_L + nc + 0, R, L, c);
    innerconstraint2->IntLoadResidual_CqL(off_L + nc + 1, R, L, c);
}

void ChLinkMotorRotationDriveline::IntLoadConstraint_C(const unsigned int off_L,
                                                       ChVectorDynamic<>& Qc,
                                                       const double c,
                                                       bool do_clamp,
                                                       double recovery_clamp) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);

    unsigned int nc = mask.GetNumConstraints();
    // the following always set zero C, so skip them ...
    // innerconstraint1->IntLoadConstraint_C(off_L + nc + 0, Qc, c, do_clamp, recovery_clamp);
    // innerconstraint2->IntLoadConstraint_C(off_L + nc + 1, Qc, c, do_clamp, recovery_clamp);
    // ...and compute custom violation C:
    double cnstr_rot_error = this->GetMotorAngle() - (this->innershaft1->GetPos() - this->innershaft2->GetPos());

    double cnstr_violation = c * cnstr_rot_error;

    if (do_clamp)
        cnstr_violation = std::min(std::max(cnstr_violation, -recovery_clamp), recovery_clamp);

    // lump violation in the 1st shaft-3D-1D constraints:
    Qc(off_L + nc + 0) += cnstr_violation;
}

void ChLinkMotorRotationDriveline::IntLoadConstraint_Ct(const unsigned int off_L,
                                                        ChVectorDynamic<>& Qc,
                                                        const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadConstraint_Ct(off_L, Qc, c);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->IntLoadConstraint_Ct(off_L + nc + 0, Qc, c);
    innerconstraint2->IntLoadConstraint_Ct(off_L + nc + 1, Qc, c);
}

void ChLinkMotorRotationDriveline::IntToDescriptor(const unsigned int off_v,
                                                   const ChStateDelta& v,
                                                   const ChVectorDynamic<>& R,
                                                   const unsigned int off_L,
                                                   const ChVectorDynamic<>& L,
                                                   const ChVectorDynamic<>& Qc) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntToDescriptor(off_v, v, R, off_L, L, Qc);

    innershaft1->IntToDescriptor(off_v, v, R, off_L, L, Qc);
    innershaft2->IntToDescriptor(off_v + 1, v, R, off_L, L, Qc);
    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->IntToDescriptor(off_v, v, R, off_L + nc + 0, L, Qc);
    innerconstraint2->IntToDescriptor(off_v, v, R, off_L + nc + 1, L, Qc);
}

void ChLinkMotorRotationDriveline::IntFromDescriptor(const unsigned int off_v,
                                                     ChStateDelta& v,
                                                     const unsigned int off_L,
                                                     ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntFromDescriptor(off_v, v, off_L, L);

    innershaft1->IntFromDescriptor(off_v, v, off_L, L);
    innershaft2->IntFromDescriptor(off_v + 1, v, off_L, L);
    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1->IntFromDescriptor(off_v, v, off_L + nc + 0, L);
    innerconstraint2->IntFromDescriptor(off_v, v, off_L + nc + 1, L);
}

//  SOLVER functions

void ChLinkMotorRotationDriveline::InjectConstraints(ChSystemDescriptor& descriptor) {
    // First, inherit to parent class
    ChLinkMotorRotation::InjectConstraints(descriptor);

    innerconstraint1->InjectConstraints(descriptor);
    innerconstraint2->InjectConstraints(descriptor);
}

void ChLinkMotorRotationDriveline::ConstraintsBiReset() {
    // First, inherit to parent class
    ChLinkMotorRotation::ConstraintsBiReset();

    innerconstraint1->ConstraintsBiReset();
    innerconstraint2->ConstraintsBiReset();
}

void ChLinkMotorRotationDriveline::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // First, inherit to parent class
    ChLinkMotorRotation::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

    innerconstraint1->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    innerconstraint2->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChLinkMotorRotationDriveline::ConstraintsBiLoad_Ct(double factor) {
    // First, inherit to parent class
    ChLinkMotorRotation::ConstraintsBiLoad_Ct(factor);

    innerconstraint1->ConstraintsBiLoad_Ct(factor);
    innerconstraint2->ConstraintsBiLoad_Ct(factor);
}

void ChLinkMotorRotationDriveline::LoadConstraintJacobians() {
    // First, inherit to parent class
    ChLinkMotorRotation::LoadConstraintJacobians();

    innerconstraint1->LoadConstraintJacobians();
    innerconstraint2->LoadConstraintJacobians();
}

void ChLinkMotorRotationDriveline::ConstraintsFetch_react(double factor) {
    // First, inherit to parent class
    ChLinkMotorRotation::ConstraintsFetch_react(factor);

    innerconstraint1->ConstraintsFetch_react(factor);
    innerconstraint2->ConstraintsFetch_react(factor);
}

void ChLinkMotorRotationDriveline::InjectVariables(ChSystemDescriptor& descriptor) {
    // First, inherit to parent class
    ChLinkMotorRotation::InjectVariables(descriptor);

    innershaft1->InjectVariables(descriptor);
    innershaft2->InjectVariables(descriptor);
}

void ChLinkMotorRotationDriveline::VariablesFbReset() {
    // First, inherit to parent class
    ChLinkMotorRotation::VariablesFbReset();

    innershaft1->VariablesFbReset();
    innershaft2->VariablesFbReset();
}

void ChLinkMotorRotationDriveline::VariablesFbLoadForces(double factor) {
    // First, inherit to parent class
    ChLinkMotorRotation::VariablesFbLoadForces(factor);

    innershaft1->VariablesFbLoadForces(factor);
    innershaft2->VariablesFbLoadForces(factor);
}

void ChLinkMotorRotationDriveline::VariablesFbIncrementMq() {
    // inherit parent class
    ChLinkMotorRotation::VariablesFbIncrementMq();

    innershaft1->VariablesFbIncrementMq();
    innershaft2->VariablesFbIncrementMq();
}

void ChLinkMotorRotationDriveline::VariablesQbLoadSpeed() {
    // First, inherit to parent class
    ChLinkMotorRotation::VariablesQbLoadSpeed();

    innershaft1->VariablesQbLoadSpeed();
    innershaft2->VariablesQbLoadSpeed();
}

void ChLinkMotorRotationDriveline::VariablesQbSetSpeed(double step) {
    // First, inherit to parent class
    ChLinkMotorRotation::VariablesQbSetSpeed(step);

    innershaft1->VariablesQbSetSpeed(step);
    innershaft2->VariablesQbSetSpeed(step);
}

void ChLinkMotorRotationDriveline::VariablesQbIncrementPosition(double step) {
    // First, inherit to parent class
    ChLinkMotorRotation::VariablesQbIncrementPosition(step);

    innershaft1->VariablesQbIncrementPosition(step);
    innershaft2->VariablesQbIncrementPosition(step);
}

void ChLinkMotorRotationDriveline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorRotationDriveline>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(innershaft1);
    archive_out << CHNVP(innershaft2);
    archive_out << CHNVP(innerconstraint1);
    archive_out << CHNVP(innerconstraint2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationDriveline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorRotationDriveline>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(innershaft1);
    archive_in >> CHNVP(innershaft2);
    archive_in >> CHNVP(innerconstraint1);
    archive_in >> CHNVP(innerconstraint2);
}

}  // end namespace chrono
