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

#include "chrono/physics/ChLinkMotorLinearDriveline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearDriveline)

ChLinkMotorLinearDriveline::ChLinkMotorLinearDriveline() {
    // the constraint along Z axis is guaranteed by the auxiliary ChShaftBodyRotation constraints
    this->c_z = false;
    SetupLinkMask();

    innershaft1lin = chrono_types::make_shared<ChShaft>();
    innershaft2lin = chrono_types::make_shared<ChShaft>();
    innershaft2rot = chrono_types::make_shared<ChShaft>();
    innerconstraint1lin = chrono_types::make_shared<ChShaftBodyTranslation>();
    innerconstraint2lin = chrono_types::make_shared<ChShaftBodyTranslation>();
    innerconstraint2rot = chrono_types::make_shared<ChShaftBodyRotation>();
    shaft2_rotation_dir = VECT_Z;
}

ChLinkMotorLinearDriveline::ChLinkMotorLinearDriveline(const ChLinkMotorLinearDriveline& other)
    : ChLinkMotorLinear(other) {
    innershaft1lin = other.innershaft1lin;
    innershaft2lin = other.innershaft2lin;
    innershaft2rot = other.innershaft2rot;
    innerconstraint1lin = other.innerconstraint1lin;
    innerconstraint2lin = other.innerconstraint2lin;
    innerconstraint2rot = other.innerconstraint2rot;
    shaft2_rotation_dir = other.shaft2_rotation_dir;
}

ChLinkMotorLinearDriveline::~ChLinkMotorLinearDriveline() {}

void ChLinkMotorLinearDriveline::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                            std::shared_ptr<ChBodyFrame> mbody2,
                                            ChFrame<> mabsframe) {
    this->Initialize(mbody1, mbody2, false, mabsframe, mabsframe);
}

void ChLinkMotorLinearDriveline::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                            std::shared_ptr<ChBodyFrame> mbody2,
                                            bool pos_are_relative,
                                            ChFrame<> mframe1,
                                            ChFrame<> mframe2) {
    ChLinkMotorLinear::Initialize(mbody1, mbody2, pos_are_relative, mframe1, mframe2);
    innerconstraint1lin->Initialize(innershaft1lin, mbody1, VECT_Z, VNULL);
    innerconstraint2lin->Initialize(innershaft2lin, mbody2, VECT_Z, VNULL);
    innerconstraint2rot->Initialize(innershaft2rot, mbody2, VECT_Z);
}

void ChLinkMotorLinearDriveline::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                            std::shared_ptr<ChBodyFrame> mbody2,
                                            bool pos_are_relative,
                                            const ChVector3d& mpt1,
                                            const ChVector3d& mpt2,
                                            const ChVector3d& mnorm1,
                                            const ChVector3d& mnorm2) {
    ChLinkMotorLinear::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, mnorm1, mnorm2);
    innerconstraint1lin->Initialize(innershaft1lin, mbody1, VECT_Z, VNULL);
    innerconstraint2lin->Initialize(innershaft2lin, mbody2, VECT_Z, VNULL);
    innerconstraint2rot->Initialize(innershaft2rot, mbody2, VECT_Z);
}

void ChLinkMotorLinearDriveline::Setup() {
    if (innershaft1lin->IsActive()) {
        innershaft1lin->SetOffset_x(this->offset_x + 0);
        innershaft1lin->SetOffset_w(this->offset_w + 0);
    }
    if (innershaft2lin->IsActive()) {
        innershaft2lin->SetOffset_x(this->offset_x + 1);
        innershaft2lin->SetOffset_w(this->offset_w + 1);
    }
    if (innershaft2rot->IsActive()) {
        innershaft2rot->SetOffset_x(this->offset_x + 2);
        innershaft2rot->SetOffset_w(this->offset_w + 2);
    }
    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->SetOffset_L(this->offset_L + nc + 0);
    innerconstraint2lin->SetOffset_L(this->offset_L + nc + 1);
    innerconstraint2rot->SetOffset_L(this->offset_L + nc + 2);
}

void ChLinkMotorLinearDriveline::Update(double time, bool update_assets) {
    // Inherit parent class:
    ChLinkMotorLinear::Update(time, update_assets);

    // Update the direction of 1D-3D ChShaftBody constraints:
    ChVector3d abs_shaftdir = this->GetFrame2Abs().TransformDirectionLocalToParent(VECT_Z);
    ChVector3d shaftdir_b1 = this->m_body1->TransformDirectionParentToLocal(abs_shaftdir);
    ChVector3d shaftdir_b2 = this->m_body2->TransformDirectionParentToLocal(abs_shaftdir);
    ChVector3d shaftpos_b1 = this->m_body1->TransformPointParentToLocal(this->GetFrame2Abs().GetCoordsys().pos);
    ChVector3d shaftpos_b2 = this->m_body2->TransformPointParentToLocal(this->GetFrame2Abs().GetCoordsys().pos);
    ChVector3d abs_shaft2_rotation_dir =
        this->GetFrame2Abs().TransformDirectionLocalToParent(this->shaft2_rotation_dir);
    ChVector3d shaftdir_b2rot = this->m_body2->TransformDirectionParentToLocal(abs_shaft2_rotation_dir);

    innerconstraint1lin->SetShaftDirection(shaftdir_b1);
    innerconstraint1lin->SetShaftPos(shaftpos_b1);

    innerconstraint2lin->SetShaftDirection(shaftdir_b2);
    innerconstraint2lin->SetShaftPos(shaftpos_b2);

    innerconstraint2rot->SetShaftDirection(shaftdir_b2rot);
}

unsigned int ChLinkMotorLinearDriveline::GetNumCoordsPosLevel() {
    return 3 + ChLinkMotorLinear::GetNumCoordsPosLevel();
}

unsigned int ChLinkMotorLinearDriveline::GetNumConstraints() {
    return 3 + ChLinkMotorLinear::GetNumConstraints();
}

unsigned int ChLinkMotorLinearDriveline::GetNumConstraintsBilateral() {
    return 3 + ChLinkMotorLinear::GetNumConstraintsBilateral();
}

void ChLinkMotorLinearDriveline::IntStateGather(const unsigned int off_x,
                                                ChState& x,
                                                const unsigned int off_v,
                                                ChStateDelta& v,
                                                double& T) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateGather(off_x, x, off_v, v, T);

    innershaft1lin->IntStateGather(off_x + 0, x, off_v + 0, v, T);
    innershaft2lin->IntStateGather(off_x + 1, x, off_v + 1, v, T);
    innershaft2rot->IntStateGather(off_x + 2, x, off_v + 2, v, T);
}

void ChLinkMotorLinearDriveline::IntStateScatter(const unsigned int off_x,
                                                 const ChState& x,
                                                 const unsigned int off_v,
                                                 const ChStateDelta& v,
                                                 const double T,
                                                 bool full_update) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateScatter(off_x, x, off_v, v, T, full_update);

    innershaft1lin->IntStateScatter(off_x + 0, x, off_v + 0, v, T, full_update);
    innershaft2lin->IntStateScatter(off_x + 1, x, off_v + 1, v, T, full_update);
    innershaft2rot->IntStateScatter(off_x + 2, x, off_v + 2, v, T, full_update);

    Update(T, full_update);
}

void ChLinkMotorLinearDriveline::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateGatherAcceleration(off_a, a);

    innershaft1lin->IntStateScatterAcceleration(off_a + 0, a);
    innershaft2lin->IntStateScatterAcceleration(off_a + 1, a);
    innershaft2rot->IntStateScatterAcceleration(off_a + 2, a);
}

void ChLinkMotorLinearDriveline::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateScatterAcceleration(off_a, a);

    innershaft1lin->IntStateScatterAcceleration(off_a + 0, a);
    innershaft2lin->IntStateScatterAcceleration(off_a + 1, a);
    innershaft2rot->IntStateScatterAcceleration(off_a + 2, a);
}

void ChLinkMotorLinearDriveline::IntStateIncrement(const unsigned int off_x,
                                                   ChState& x_new,
                                                   const ChState& x,
                                                   const unsigned int off_v,
                                                   const ChStateDelta& Dv) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateIncrement(off_x, x_new, x, off_v, Dv);

    innershaft1lin->IntStateIncrement(off_x + 0, x_new, x, off_v + 0, Dv);
    innershaft2lin->IntStateIncrement(off_x + 1, x_new, x, off_v + 1, Dv);
    innershaft2rot->IntStateIncrement(off_x + 2, x_new, x, off_v + 2, Dv);
}

void ChLinkMotorLinearDriveline::IntStateGetIncrement(const unsigned int off_x,
                                                      const ChState& x_new,
                                                      const ChState& x,
                                                      const unsigned int off_v,
                                                      ChStateDelta& Dv) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateGetIncrement(off_x, x_new, x, off_v, Dv);

    innershaft1lin->IntStateGetIncrement(off_x + 0, x_new, x, off_v + 0, Dv);
    innershaft2lin->IntStateGetIncrement(off_x + 1, x_new, x, off_v + 1, Dv);
    innershaft2rot->IntStateGetIncrement(off_x + 2, x_new, x, off_v + 2, Dv);
}

void ChLinkMotorLinearDriveline::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateGatherReactions(off_L, L);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->IntStateGatherReactions(off_L + nc + 0, L);
    innerconstraint2lin->IntStateGatherReactions(off_L + nc + 1, L);
    innerconstraint2rot->IntStateGatherReactions(off_L + nc + 2, L);
}

void ChLinkMotorLinearDriveline::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateScatterReactions(off_L, L);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->IntStateScatterReactions(off_L + nc + 0, L);
    innerconstraint2lin->IntStateScatterReactions(off_L + nc + 1, L);
    innerconstraint2rot->IntStateScatterReactions(off_L + nc + 2, L);
}

void ChLinkMotorLinearDriveline::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadResidual_F(off, R, c);

    innershaft1lin->IntLoadResidual_F(off + 0, R, c);
    innershaft2lin->IntLoadResidual_F(off + 1, R, c);
    innershaft2rot->IntLoadResidual_F(off + 2, R, c);
}

void ChLinkMotorLinearDriveline::IntLoadResidual_Mv(const unsigned int off,
                                                    ChVectorDynamic<>& R,
                                                    const ChVectorDynamic<>& w,
                                                    const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadResidual_Mv(off, R, w, c);

    innershaft1lin->IntLoadResidual_Mv(off + 0, R, w, c);
    innershaft2lin->IntLoadResidual_Mv(off + 1, R, w, c);
    innershaft2rot->IntLoadResidual_Mv(off + 2, R, w, c);
}

void ChLinkMotorLinearDriveline::IntLoadLumpedMass_Md(const unsigned int off,
                                                      ChVectorDynamic<>& Md,
                                                      double& err,
                                                      const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadLumpedMass_Md(off, Md, err, c);

    innershaft1lin->IntLoadLumpedMass_Md(off + 0, Md, err, c);
    innershaft2lin->IntLoadLumpedMass_Md(off + 1, Md, err, c);
    innershaft2rot->IntLoadLumpedMass_Md(off + 2, Md, err, c);
}

void ChLinkMotorLinearDriveline::IntLoadResidual_CqL(const unsigned int off_L,
                                                     ChVectorDynamic<>& R,
                                                     const ChVectorDynamic<>& L,
                                                     const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadResidual_CqL(off_L, R, L, c);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->IntLoadResidual_CqL(off_L + nc + 0, R, L, c);
    innerconstraint2lin->IntLoadResidual_CqL(off_L + nc + 1, R, L, c);
    innerconstraint2rot->IntLoadResidual_CqL(off_L + nc + 2, R, L, c);
}

void ChLinkMotorLinearDriveline::IntLoadConstraint_C(const unsigned int off_L,
                                                     ChVectorDynamic<>& Qc,
                                                     const double c,
                                                     bool do_clamp,
                                                     double recovery_clamp) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);

    unsigned int nc = mask.GetNumConstraints();

    // compute custom violation C:
    double cnstr_pos_error1 =
        this->GetMotorPos() - (this->innershaft1lin->GetPos());  // - this->innershaft2lin->GetPos());
    double cnstr_violation1 = c * cnstr_pos_error1;
    if (do_clamp)
        cnstr_violation1 = std::min(std::max(cnstr_violation1, -recovery_clamp), recovery_clamp);
    Qc(off_L + nc + 0) += cnstr_violation1;

    // Always drive inner linear shaft 2 to zero
    // (hack! this is not exact if also the guide, part 2, moves fast?)
    double cnstr_violation2 = c * -this->innershaft2lin->GetPos();
    if (do_clamp)
        cnstr_violation2 = std::min(std::max(cnstr_violation2, -recovery_clamp), recovery_clamp);
    Qc(off_L + nc + 1) += cnstr_violation2;

    // Always drive inner rotational shaft 2 to zero
    // (hack! this is not exact if also the guide, part 2, rotates fast?)
    double cnstr_violation2r = c * -innershaft2rot->GetPos();
    if (do_clamp)
        cnstr_violation2r = std::min(std::max(cnstr_violation2r, -recovery_clamp), recovery_clamp);
    Qc(off_L + nc + 2) += cnstr_violation2r;
}

void ChLinkMotorLinearDriveline::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadConstraint_Ct(off_L, Qc, c);

    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->IntLoadConstraint_Ct(off_L + nc + 0, Qc, c);
    innerconstraint2lin->IntLoadConstraint_Ct(off_L + nc + 1, Qc, c);
    innerconstraint2rot->IntLoadConstraint_Ct(off_L + nc + 2, Qc, c);
}

void ChLinkMotorLinearDriveline::IntToDescriptor(const unsigned int off_v,
                                                 const ChStateDelta& v,
                                                 const ChVectorDynamic<>& R,
                                                 const unsigned int off_L,
                                                 const ChVectorDynamic<>& L,
                                                 const ChVectorDynamic<>& Qc) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntToDescriptor(off_v, v, R, off_L, L, Qc);

    innershaft1lin->IntToDescriptor(off_v, v, R, off_L, L, Qc);
    innershaft2lin->IntToDescriptor(off_v + 1, v, R, off_L, L, Qc);
    innershaft2rot->IntToDescriptor(off_v + 2, v, R, off_L, L, Qc);
    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->IntToDescriptor(off_v, v, R, off_L + nc + 0, L, Qc);
    innerconstraint2lin->IntToDescriptor(off_v, v, R, off_L + nc + 1, L, Qc);
    innerconstraint2rot->IntToDescriptor(off_v, v, R, off_L + nc + 2, L, Qc);
}

void ChLinkMotorLinearDriveline::IntFromDescriptor(const unsigned int off_v,
                                                   ChStateDelta& v,
                                                   const unsigned int off_L,
                                                   ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntFromDescriptor(off_v, v, off_L, L);

    innershaft1lin->IntFromDescriptor(off_v, v, off_L, L);
    innershaft2lin->IntFromDescriptor(off_v + 1, v, off_L, L);
    innershaft2rot->IntFromDescriptor(off_v + 2, v, off_L, L);
    unsigned int nc = mask.GetNumConstraints();
    innerconstraint1lin->IntFromDescriptor(off_v, v, off_L + nc + 0, L);
    innerconstraint2lin->IntFromDescriptor(off_v, v, off_L + nc + 1, L);
    innerconstraint2rot->IntFromDescriptor(off_v, v, off_L + nc + 2, L);
}

//
//  SOLVER functions
//

void ChLinkMotorLinearDriveline::InjectConstraints(ChSystemDescriptor& descriptor) {
    // First, inherit to parent class
    ChLinkMotorLinear::InjectConstraints(descriptor);

    innerconstraint1lin->InjectConstraints(descriptor);
    innerconstraint2lin->InjectConstraints(descriptor);
    innerconstraint2rot->InjectConstraints(descriptor);
}

void ChLinkMotorLinearDriveline::ConstraintsBiReset() {
    // First, inherit to parent class
    ChLinkMotorLinear::ConstraintsBiReset();

    innerconstraint1lin->ConstraintsBiReset();
    innerconstraint2lin->ConstraintsBiReset();
    innerconstraint2rot->ConstraintsBiReset();
}

void ChLinkMotorLinearDriveline::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // First, inherit to parent class
    ChLinkMotorLinear::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

    innerconstraint1lin->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    innerconstraint2lin->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
    innerconstraint2rot->ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);
}

void ChLinkMotorLinearDriveline::ConstraintsBiLoad_Ct(double factor) {
    // First, inherit to parent class
    ChLinkMotorLinear::ConstraintsBiLoad_Ct(factor);

    innerconstraint1lin->ConstraintsBiLoad_Ct(factor);
    innerconstraint2lin->ConstraintsBiLoad_Ct(factor);
    innerconstraint2rot->ConstraintsBiLoad_Ct(factor);
}

void ChLinkMotorLinearDriveline::LoadConstraintJacobians() {
    // First, inherit to parent class
    ChLinkMotorLinear::LoadConstraintJacobians();

    innerconstraint1lin->LoadConstraintJacobians();
    innerconstraint2lin->LoadConstraintJacobians();
    innerconstraint2rot->LoadConstraintJacobians();
}

void ChLinkMotorLinearDriveline::ConstraintsFetch_react(double factor) {
    // First, inherit to parent class
    ChLinkMotorLinear::ConstraintsFetch_react(factor);

    innerconstraint1lin->ConstraintsFetch_react(factor);
    innerconstraint2lin->ConstraintsFetch_react(factor);
    innerconstraint2rot->ConstraintsFetch_react(factor);
}

void ChLinkMotorLinearDriveline::InjectVariables(ChSystemDescriptor& descriptor) {
    // First, inherit to parent class
    ChLinkMotorLinear::InjectVariables(descriptor);

    innershaft1lin->InjectVariables(descriptor);
    innershaft2lin->InjectVariables(descriptor);
    innershaft2rot->InjectVariables(descriptor);
}

void ChLinkMotorLinearDriveline::VariablesFbReset() {
    // First, inherit to parent class
    ChLinkMotorLinear::VariablesFbReset();

    innershaft1lin->VariablesFbReset();
    innershaft2lin->VariablesFbReset();
    innershaft2rot->VariablesFbReset();
}

void ChLinkMotorLinearDriveline::VariablesFbLoadForces(double factor) {
    // First, inherit to parent class
    ChLinkMotorLinear::VariablesFbLoadForces(factor);

    innershaft1lin->VariablesFbLoadForces(factor);
    innershaft2lin->VariablesFbLoadForces(factor);
    innershaft2rot->VariablesFbLoadForces(factor);
}

void ChLinkMotorLinearDriveline::VariablesFbIncrementMq() {
    // inherit parent class
    ChLinkMotorLinear::VariablesFbIncrementMq();

    innershaft1lin->VariablesFbIncrementMq();
    innershaft2lin->VariablesFbIncrementMq();
    innershaft2rot->VariablesFbIncrementMq();
}

void ChLinkMotorLinearDriveline::VariablesQbLoadSpeed() {
    // First, inherit to parent class
    ChLinkMotorLinear::VariablesQbLoadSpeed();

    innershaft1lin->VariablesQbLoadSpeed();
    innershaft2lin->VariablesQbLoadSpeed();
    innershaft2rot->VariablesQbLoadSpeed();
}

void ChLinkMotorLinearDriveline::VariablesQbSetSpeed(double step) {
    // First, inherit to parent class
    ChLinkMotorLinear::VariablesQbSetSpeed(step);

    innershaft1lin->VariablesQbSetSpeed(step);
    innershaft2lin->VariablesQbSetSpeed(step);
    innershaft2rot->VariablesQbSetSpeed(step);
}

void ChLinkMotorLinearDriveline::VariablesQbIncrementPosition(double step) {
    // First, inherit to parent class
    ChLinkMotorLinear::VariablesQbIncrementPosition(step);

    innershaft1lin->VariablesQbIncrementPosition(step);
    innershaft2lin->VariablesQbIncrementPosition(step);
    innershaft2rot->VariablesQbIncrementPosition(step);
}

void ChLinkMotorLinearDriveline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLinkMotorLinearDriveline>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(innershaft1lin);
    archive_out << CHNVP(innershaft2lin);
    archive_out << CHNVP(innershaft2rot);
    archive_out << CHNVP(innerconstraint1lin);
    archive_out << CHNVP(innerconstraint2lin);
    archive_out << CHNVP(innerconstraint2rot);
    archive_out << CHNVP(shaft2_rotation_dir);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearDriveline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLinkMotorLinearDriveline>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(innershaft1lin);
    archive_in >> CHNVP(innershaft2lin);
    archive_in >> CHNVP(innershaft2rot);
    archive_in >> CHNVP(innerconstraint1lin);
    archive_in >> CHNVP(innerconstraint2lin);
    archive_in >> CHNVP(innerconstraint2rot);
    archive_in >> CHNVP(shaft2_rotation_dir);
}

}  // end namespace chrono
