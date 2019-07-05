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
    
    this->c_x = false;
    SetupLinkMask();

    innershaft1lin = std::make_shared<ChShaft>();
    innershaft2lin = std::make_shared<ChShaft>();
    innershaft2rot = std::make_shared<ChShaft>();
    innerconstraint1lin = std::make_shared<ChShaftsBodyTranslation>();
    innerconstraint2lin = std::make_shared<ChShaftsBodyTranslation>(); 
    innerconstraint2rot = std::make_shared<ChShaftsBody>(); 
    shaft2_rotation_dir = VECT_X;
}

ChLinkMotorLinearDriveline::ChLinkMotorLinearDriveline(const ChLinkMotorLinearDriveline& other) : ChLinkMotorLinear(other) {
    innershaft1lin = other.innershaft1lin;
    innershaft2lin = other.innershaft2lin;
    innershaft2rot = other.innershaft2rot;
    innerconstraint1lin = other.innerconstraint1lin;
    innerconstraint2lin = other.innerconstraint2lin; 
    innerconstraint2rot = other.innerconstraint2rot; 
    shaft2_rotation_dir = other.shaft2_rotation_dir;
}

ChLinkMotorLinearDriveline::~ChLinkMotorLinearDriveline() {
    
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
    int nc = mask->nconstr;
    innerconstraint1lin->SetOffset_L(this->offset_L + nc + 0);
    innerconstraint2lin->SetOffset_L(this->offset_L + nc + 1);
    innerconstraint2rot->SetOffset_L(this->offset_L + nc + 2);
} 


void ChLinkMotorLinearDriveline::Update(double mytime, bool update_assets) {

     // Inherit parent class:
    ChLinkMotorLinear::Update(mytime, update_assets);

    if (Body1 && Body2) {
        // Note: we wrap Body1 and Body2 in shared_ptr with custom no-op destructors
        // so that the two objects are not destroyed when these shared_ptr go out of
        // scope (since Body1 and Body2 are still managed through other shared_ptr).
        std::shared_ptr<ChBodyFrame> b1(Body1, [](ChBodyFrame*) {});
        std::shared_ptr<ChBodyFrame> b2(Body2, [](ChBodyFrame*) {});
        if (innerconstraint1lin)
            innerconstraint1lin->Initialize(innershaft1lin, b1, VECT_X, VNULL);
        if (innerconstraint2lin)
            innerconstraint2lin->Initialize(innershaft2lin, b2, VECT_X, VNULL);
        if (innerconstraint2rot)
            innerconstraint2rot->Initialize(innershaft2rot, b2, VECT_X);
        // btw. The above initialization code could be moved in a place that is executed once per simulation

        // Update the direction of 1D-3D ChShaftBody constraints:
        ChVector<> abs_shaftdir = this->GetLinkAbsoluteCoords().TransformDirectionLocalToParent(VECT_X);
        ChVector<> shaftdir_b1 =    this->Body1->TransformDirectionParentToLocal(abs_shaftdir);
        ChVector<> shaftdir_b2 =    this->Body2->TransformDirectionParentToLocal(abs_shaftdir);
        ChVector<> shaftpos_b1 =   this->Body1->TransformPointParentToLocal(this->GetLinkAbsoluteCoords().pos);
        ChVector<> shaftpos_b2 =   this->Body2->TransformPointParentToLocal(this->GetLinkAbsoluteCoords().pos);
        ChVector<> abs_shaft2_rotation_dir = this->GetLinkAbsoluteCoords().TransformDirectionLocalToParent(this->shaft2_rotation_dir);
        ChVector<> shaftdir_b2rot = this->Body2->TransformDirectionParentToLocal(abs_shaft2_rotation_dir);

        innerconstraint1lin->SetShaftDirection(shaftdir_b1);
        innerconstraint1lin->SetShaftPos(shaftpos_b1);

        innerconstraint2lin->SetShaftDirection(shaftdir_b2);
        innerconstraint2lin->SetShaftPos(shaftpos_b2);

        innerconstraint2rot->SetShaftDirection(shaftdir_b2rot);

    }

}


int ChLinkMotorLinearDriveline::GetDOF() {
    return 3 + ChLinkMotorLinear::GetDOF();
}

int ChLinkMotorLinearDriveline::GetDOC() {
    return 3 + ChLinkMotorLinear::GetDOC();
}

int ChLinkMotorLinearDriveline::GetDOC_c() {
    return 3 + ChLinkMotorLinear::GetDOC_c();
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
                                   const double T) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateScatter(off_x, x, off_v, v, T);

    innershaft1lin->IntStateScatter(off_x + 0, x, off_v + 0, v, T);
    innershaft2lin->IntStateScatter(off_x + 1, x, off_v + 1, v, T);
    innershaft2rot->IntStateScatter(off_x + 2, x, off_v + 2, v, T);
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

void ChLinkMotorLinearDriveline::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateGatherReactions(off_L, L);

    int nc = mask->nconstr;
    innerconstraint1lin->IntStateGatherReactions(off_L + nc + 0, L);
    innerconstraint2lin->IntStateGatherReactions(off_L + nc + 1, L);
    innerconstraint2rot->IntStateGatherReactions(off_L + nc + 2, L);
}

void ChLinkMotorLinearDriveline::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntStateScatterReactions(off_L, L);

    int nc = mask->nconstr;
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

void ChLinkMotorLinearDriveline::IntLoadResidual_CqL(const unsigned int off_L,
                                       ChVectorDynamic<>& R,
                                       const ChVectorDynamic<>& L,
                                       const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadResidual_CqL(off_L, R, L, c);

    int nc = mask->nconstr;
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

    int nc = mask->nconstr;

    // compute custom violation C:
    double cnstr_pos_error1 =  this->GetMotorPos() - (this->innershaft1lin->GetPos());// - this->innershaft2lin->GetPos());
    double cnstr_violation1 = c * cnstr_pos_error1;
    if (do_clamp)
        cnstr_violation1 = ChMin(ChMax(cnstr_violation1, -recovery_clamp), recovery_clamp);
    Qc(off_L + nc + 0) += cnstr_violation1;

    // Always drive inner linear shaft 2 to zero
    // (hack! this is not exact if also the guide, part 2, moves fast?)
    double cnstr_violation2 = c * -this->innershaft2lin->GetPos();
    if (do_clamp)
        cnstr_violation2 = ChMin(ChMax(cnstr_violation2, -recovery_clamp), recovery_clamp);
    Qc(off_L + nc + 1) += cnstr_violation2;

    // Always drive inner rotational shaft 2 to zero
    // (hack! this is not exact if also the guide, part 2, rotates fast?)
    double cnstr_violation2r = c * -innershaft2rot->GetPos();
    if (do_clamp)
        cnstr_violation2r = ChMin(ChMax(cnstr_violation2r, -recovery_clamp), recovery_clamp);
    Qc(off_L + nc + 2) += cnstr_violation2r;

}

void ChLinkMotorLinearDriveline::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    // First, inherit to parent class
    ChLinkMotorLinear::IntLoadConstraint_Ct(off_L, Qc, c);

    int nc = mask->nconstr;
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
    int nc = mask->nconstr;
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
    int nc = mask->nconstr;
    innerconstraint1lin->IntFromDescriptor(off_v, v, off_L + nc + 0, L);
    innerconstraint2lin->IntFromDescriptor(off_v, v, off_L + nc + 1, L);
    innerconstraint2rot->IntFromDescriptor(off_v, v, off_L + nc + 2, L);
}

//
//  SOLVER functions
//

void ChLinkMotorLinearDriveline::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // First, inherit to parent class
    ChLinkMotorLinear::InjectConstraints(mdescriptor);

    innerconstraint1lin->InjectConstraints(mdescriptor);
    innerconstraint2lin->InjectConstraints(mdescriptor);
    innerconstraint2rot->InjectConstraints(mdescriptor);
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

void ChLinkMotorLinearDriveline::ConstraintsLoadJacobians() {
    // First, inherit to parent class
    ChLinkMotorLinear::ConstraintsLoadJacobians();

    innerconstraint1lin->ConstraintsLoadJacobians();
    innerconstraint2lin->ConstraintsLoadJacobians();
    innerconstraint2rot->ConstraintsLoadJacobians();
}

void ChLinkMotorLinearDriveline::ConstraintsFetch_react(double factor) {
    // First, inherit to parent class
    ChLinkMotorLinear::ConstraintsFetch_react(factor);

    innerconstraint1lin->ConstraintsFetch_react(factor);
    innerconstraint2lin->ConstraintsFetch_react(factor);
    innerconstraint2rot->ConstraintsFetch_react(factor);
}

void ChLinkMotorLinearDriveline::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // First, inherit to parent class
    ChLinkMotorLinear::InjectVariables(mdescriptor);

    innershaft1lin->InjectVariables(mdescriptor);
    innershaft2lin->InjectVariables(mdescriptor);
    innershaft2rot->InjectVariables(mdescriptor);
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




void ChLinkMotorLinearDriveline::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinearDriveline>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(innershaft1lin);
    marchive << CHNVP(innershaft2lin);
    marchive << CHNVP(innershaft2rot);
    marchive << CHNVP(innerconstraint1lin);
    marchive << CHNVP(innerconstraint2lin);
    marchive << CHNVP(innerconstraint2rot);
    marchive << CHNVP(shaft2_rotation_dir);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearDriveline::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorLinearDriveline>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(innershaft1lin);
    marchive >> CHNVP(innershaft2lin);
    marchive >> CHNVP(innershaft2rot);
    marchive >> CHNVP(innerconstraint1lin);
    marchive >> CHNVP(innerconstraint2lin);
    marchive >> CHNVP(innerconstraint2rot);
    marchive >> CHNVP(shaft2_rotation_dir);
}






}  // end namespace chrono
