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

    innershaft1 = std::make_shared<ChShaft>();
    innershaft2 = std::make_shared<ChShaft>();
    innerconstraint1 = std::make_shared<ChShaftsBody>();
    innerconstraint2 = std::make_shared<ChShaftsBody>();
}

ChLinkMotorRotationDriveline::ChLinkMotorRotationDriveline(const ChLinkMotorRotationDriveline& other)
    : ChLinkMotorRotation(other) {
    innershaft1 = other.innershaft1;
    innershaft2 = other.innershaft2;
    innerconstraint1 = other.innerconstraint1;
    innerconstraint2 = other.innerconstraint2;
}

ChLinkMotorRotationDriveline::~ChLinkMotorRotationDriveline() {}

void ChLinkMotorRotationDriveline::Setup() {
	if (innershaft1->IsActive()) {
        innershaft1->SetOffset_x(this->offset_x + 0);
        innershaft1->SetOffset_w(this->offset_w + 0);
	}
    if (innershaft2->IsActive()) {
        innershaft2->SetOffset_x(this->offset_x + 1);
        innershaft2->SetOffset_w(this->offset_w + 1);
    }
    int nc = mask->nconstr;
    innerconstraint1->SetOffset_L(this->offset_L + nc + 0);
    innerconstraint2->SetOffset_L(this->offset_L + nc + 1);
} 

void ChLinkMotorRotationDriveline::Update(double mytime, bool update_assets) {

     // Inherit parent class:
    ChLinkMotorRotation::Update(mytime, update_assets);

    if (Body1 && Body2) {
        // Note: we wrap Body1 and Body2 in shared_ptr with custom no-op destructors
        // so that the two objects are not destroyed when these shared_ptr go out of
        // scope (since Body1 and Body2 are still managed through other shared_ptr).
        std::shared_ptr<ChBodyFrame> b1(Body1, [](ChBodyFrame*) {});
        std::shared_ptr<ChBodyFrame> b2(Body2, [](ChBodyFrame*) {});
        if (innerconstraint1)
            innerconstraint1->Initialize(innershaft1, b1, VECT_Z);
        if (innerconstraint2)
            innerconstraint2->Initialize(innershaft2, b2, VECT_Z);
        // btw. The above initialization code could be moved in a place that is executed once per simulation

        // Update the direction of 1D-3D ChShaftBody constraints:
        ChVector<> abs_shaftdir = this->GetLinkAbsoluteCoords().TransformDirectionLocalToParent(VECT_Z);
        ChVector<> shaftdir_b1 = this->Body1->TransformDirectionParentToLocal(abs_shaftdir);
        ChVector<> shaftdir_b2 = this->Body2->TransformDirectionParentToLocal(abs_shaftdir);

        innerconstraint1->SetShaftDirection(shaftdir_b1);
        innerconstraint2->SetShaftDirection(shaftdir_b2);
    }

}


int ChLinkMotorRotationDriveline::GetDOF() {
    return 2 + ChLinkMotorRotation::GetDOF();
}

int ChLinkMotorRotationDriveline::GetDOC() {
    return 2 + ChLinkMotorRotation::GetDOC();
}

int ChLinkMotorRotationDriveline::GetDOC_c() {
    return 2 + ChLinkMotorRotation::GetDOC_c();
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
                                   const double T) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateScatter(off_x, x, off_v, v, T);

    innershaft1->IntStateScatter(off_x + 0, x, off_v + 0, v, T);
    innershaft2->IntStateScatter(off_x + 1, x, off_v + 1, v, T);
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

void ChLinkMotorRotationDriveline::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateGatherReactions(off_L, L);

    int nc = mask->nconstr;
    innerconstraint1->IntStateGatherReactions(off_L + nc + 0, L);
    innerconstraint2->IntStateGatherReactions(off_L + nc + 1, L);
}

void ChLinkMotorRotationDriveline::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntStateScatterReactions(off_L, L);

    int nc = mask->nconstr;
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

void ChLinkMotorRotationDriveline::IntLoadResidual_CqL(const unsigned int off_L,
                                       ChVectorDynamic<>& R,
                                       const ChVectorDynamic<>& L,
                                       const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadResidual_CqL(off_L, R, L, c);

    int nc = mask->nconstr;
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

    int nc = mask->nconstr;
    // the following always set zero C, so skip them ...
    //innerconstraint1->IntLoadConstraint_C(off_L + nc + 0, Qc, c, do_clamp, recovery_clamp);
    //innerconstraint2->IntLoadConstraint_C(off_L + nc + 1, Qc, c, do_clamp, recovery_clamp);
    // ...and compute custom violation C:
    double cnstr_rot_error =  this->GetMotorRot() - (this->innershaft1->GetPos() - this->innershaft2->GetPos());

    double cnstr_violation = c * cnstr_rot_error;

    if (do_clamp)
        cnstr_violation = ChMin(ChMax(cnstr_violation, -recovery_clamp), recovery_clamp);

    // lump violation in the 1st shaft-3D-1D constraints:
    Qc(off_L + nc + 0) += cnstr_violation;

}

void ChLinkMotorRotationDriveline::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    // First, inherit to parent class
    ChLinkMotorRotation::IntLoadConstraint_Ct(off_L, Qc, c);

    int nc = mask->nconstr;
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
    int nc = mask->nconstr;
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
    int nc = mask->nconstr;
    innerconstraint1->IntFromDescriptor(off_v, v, off_L + nc + 0, L);
    innerconstraint2->IntFromDescriptor(off_v, v, off_L + nc + 1, L);
}

//
//  SOLVER functions
//

void ChLinkMotorRotationDriveline::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // First, inherit to parent class
    ChLinkMotorRotation::InjectConstraints(mdescriptor);

    innerconstraint1->InjectConstraints(mdescriptor);
    innerconstraint2->InjectConstraints(mdescriptor);
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

void ChLinkMotorRotationDriveline::ConstraintsLoadJacobians() {
    // First, inherit to parent class
    ChLinkMotorRotation::ConstraintsLoadJacobians();

    innerconstraint1->ConstraintsLoadJacobians();
    innerconstraint2->ConstraintsLoadJacobians();
}

void ChLinkMotorRotationDriveline::ConstraintsFetch_react(double factor) {
    // First, inherit to parent class
    ChLinkMotorRotation::ConstraintsFetch_react(factor);

    innerconstraint1->ConstraintsFetch_react(factor);
    innerconstraint2->ConstraintsFetch_react(factor);
}

void ChLinkMotorRotationDriveline::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // First, inherit to parent class
    ChLinkMotorRotation::InjectVariables(mdescriptor);

    innershaft1->InjectVariables(mdescriptor);
    innershaft2->InjectVariables(mdescriptor);
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




void ChLinkMotorRotationDriveline::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationDriveline>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(innershaft1);
    marchive << CHNVP(innershaft2);
    marchive << CHNVP(innerconstraint1);
    marchive << CHNVP(innerconstraint2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationDriveline::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorRotationDriveline>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(innershaft1);
    marchive >> CHNVP(innershaft2);
    marchive >> CHNVP(innerconstraint1);
    marchive >> CHNVP(innerconstraint2);
}




}  // end namespace chrono
