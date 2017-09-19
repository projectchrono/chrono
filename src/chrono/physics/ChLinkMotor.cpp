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

#include "chrono/physics/ChLinkMotor.h"

namespace chrono {

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChLinkMotor)  NO! ABSTRACT!

void ChLinkMotor::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotor>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotor::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotor>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
}


// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChLinkMotorLinear)  NO! ABSTRACT!

ChLinkMotorLinear::ChLinkMotorLinear()  {

    this->SetGuideConstraint(GuideConstraint::PRISMATIC);

    mpos = 0;
    mpos_dt = 0;
    mpos_dtdt = 0;
}

ChLinkMotorLinear::ChLinkMotorLinear(const ChLinkMotorLinear& other) : ChLinkMotor(other) {
   
    mpos = other.mpos;
    mpos_dt = other.mpos_dt;
    mpos_dtdt = other.mpos_dtdt;
}

ChLinkMotorLinear::~ChLinkMotorLinear() {
    
}

void ChLinkMotorLinear::SetGuideConstraint(bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz)  {
    
        this->c_y =  mc_y;
        this->c_z =  mc_z;
        this->c_rx = mc_rx;
        this->c_ry = mc_ry;
        this->c_rz = mc_rz;
        SetupLinkMask();
}

void ChLinkMotorLinear::SetGuideConstraint(const GuideConstraint mconstraint)  {
    if (mconstraint == GuideConstraint::FREE) {
        this->c_y =  false;
        this->c_z =  false;
        this->c_rx = false;
        this->c_ry = false;
        this->c_rz = false;
        SetupLinkMask();
    }
    if (mconstraint == GuideConstraint::PRISMATIC) {
        this->c_y =  true;
        this->c_z =  true;
        this->c_rx = true;
        this->c_ry = true;
        this->c_rz = true;
        SetupLinkMask();
    }
    if (mconstraint == GuideConstraint::SPHERICAL) {
        this->c_y =  true;
        this->c_z =  true;
        this->c_rx = false;
        this->c_ry = false;
        this->c_rz = false;
        SetupLinkMask();
    }
}


void ChLinkMotorLinear::Update(double mytime, bool update_assets) {
    
    // Inherit parent class:
    ChLinkMotor::Update(mytime, update_assets);

    // compute aux data for future reference (istantaneous pos speed accel)
    ChFrameMoving<> aframe1 = ChFrameMoving<>(this->frame1) >> (ChFrameMoving<>)(*this->Body1);
    ChFrameMoving<> aframe2 = ChFrameMoving<>(this->frame2) >> (ChFrameMoving<>)(*this->Body2);
    ChFrameMoving<> aframe12;
    aframe2.TransformParentToLocal(aframe1, aframe12);
    this->mpos = aframe12.GetPos().x();
    this->mpos_dt = aframe12.GetPos_dt().x();
    this->mpos_dtdt = aframe12.GetPos_dtdt().x();
}


void ChLinkMotorLinear::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinear>();

    // serialize parent class
    ChLinkMotor::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinear::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorLinear>();

    // deserialize parent class
    ChLinkMotor::ArchiveIN(marchive);

    // deserialize all member data:
}





// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearPosition)

ChLinkMotorLinearPosition::ChLinkMotorLinearPosition() {
    
    // default motion function : a ramp
    this->f_pos = std::make_shared<ChFunction_Ramp>(
        0.0,   // default y(0)
        1.0    // default dy/dx , i.e.   1 [m/s]
        );
    
    pos_offset = 0;
}

ChLinkMotorLinearPosition::ChLinkMotorLinearPosition(const ChLinkMotorLinearPosition& other) : ChLinkMotorLinear(other) {
   this->f_pos = other.f_pos;
   this->pos_offset = other.pos_offset;
}

ChLinkMotorLinearPosition::~ChLinkMotorLinearPosition() {
    
}


void ChLinkMotorLinearPosition::Update(double mytime, bool update_assets) {
    
    // Inherit parent class:
    ChLinkMotorLinear::Update(mytime, update_assets);

    this->f_pos->Update(mytime); // call callbacks if any

    // Add the time-dependent term in residual C as 
    //   C = d_error - d_setpoint - d_offset
    // with d_error = x_pos_A- x_pos_B, and d_setpoint = x(t)
    C->ElementN(0) = this->mpos  - this->f_pos->Get_y(this->GetChTime()) - this->pos_offset;
}

void ChLinkMotorLinearPosition::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {

    double mCt = - this->f_pos->Get_y_dx(this->GetChTime());
    if (mask->Constr_N(0).IsActive()) {
        Qc(off_L + 0) += c * mCt;
    }
}


void ChLinkMotorLinearPosition::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = - this->f_pos->Get_y_dx(this->GetChTime());
    if (mask->Constr_N(0).IsActive()) {
            mask->Constr_N(0).Set_b_i(mask->Constr_N(0).Get_b_i() + factor * mCt);
    }
}



void ChLinkMotorLinearPosition::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinearPosition>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_pos);
    marchive << CHNVP(pos_offset);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearPosition::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorLinearPosition>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_pos);
    marchive >> CHNVP(pos_offset);
}





// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearSpeed)

ChLinkMotorLinearSpeed::ChLinkMotorLinearSpeed() {

    this->variable.GetMass()(0,0) = 1.0;
    this->variable.GetInvMass()(0,0) = 1.0;

    this->f_speed = std::make_shared<ChFunction_Const>(1.0);

    this->pos_offset = 0;

    this->aux_dt = 0; // used for integrating speed, = pos
    this->aux_dtdt = 0;

    this->avoid_position_drift = true;
}

ChLinkMotorLinearSpeed::ChLinkMotorLinearSpeed(const ChLinkMotorLinearSpeed& other) : ChLinkMotorLinear(other) {
    
    this->variable = other.variable;

    this->f_speed = other.f_speed;

    this->pos_offset =other.pos_offset;

    this->aux_dt = other.aux_dt; 
    this->aux_dtdt = other.aux_dtdt;
    
    this->avoid_position_drift = other.avoid_position_drift;
}

ChLinkMotorLinearSpeed::~ChLinkMotorLinearSpeed() {
    
}


void ChLinkMotorLinearSpeed::Update(double mytime, bool update_assets) {
    
    // Inherit parent class:
    ChLinkMotorLinear::Update(mytime, update_assets);

    this->f_speed->Update(mytime); // call callbacks if any

    // Add the time-dependent term in residual C as 
    //   C = d_error - d_setpoint - d_offset
    // with d_error = x_pos_A- x_pos_B, and d_setpoint = x(t)
    if (this->avoid_position_drift) 
        C->ElementN(0) = this->mpos  - aux_dt - this->pos_offset; 
    else
        C->ElementN(0) = 0.0;
}

void ChLinkMotorLinearSpeed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {

    double mCt = - this->f_speed->Get_y(this->GetChTime());
    if (mask->Constr_N(0).IsActive()) {
        Qc(off_L + 0) += c * mCt;
    }
}


void ChLinkMotorLinearSpeed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = - this->f_speed->Get_y(this->GetChTime());
    if (mask->Constr_N(0).IsActive()) {
            mask->Constr_N(0).Set_b_i(mask->Constr_N(0).Get_b_i() + factor * mCt);
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMotorLinearSpeed::IntStateGather(const unsigned int off_x,  // offset in x state vector
                             ChState& x,                // state vector, position part
                             const unsigned int off_v,  // offset in v state vector
                             ChStateDelta& v,           // state vector, speed part
                             double& T                  // time
                             ) {
    x(off_x) = 0;//aux;
    v(off_v) = aux_dt;
    T = GetChTime();
}

void ChLinkMotorLinearSpeed::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                              const ChState& x,          // state vector, position part
                              const unsigned int off_v,  // offset in v state vector
                              const ChStateDelta& v,     // state vector, speed part
                              const double T             // time
                              ) {
    //aux = x(off_x);
    aux_dt = v(off_v);
}

void ChLinkMotorLinearSpeed::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = aux_dtdt;
}

void ChLinkMotorLinearSpeed::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    aux_dtdt = a(off_a);
}

void ChLinkMotorLinearSpeed::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                const double c           // a scaling factor
                                ) {
    double imposed_speed = this->f_speed->Get_y(this->GetChTime());
    R(off) +=  imposed_speed * c;
}

void ChLinkMotorLinearSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                 ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  // the w vector
                                 const double c               // a scaling factor
                                 ) {
    R(off) += c * 1.0 * w(off);
}

void ChLinkMotorLinearSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                              const ChStateDelta& v,
                              const ChVectorDynamic<>& R,
                              const unsigned int off_L,  // offset in L, Qc
                              const ChVectorDynamic<>& L,
                              const ChVectorDynamic<>& Qc) {
    // inherit parent
    ChLinkMotorLinear::IntToDescriptor( off_v, v, R, off_L,  L, Qc);

    this->variable.Get_qb()(0, 0) = v(off_v);
    this->variable.Get_fb()(0, 0) = R(off_v);
}

void ChLinkMotorLinearSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                ChStateDelta& v,
                                const unsigned int off_L,  // offset in L
                                ChVectorDynamic<>& L) {
    //inherit parent
    ChLinkMotorLinear::IntFromDescriptor(off_v, v, off_L, L);

    v(off_v) = this->variable.Get_qb()(0, 0);
}

////
void ChLinkMotorLinearSpeed::InjectVariables(ChSystemDescriptor& mdescriptor) {
    variable.SetDisabled(!IsActive());

    mdescriptor.InsertVariables(&variable);
}

void ChLinkMotorLinearSpeed::VariablesFbReset() {
    variable.Get_fb().FillElem(0.0);
}

void ChLinkMotorLinearSpeed::VariablesFbLoadForces(double factor) {
    
    double imposed_speed = this->f_speed->Get_y(this->GetChTime());
    variable.Get_fb().ElementN(0) += imposed_speed * factor;
}

void ChLinkMotorLinearSpeed::VariablesFbIncrementMq() {
    variable.Compute_inc_Mb_v(variable.Get_fb(), variable.Get_qb());
}

void ChLinkMotorLinearSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.Get_qb().SetElement(0, 0, aux_dt);
}

void ChLinkMotorLinearSpeed::VariablesQbSetSpeed(double step) {
    double old_dt = aux_dt;

    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.Get_qb().GetElement(0, 0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}



void ChLinkMotorLinearSpeed::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinearSpeed>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_speed);
    marchive << CHNVP(pos_offset);
    marchive << CHNVP(avoid_position_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearSpeed::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorLinearSpeed>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_speed);
    marchive >> CHNVP(pos_offset);
    marchive >> CHNVP(avoid_position_drift);
}





// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorLinearForce)

ChLinkMotorLinearForce::ChLinkMotorLinearForce() {
    
    this->c_x = false;
    SetupLinkMask();

    this->f_force = std::make_shared<ChFunction_Const>(0.0);
}

ChLinkMotorLinearForce::ChLinkMotorLinearForce(const ChLinkMotorLinearForce& other) : ChLinkMotorLinear(other) {
   this->f_force = other.f_force;
}

ChLinkMotorLinearForce::~ChLinkMotorLinearForce() {
    
}

void ChLinkMotorLinearForce::Update(double mytime, bool update_assets) {

     // Inherit parent class:
    ChLinkMotorLinear::Update(mytime, update_assets);

    this->f_force->Update(mytime); // call callbacks if any
}

void ChLinkMotorLinearForce::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant force
    double mF = this->f_force->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_force = aframe2.GetA().Matr_x_Vect(ChVector<>(  mF,0,0 ) );
    Vector mbody_force;
    Vector mbody_torque;

    if (Body2->Variables().IsActive()) {
        Body2->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
        R.PasteSumVector(- mbody_force * c, Body2->Variables().GetOffset(), 0);
        R.PasteSumVector(Body2->TransformDirectionParentToLocal(mbody_torque) * -c,
                            Body2->Variables().GetOffset() + 3, 0);
    }

    if (Body1->Variables().IsActive()) {
        Body1->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
        R.PasteSumVector(  mbody_force * c, Body1->Variables().GetOffset(), 0);
        R.PasteSumVector(Body1->TransformDirectionParentToLocal(mbody_torque) * c,
                            Body1->Variables().GetOffset() + 3, 0);
    }
}


void ChLinkMotorLinearForce::ConstraintsFbLoadForces(double factor) {
    // compute instant force
    double mF = this->f_force->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_force = aframe2.GetA().Matr_x_Vect(ChVector<>(  mF,0,0 ) );
    Vector mbody_force;
    Vector mbody_torque;
     Body2->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
    Body2->Variables().Get_fb().PasteSumVector(- mbody_force * factor, 0, 0);
    Body2->Variables().Get_fb().PasteSumVector(Body2->TransformDirectionParentToLocal(mbody_torque) * -factor, 3, 0);

    Body1->To_abs_forcetorque(m_abs_force,
                                    aframe1.GetPos(),            // absolute application point is always marker1
                                    false,                       // from abs. space
                                    mbody_force, mbody_torque);  // resulting force-torque, both in abs coords
    Body1->Variables().Get_fb().PasteSumVector( mbody_force * factor, 0, 0);
    Body1->Variables().Get_fb().PasteSumVector(Body1->TransformDirectionParentToLocal(mbody_torque) * factor, 3, 0);
}

void ChLinkMotorLinearForce::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorLinearForce>();

    // serialize parent class
    ChLinkMotorLinear::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_force);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorLinearForce::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorLinearForce>();

    // deserialize parent class
    ChLinkMotorLinear::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_force);
}



// -----------------------------------------------------------------------------

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
/*
        innerconstraint1lin->SetShaftDirection(shaftdir_b1);
        innerconstraint1lin->SetShaftPos(shaftpos_b1);

        innerconstraint2lin->SetShaftDirection(shaftdir_b2);
        innerconstraint2lin->SetShaftPos(shaftpos_b2);

        innerconstraint2rot->SetShaftDirection(shaftdir_b2rot);
*/
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




// -----------------------------------------------------------------------------
//
// Rotational motors




// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChLinkMotorRotation)  NO! ABSTRACT!

ChLinkMotorRotation::ChLinkMotorRotation()  {

    this->SetSpindleConstraint(SpindleConstraint::REVOLUTE);

    mrot = 0;
    mrot_dt = 0;
    mrot_dtdt = 0;
}

ChLinkMotorRotation::ChLinkMotorRotation(const ChLinkMotorRotation& other) : ChLinkMotor(other) {
   
    mrot = other.mrot;
    mrot_dt = other.mrot_dt;
    mrot_dtdt = other.mrot_dtdt;
}

ChLinkMotorRotation::~ChLinkMotorRotation() {
    
}

void ChLinkMotorRotation::SetSpindleConstraint(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry)  {
    
        this->c_x =  mc_x;
        this->c_y =  mc_y;
        this->c_z =  mc_z;
        this->c_rx = mc_rx;
        this->c_ry = mc_ry;
        SetupLinkMask();
}

void ChLinkMotorRotation::SetSpindleConstraint(const SpindleConstraint mconstraint)  {
    if (mconstraint == SpindleConstraint::FREE) {
        this->c_x =  false;
        this->c_y =  false;
        this->c_z =  false;
        this->c_rx = false;
        this->c_ry = false;
        SetupLinkMask();
    }
    if (mconstraint == SpindleConstraint::REVOLUTE) {
        this->c_x =  true;
        this->c_y =  true;
        this->c_z =  true;
        this->c_rx = true;
        this->c_ry = true;
        SetupLinkMask();
    }
    if (mconstraint == SpindleConstraint::CYLINDRICAL) {
        this->c_x =  true;
        this->c_y =  true;
        this->c_z =  false;
        this->c_rx = true;
        this->c_ry = true;
        SetupLinkMask();
    }
    if (mconstraint == SpindleConstraint::OLDHAM) {
        this->c_x =  false;
        this->c_y =  false;
        this->c_z =  false;
        this->c_rx = true;
        this->c_ry = true;
        SetupLinkMask();
    }
}


void ChLinkMotorRotation::Update(double mytime, bool update_assets) {
    
    // Inherit parent class:
    ChLinkMotor::Update(mytime, update_assets);

    // compute aux data for future reference (istantaneous pos speed accel)
    ChFrameMoving<> aframe1 = ChFrameMoving<>(this->frame1) >> (ChFrameMoving<>)(*this->Body1);
    ChFrameMoving<> aframe2 = ChFrameMoving<>(this->frame2) >> (ChFrameMoving<>)(*this->Body2);
    ChFrameMoving<> aframe12;
    aframe2.TransformParentToLocal(aframe1, aframe12);

    // multi-turn rotation code
    double last_totrot = this->mrot;
    double last_rot = remainder(last_totrot, CH_C_2PI);
    double last_turns = last_totrot - last_rot;
    double new_rot  = remainder(aframe12.GetRot().Q_to_Rotv().z(), CH_C_2PI);
    this->mrot = last_turns + new_rot;
    if (fabs(new_rot + CH_C_2PI - last_rot) < fabs(new_rot-last_rot) )
        this->mrot = last_turns + new_rot + CH_C_2PI;
    if (fabs(new_rot - CH_C_2PI - last_rot) < fabs(new_rot-last_rot) )
        this->mrot = last_turns + new_rot - CH_C_2PI;
    
    this->mrot_dt = aframe12.GetWvel_loc().z();
    this->mrot_dtdt = aframe12.GetWacc_loc().z();
}


void ChLinkMotorRotation::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotation>();

    // serialize parent class
    ChLinkMotor::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotation::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorRotation>();

    // deserialize parent class
    ChLinkMotor::ArchiveIN(marchive);

    // deserialize all member data:
}





// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationAngle)

ChLinkMotorRotationAngle::ChLinkMotorRotationAngle() {
    
    // default motion function : a ramp
    this->f_rot = std::make_shared<ChFunction_Ramp>(
        0.0,   // default y(0)
        1.0    // default dy/dx , i.e.   1 [rad/s]
        );
    
    rot_offset = 0;
}

ChLinkMotorRotationAngle::ChLinkMotorRotationAngle(const ChLinkMotorRotationAngle& other) : ChLinkMotorRotation(other) {
   this->f_rot = other.f_rot;
   this->rot_offset = other.rot_offset;
}

ChLinkMotorRotationAngle::~ChLinkMotorRotationAngle() {
    
}


void ChLinkMotorRotationAngle::Update(double mytime, bool update_assets) {
    
    // Inherit parent class:
    ChLinkMotorRotation::Update(mytime, update_assets);

    this->f_rot->Update(mytime); // call callbacks if any

    // Override the rotational jacobian [Cq] and the rotational residual C, 
    // by assuming an additional hidden frame that rotates about frame2:

    if (this->Body1 && this->Body2) {

        ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
        ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
        
        ChFrame<> aframe12;
        aframe2.TransformParentToLocal(aframe1, aframe12); 

        ChFrame<> aframe2rotating;

        double aux_rotation;

        aux_rotation = this->f_rot->Get_y(mytime) + this->rot_offset;
      
        aframe2rotating.SetRot( aframe2.GetRot() * Q_from_AngAxis(aux_rotation, VECT_Z) );

        ChFrame<> aframe12rotating;
        aframe2rotating.TransformParentToLocal(aframe1, aframe12rotating); 

        ChMatrix33<> Jw1, Jw2;
        ChMatrix33<> mtempM, mtempQ;

        ChMatrix33<> abs_plane_rotating = aframe2rotating.GetA();

        Jw1.MatrTMultiply(abs_plane_rotating, Body1->GetA());
        Jw2.MatrTMultiply(abs_plane_rotating, Body2->GetA());

        Jw2.MatrNeg();

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        mtempM.Set_X_matrix((aframe12rotating.GetRot().GetVector()) * 0.5);
        mtempM(0, 0) = 0.5 * aframe12rotating.GetRot().e0();
        mtempM(1, 1) = 0.5 * aframe12rotating.GetRot().e0();
        mtempM(2, 2) = 0.5 * aframe12rotating.GetRot().e0();
        mtempQ.MatrTMultiply(mtempM, Jw1);
        Jw1 = mtempQ;
        mtempQ.MatrTMultiply(mtempM, Jw2);
        Jw2 = mtempQ;
      
        int nc = 0;

        if (c_x) {
            nc++;
        }
        if (c_y) {
            nc++;
        }
        if (c_z) {
            nc++;
        }
        if (c_rx) {
            this->C->ElementN(nc) = aframe12rotating.GetRot().e1();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 0, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 0, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_ry) {
            this->C->ElementN(nc) = aframe12rotating.GetRot().e2();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 1, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 1, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_rz) {
            this->C->ElementN(nc) = aframe12rotating.GetRot().e3();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 2, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 2, 0, 1, 3, 0, 3);
            nc++;
        }
    }
}

void ChLinkMotorRotationAngle::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {

    double mCt = - 0.5 * this->f_rot->Get_y_dx(this->GetChTime());
    int ncrz = mask->nconstr - 1;
    if (mask->Constr_N(ncrz).IsActive()) {
        Qc(off_L + ncrz) += c * mCt; 
    }
}


void ChLinkMotorRotationAngle::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = - 0.5 * this->f_rot->Get_y_dx(this->GetChTime());
    int ncrz = mask->nconstr - 1;
    if (mask->Constr_N(ncrz).IsActive()) {
            mask->Constr_N(ncrz).Set_b_i(mask->Constr_N(ncrz).Get_b_i() + factor * mCt); 
    }
}



void ChLinkMotorRotationAngle::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationAngle>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_rot);
    marchive << CHNVP(rot_offset);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationAngle::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorRotationAngle>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_rot);
    marchive >> CHNVP(rot_offset);
}





// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationSpeed)

ChLinkMotorRotationSpeed::ChLinkMotorRotationSpeed() {

    this->variable.GetMass()(0,0) = 1.0;
    this->variable.GetInvMass()(0,0) = 1.0;

    this->f_speed = std::make_shared<ChFunction_Const>(1.0);

    this->rot_offset = 0;

    this->aux_dt = 0; // used for integrating speed, = rot
    this->aux_dtdt = 0;

    this->avoid_angle_drift = true;
}

ChLinkMotorRotationSpeed::ChLinkMotorRotationSpeed(const ChLinkMotorRotationSpeed& other) : ChLinkMotorRotation(other) {
    
    this->variable = other.variable;

    this->f_speed = other.f_speed;

    this->rot_offset =other.rot_offset;

    this->aux_dt = other.aux_dt; 
    this->aux_dtdt = other.aux_dtdt;
    
    this->avoid_angle_drift = other.avoid_angle_drift;
}

ChLinkMotorRotationSpeed::~ChLinkMotorRotationSpeed() {
    
}


void ChLinkMotorRotationSpeed::Update(double mytime, bool update_assets) {
    
    // Inherit parent class:
    ChLinkMotorRotation::Update(mytime, update_assets);

    this->f_speed->Update(mytime); // call callbacks if any

    // Override the rotational jacobian [Cq] and the rotational residual C, 
    // by assuming an additional hidden frame that rotates about frame2:

    if (this->Body1 && this->Body2) {

        ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
        ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
        
        ChFrame<> aframe12;
        aframe2.TransformParentToLocal(aframe1, aframe12); 

        ChFrame<> aframe2rotating;

        double aux_rotation;

        if (this->avoid_angle_drift) {
            aux_rotation = this->aux_dt + this->rot_offset;
        }
        else {
            // to have it aligned to current rot, to allow C=0.
            aux_rotation = aframe12.GetRot().Q_to_Rotv().z();
        }
        
        aframe2rotating.SetRot( aframe2.GetRot() * Q_from_AngAxis(aux_rotation, VECT_Z) );

        ChFrame<> aframe12rotating;
        aframe2rotating.TransformParentToLocal(aframe1, aframe12rotating); 

        ChMatrix33<> Jw1, Jw2;
        ChMatrix33<> mtempM, mtempQ;

        ChMatrix33<> abs_plane_rotating = aframe2rotating.GetA();

        Jw1.MatrTMultiply(abs_plane_rotating, Body1->GetA());
        Jw2.MatrTMultiply(abs_plane_rotating, Body2->GetA());

        Jw2.MatrNeg();

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        mtempM.Set_X_matrix((aframe12rotating.GetRot().GetVector()) * 0.5);
        mtempM(0, 0) = 0.5 * aframe12rotating.GetRot().e0();
        mtempM(1, 1) = 0.5 * aframe12rotating.GetRot().e0();
        mtempM(2, 2) = 0.5 * aframe12rotating.GetRot().e0();
        mtempQ.MatrTMultiply(mtempM, Jw1);
        Jw1 = mtempQ;
        mtempQ.MatrTMultiply(mtempM, Jw2);
        Jw2 = mtempQ;
      
        int nc = 0;

        if (c_x) {
            nc++;
        }
        if (c_y) {
            nc++;
        }
        if (c_z) {
            nc++;
        }
        if (c_rx) {
            this->C->ElementN(nc) = aframe12rotating.GetRot().e1();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 0, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 0, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_ry) {
            this->C->ElementN(nc) = aframe12rotating.GetRot().e2();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 1, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 1, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_rz) {
            this->C->ElementN(nc) = aframe12rotating.GetRot().e3();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 2, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 2, 0, 1, 3, 0, 3);
            nc++;
        }

    }

}

void ChLinkMotorRotationSpeed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {

    double mCt = - 0.5 * this->f_speed->Get_y(this->GetChTime());
    
    int ncrz = mask->nconstr - 1;
    if (mask->Constr_N(ncrz).IsActive()) {
        Qc(off_L + ncrz) += c * mCt;
    }
}


void ChLinkMotorRotationSpeed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = - 0.5 * this->f_speed->Get_y(this->GetChTime());
    int ncrz = mask->nconstr - 1;
    if (mask->Constr_N(ncrz).IsActive()) {
            mask->Constr_N(ncrz).Set_b_i(mask->Constr_N(ncrz).Get_b_i() + factor * mCt);
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMotorRotationSpeed::IntStateGather(const unsigned int off_x,  // offset in x state vector
                             ChState& x,                // state vector, position part
                             const unsigned int off_v,  // offset in v state vector
                             ChStateDelta& v,           // state vector, speed part
                             double& T                  // time
                             ) {
    x(off_x) = 0;//aux;
    v(off_v) = aux_dt;
    T = GetChTime();
}

void ChLinkMotorRotationSpeed::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                              const ChState& x,          // state vector, position part
                              const unsigned int off_v,  // offset in v state vector
                              const ChStateDelta& v,     // state vector, speed part
                              const double T             // time
                              ) {
    //aux = x(off_x);
    aux_dt = v(off_v);
}

void ChLinkMotorRotationSpeed::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = aux_dtdt;
}

void ChLinkMotorRotationSpeed::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    aux_dtdt = a(off_a);
}

void ChLinkMotorRotationSpeed::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                const double c           // a scaling factor
                                ) {
    double imposed_speed = this->f_speed->Get_y(this->GetChTime());
    R(off) +=  imposed_speed * c;
}

void ChLinkMotorRotationSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                 ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  // the w vector
                                 const double c               // a scaling factor
                                 ) {
    R(off) += c * 1.0 * w(off);
}

void ChLinkMotorRotationSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                              const ChStateDelta& v,
                              const ChVectorDynamic<>& R,
                              const unsigned int off_L,  // offset in L, Qc
                              const ChVectorDynamic<>& L,
                              const ChVectorDynamic<>& Qc) {
    // inherit parent
    ChLinkMotorRotation::IntToDescriptor( off_v, v, R, off_L,  L, Qc);

    this->variable.Get_qb()(0, 0) = v(off_v);
    this->variable.Get_fb()(0, 0) = R(off_v);
}

void ChLinkMotorRotationSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                ChStateDelta& v,
                                const unsigned int off_L,  // offset in L
                                ChVectorDynamic<>& L) {
    //inherit parent
    ChLinkMotorRotation::IntFromDescriptor(off_v, v, off_L, L);

    v(off_v) = this->variable.Get_qb()(0, 0);
}

////
void ChLinkMotorRotationSpeed::InjectVariables(ChSystemDescriptor& mdescriptor) {
    variable.SetDisabled(!IsActive());

    mdescriptor.InsertVariables(&variable);
}

void ChLinkMotorRotationSpeed::VariablesFbReset() {
    variable.Get_fb().FillElem(0.0);
}

void ChLinkMotorRotationSpeed::VariablesFbLoadForces(double factor) {
    
    double imposed_speed = this->f_speed->Get_y(this->GetChTime());
    variable.Get_fb().ElementN(0) += imposed_speed * factor;
}

void ChLinkMotorRotationSpeed::VariablesFbIncrementMq() {
    variable.Compute_inc_Mb_v(variable.Get_fb(), variable.Get_qb());
}

void ChLinkMotorRotationSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.Get_qb().SetElement(0, 0, aux_dt);
}

void ChLinkMotorRotationSpeed::VariablesQbSetSpeed(double step) {
    double old_dt = aux_dt;

    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.Get_qb().GetElement(0, 0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}



void ChLinkMotorRotationSpeed::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationSpeed>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_speed);
    marchive << CHNVP(rot_offset);
    marchive << CHNVP(avoid_angle_drift);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationSpeed::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorRotationSpeed>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_speed);
    marchive >> CHNVP(rot_offset);
    marchive >> CHNVP(avoid_angle_drift);
}





// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotorRotationTorque)

ChLinkMotorRotationTorque::ChLinkMotorRotationTorque() {
    
    this->c_rz = false;
    SetupLinkMask();

    this->f_torque = std::make_shared<ChFunction_Const>(0.0);
}

ChLinkMotorRotationTorque::ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other) : ChLinkMotorRotation(other) {
   this->f_torque = other.f_torque;
}

ChLinkMotorRotationTorque::~ChLinkMotorRotationTorque() {
    
}

void ChLinkMotorRotationTorque::Update(double mytime, bool update_assets) {

     // Inherit parent class:
    ChLinkMotorRotation::Update(mytime, update_assets);

    this->f_torque->Update(mytime); // call callbacks if any
}

void ChLinkMotorRotationTorque::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // compute instant torque
    double mT = this->f_torque->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_torque = aframe2.GetA().Matr_x_Vect(ChVector<>(  0,0,mT ) );

    if (Body2->Variables().IsActive()) {
        R.PasteSumVector(Body2->TransformDirectionParentToLocal(m_abs_torque) * -c,
                            Body2->Variables().GetOffset() + 3, 0);
    }

    if (Body1->Variables().IsActive()) {
        R.PasteSumVector(Body1->TransformDirectionParentToLocal(m_abs_torque) * c,
                            Body1->Variables().GetOffset() + 3, 0);
    }
}


void ChLinkMotorRotationTorque::ConstraintsFbLoadForces(double factor) {
    // compute instant torque
    double mT = this->f_torque->Get_y( this->GetChTime());

    ChFrame<> aframe1 = this->frame1 >> (*this->Body1);
    ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
    Vector m_abs_torque = aframe2.GetA().Matr_x_Vect(ChVector<>(  0,0,mT ) );

    Body2->Variables().Get_fb().PasteSumVector(Body2->TransformDirectionParentToLocal(m_abs_torque) * -factor, 3, 0);

    Body1->Variables().Get_fb().PasteSumVector(Body1->TransformDirectionParentToLocal(m_abs_torque) * factor, 3, 0);
}

void ChLinkMotorRotationTorque::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotorRotationTorque>();

    // serialize parent class
    ChLinkMotorRotation::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(f_torque);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotorRotationTorque::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMotorRotationTorque>();

    // deserialize parent class
    ChLinkMotorRotation::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(f_torque);
}





// -----------------------------------------------------------------------------

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

ChLinkMotorRotationDriveline::ChLinkMotorRotationDriveline(const ChLinkMotorRotationDriveline& other) : ChLinkMotorRotation(other) {
    innershaft1 = other.innershaft1;
    innershaft2 = other.innershaft2;
    innerconstraint1 = other.innerconstraint1;
    innerconstraint2 = other.innerconstraint2; 
}

ChLinkMotorRotationDriveline::~ChLinkMotorRotationDriveline() {
    
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
