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
    this->mrot = aframe12.GetRot().Q_to_Rotv().z(); //***TODO*** multi-turn
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
        Qc(off_L + ncrz) += c * mCt; //***TODO** x2 term?
    }
}


void ChLinkMotorRotationAngle::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double mCt = - 0.5 * this->f_rot->Get_y_dx(this->GetChTime());
    int ncrz = mask->nconstr - 1;
    if (mask->Constr_N(ncrz).IsActive()) {
            mask->Constr_N(ncrz).Set_b_i(mask->Constr_N(ncrz).Get_b_i() + factor * mCt); //***TODO** x2 term?
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














}  // end namespace chrono
