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

#include "chrono/physics/ChShaftsMotorSpeed.h"

namespace chrono {



// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsMotorSpeed)

ChShaftsMotorSpeed::ChShaftsMotorSpeed() : motor_torque(0) {

    this->variable.GetMass()(0,0) = 1.0;
    this->variable.GetInvMass()(0,0) = 1.0;

    this->f_speed = chrono_types::make_shared<ChFunction_Const>(1.0);

    this->rot_offset = 0;

    this->aux_dt = 0; // used for integrating speed, = pos
    this->aux_dtdt = 0;

    this->avoid_angle_drift = true;
}

ChShaftsMotorSpeed::ChShaftsMotorSpeed(const ChShaftsMotorSpeed& other) : ChShaftsMotorBase(other) {

   this->variable = other.variable;

    this->f_speed = other.f_speed;

    this->rot_offset =other.rot_offset;

    this->aux_dt = other.aux_dt; 
    this->aux_dtdt = other.aux_dtdt;
    
    this->avoid_angle_drift = other.avoid_angle_drift;

}

bool ChShaftsMotorSpeed::Initialize(std::shared_ptr<ChShaft> mshaft1, std::shared_ptr<ChShaft> mshaft2) {
    // Parent class initialize
    if (!ChShaftsMotorBase::Initialize(mshaft1, mshaft2))
        return false;

    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsMotorSpeed::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsMotorBase::Update(mytime, update_assets);

    // update class data

    this->f_speed->Update(mytime); // call callbacks if any

}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsMotorSpeed::IntStateGather(const unsigned int off_x,  // offset in x state vector
                             ChState& x,                // state vector, position part
                             const unsigned int off_v,  // offset in v state vector
                             ChStateDelta& v,           // state vector, speed part
                             double& T                  // time
                             ) {
    x(off_x) = 0;//aux;
    v(off_v) = aux_dt;
    T = GetChTime();
}

void ChShaftsMotorSpeed::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                         const ChState& x,          // state vector, position part
                                         const unsigned int off_v,  // offset in v state vector
                                         const ChStateDelta& v,     // state vector, speed part
                                         const double T,            // time
                                         bool full_update           // perform complete update
) {
    // aux = x(off_x);
    aux_dt = v(off_v);

    Update(T, full_update);
}

void ChShaftsMotorSpeed::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = aux_dtdt;
}

void ChShaftsMotorSpeed::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    aux_dtdt = a(off_a);
}

void ChShaftsMotorSpeed::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
        L(off_L) =  motor_torque;
}

void ChShaftsMotorSpeed::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
        motor_torque =  L(off_L);
}

void ChShaftsMotorSpeed::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                const double c           // a scaling factor
                                ) {
    double imposed_speed = this->f_speed->Get_y(this->GetChTime());
    R(off) +=  imposed_speed * c;
}

void ChShaftsMotorSpeed::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                 ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  // the w vector
                                 const double c               // a scaling factor
                                 ) {
    R(off) += c * 1.0 * w(off);
}

void ChShaftsMotorSpeed::IntLoadLumpedMass_Md(const unsigned int off,
                                              ChVectorDynamic<>& Md,
                                              double& err,
                                              const double c 
) {
    Md(off) += c * 1.0;
}

void ChShaftsMotorSpeed::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                        ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                        const ChVectorDynamic<>& L,  // the L vector
                                        const double c               // a scaling factor
                                        ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsMotorSpeed::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                        ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                        const double c,            // a scaling factor
                                        bool do_clamp,             // apply clamping to c*C?
                                        double recovery_clamp      // value for min/max clamping of c*C
                                        ) {
    // Add the time-dependent term in residual C as 
    //   C = d_error - d_setpoint - d_offset
    // with d_error = x_pos_A-x_pos_B, and d_setpoint = x(t)
    double C;
    if (this->avoid_angle_drift) 
        C = this->GetMotorRot()  - aux_dt - this->rot_offset; 
    else
        C = 0.0;

    double res = c * C;

    if (do_clamp) {
        res = ChMin(ChMax(res, -recovery_clamp), recovery_clamp);
    }
    Qc(off_L) += res;
}

void ChShaftsMotorSpeed::IntLoadConstraint_Ct(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*Ct
                                         const double c             // a scaling factor
                                         ) {
    double ct = - this->f_speed->Get_y(this->GetChTime());
    Qc(off_L) += c * ct;
}

void ChShaftsMotorSpeed::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                    const ChStateDelta& v,
                                    const ChVectorDynamic<>& R,
                                    const unsigned int off_L,  // offset in L, Qc
                                    const ChVectorDynamic<>& L,
                                    const ChVectorDynamic<>& Qc) {
     constraint.Set_l_i(L(off_L));
     constraint.Set_b_i(Qc(off_L));

     this->variable.Get_qb()(0, 0) = v(off_v);
     this->variable.Get_fb()(0, 0) = R(off_v);
}

void ChShaftsMotorSpeed::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                      ChStateDelta& v,
                                      const unsigned int off_L,  // offset in L
                                      ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();

    v(off_v) = this->variable.Get_qb()(0, 0);
}

// SOLVER INTERFACES

void ChShaftsMotorSpeed::InjectConstraints(ChSystemDescriptor& mdescriptor) {

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsMotorSpeed::InjectVariables(ChSystemDescriptor& mdescriptor) {

    mdescriptor.InsertVariables(&variable);
}

void ChShaftsMotorSpeed::VariablesFbReset() {
    variable.Get_fb().setZero();
}

void ChShaftsMotorSpeed::VariablesFbLoadForces(double factor) {
    
    double imposed_speed = this->f_speed->Get_y(this->GetChTime());
    variable.Get_fb()(0) += imposed_speed * factor;
}

void ChShaftsMotorSpeed::VariablesFbIncrementMq() {
    variable.Compute_inc_Mb_v(variable.Get_fb(), variable.Get_qb());
}

void ChShaftsMotorSpeed::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variable.Get_qb()(0) = aux_dt;
}

void ChShaftsMotorSpeed::VariablesQbSetSpeed(double step) {
    // from 'qb' vector, sets body speed, and updates auxiliary data
    aux_dt = variable.Get_qb()(0);

    // Compute accel. by BDF (approximate by differentiation); not needed
}


void ChShaftsMotorSpeed::ConstraintsBiReset() {

     constraint.Set_b_i(0.);
}

void ChShaftsMotorSpeed::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {

    double C;
    if (this->avoid_angle_drift) 
        C = this->GetMotorRot()  - aux_dt - this->rot_offset; 
    else
        C = 0.0;

    double res = factor * C;

    if (do_clamp) {
        res = ChMin(ChMax(res, -recovery_clamp), recovery_clamp);
    }

    constraint.Set_b_i(constraint.Get_b_i() + res);
}

void ChShaftsMotorSpeed::ConstraintsBiLoad_Ct(double factor) {

    double ct = - this->f_speed->Get_y(this->GetChTime());
    constraint.Set_b_i(constraint.Get_b_i() + factor * ct);
}

void ChShaftsMotorSpeed::ConstraintsLoadJacobians() {
    constraint.Get_Cq_a()(0) = 1;
    constraint.Get_Cq_b()(0) = -1;
}

void ChShaftsMotorSpeed::ConstraintsFetch_react(double factor) {
    motor_torque = - constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsMotorSpeed::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsMotorSpeed>();

    // serialize parent class
    ChShaftsMotorBase::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(motor_torque);
    marchive << CHNVP(f_speed);
    marchive << CHNVP(rot_offset);
    marchive << CHNVP(avoid_angle_drift);

}

/// Method to allow de serialization of transient data from archives.
void ChShaftsMotorSpeed::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChShaftsMotorSpeed>();

    // deserialize parent class:
    ChShaftsMotorBase::ArchiveIn(marchive);

    // deserialize all member data:
    marchive >> CHNVP(motor_torque);
    marchive >> CHNVP(f_speed);
    marchive >> CHNVP(rot_offset);
    marchive >> CHNVP(avoid_angle_drift);
    constraint.SetVariables(&shaft1->Variables(), &shaft2->Variables());
}




}  // end namespace chrono
