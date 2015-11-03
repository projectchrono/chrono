//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChShaft.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChShaft.h"
#include "physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChShaft> a_registration_ChShaft;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SHAFTS

ChShaft::ChShaft() {
    this->torque = 0;
    this->system = 0;
    this->pos = 0;
    this->pos_dt = 0;
    this->pos_dtdt = 0;
    this->inertia = 1;
    this->fixed = false;
    this->limitspeed = false;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID

    max_speed = 10.f;

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;
    this->sleeping = false;
    SetUseSleeping(true);

    variables.SetShaft(this);
}

ChShaft::~ChShaft() {
}

void ChShaft::Copy(ChShaft* source) {
    // copy the parent class data...
    ChPhysicsItem::Copy(source);

    this->torque = source->torque;
    this->system = source->system;
    this->pos = source->pos;
    this->pos_dt = source->pos_dt;
    this->pos_dtdt = source->pos_dtdt;
    this->inertia = source->inertia;
    this->fixed = source->fixed;
    this->sleeping = source->sleeping;
    this->limitspeed = source->limitspeed;

    this->system = source->system;

    variables = source->variables;

    max_speed = source->max_speed;

    sleep_time = source->sleep_time;
    sleep_starttime = source->sleep_starttime;
    sleep_minspeed = source->sleep_minspeed;
    sleep_minwvel = source->sleep_minwvel;
}

void ChShaft::SetInertia(double newJ) {
    assert(newJ > 0.);
    if (newJ <= 0.)
        return;
    this->inertia = newJ;
    this->variables.SetInertia(newJ);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaft::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                             ChState& x,                ///< state vector, position part
                             const unsigned int off_v,  ///< offset in v state vector
                             ChStateDelta& v,           ///< state vector, speed part
                             double& T)                 ///< time
{
    x(off_x) = this->pos;
    v(off_v) = this->pos_dt;
    T = this->GetChTime();
}

void ChShaft::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                              const ChState& x,          ///< state vector, position part
                              const unsigned int off_v,  ///< offset in v state vector
                              const ChStateDelta& v,     ///< state vector, speed part
                              const double T)            ///< time
{
    this->SetPos(x(off_x));
    this->SetPos_dt(v(off_v));
    this->Update(T);
}

void ChShaft::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = this->pos_dtdt;
}

void ChShaft::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    this->SetPos_dtdt(a(off_a));
}

void ChShaft::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                const double c           ///< a scaling factor
                                ) {
    // add applied forces to 'fb' vector
    R(off) += this->torque * c;
}

void ChShaft::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                 ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  ///< the w vector
                                 const double c               ///< a scaling factor
                                 ) {
    R(off) += c * this->inertia * w(off);
}

void ChShaft::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                       const ChStateDelta& v,
                       const ChVectorDynamic<>& R,
                       const unsigned int off_L,  ///< offset in L, Qc
                       const ChVectorDynamic<>& L,
                       const ChVectorDynamic<>& Qc) {
    this->variables.Get_qb()(0, 0) = v(off_v);
    this->variables.Get_fb()(0, 0) = R(off_v);
}

void ChShaft::IntFromLCP(const unsigned int off_v,  ///< offset in v
                         ChStateDelta& v,
                         const unsigned int off_L,  ///< offset in L
                         ChVectorDynamic<>& L) {
    v(off_v) = this->variables.Get_qb()(0, 0);
}

////
void ChShaft::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
    this->variables.SetDisabled(!this->IsActive());

    mdescriptor.InsertVariables(&this->variables);
}

void ChShaft::VariablesFbReset() {
    this->variables.Get_fb().FillElem(0.0);
}

void ChShaft::VariablesFbLoadForces(double factor) {
    // add applied torques to 'fb' vector
    this->variables.Get_fb().ElementN(0) += this->torque * factor;
}

void ChShaft::VariablesFbIncrementMq() {
    this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
}

void ChShaft::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
    this->variables.Get_qb().SetElement(0, 0, pos_dt);
}

void ChShaft::VariablesQbSetSpeed(double step) {
    double old_dt = this->pos_dt;

    // from 'qb' vector, sets body speed, and updates auxiliary data
    this->pos_dt = this->variables.Get_qb().GetElement(0, 0);

    // apply limits (if in speed clamping mode) to speeds.
    ClampSpeed();

    // Compute accel. by BDF (approximate by differentiation);
    if (step) {
        pos_dtdt = ((this->pos_dt - old_dt) / step);
    }
}

void ChShaft::VariablesQbIncrementPosition(double dt_step) {
    if (!this->IsActive())
        return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

    double newspeed = variables.Get_qb().GetElement(0, 0);

    // ADVANCE POSITION: pos' = pos + dt * vel
    this->pos = this->pos + newspeed * dt_step;
}

void ChShaft::SetNoSpeedNoAcceleration() {
    this->pos_dt = 0;
    this->pos_dtdt = 0;
}

////
void ChShaft::ClampSpeed() {
    if (this->GetLimitSpeed()) {
        if (this->pos_dt > max_speed)
            this->pos_dt = max_speed;
        if (this->pos_dt < -max_speed)
            this->pos_dt = -max_speed;
    }
}

bool ChShaft::TrySleeping() {
    if (this->GetUseSleeping()) {
        if (this->GetSleeping())
            return true;

        if (fabs(this->pos_dt) < this->sleep_minspeed) {
            if ((this->GetChTime() - this->sleep_starttime) > this->sleep_time) {
                SetSleeping(true);
                return true;
            }
        } else {
            this->sleep_starttime = float(this->GetChTime());
        }
    }
    return false;
}

void ChShaft::Update(double mytime, bool update_assets) {
    // Update parent class too
    ChPhysicsItem::Update(mytime, update_assets);

    // Class update

    // TrySleeping();    // See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();  // Apply limits (if in speed clamping mode) to speeds.
}

//////// FILE I/O

void ChShaft::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(torque);
    marchive << CHNVP(pos);
    marchive << CHNVP(pos_dt);
    marchive << CHNVP(pos_dtdt);
    marchive << CHNVP(inertia);
    marchive << CHNVP(fixed);
    marchive << CHNVP(limitspeed);
    marchive << CHNVP(max_speed);
    marchive << CHNVP(sleep_time);
    marchive << CHNVP(sleep_starttime);
    marchive << CHNVP(sleep_minspeed);
    marchive << CHNVP(sleep_minwvel);
    marchive << CHNVP(sleeping);
    marchive << CHNVP(use_sleeping);
}

/// Method to allow de serialization of transient data from archives.
void ChShaft::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(torque);
    marchive >> CHNVP(pos);
    marchive >> CHNVP(pos_dt);
    marchive >> CHNVP(pos_dtdt);
    marchive >> CHNVP(inertia);
    marchive >> CHNVP(fixed);
    marchive >> CHNVP(limitspeed);
    marchive >> CHNVP(max_speed);
    marchive >> CHNVP(sleep_time);
    marchive >> CHNVP(sleep_starttime);
    marchive >> CHNVP(sleep_minspeed);
    marchive >> CHNVP(sleep_minwvel);
    marchive >> CHNVP(sleeping);
    marchive >> CHNVP(use_sleeping);
}

}  // END_OF_NAMESPACE____

/////////////////////
