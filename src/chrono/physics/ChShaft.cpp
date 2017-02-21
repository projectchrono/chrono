// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaft)

ChShaft::ChShaft()
    : torque(0),
      pos(0),
      pos_dt(0),
      pos_dtdt(0),
      inertia(1),
      fixed(false),
      limitspeed(false),
      max_speed(10.0f),
      sleep_time(0.6f),
      sleep_starttime(0),
      sleep_minspeed(0.1f),
      sleep_minwvel(0.04f),
      sleeping(false) {
    SetUseSleeping(true);
    variables.SetShaft(this);
}

ChShaft::ChShaft(const ChShaft& other) : ChPhysicsItem(other) {
    torque = other.torque;
    system = other.system;
    pos = other.pos;
    pos_dt = other.pos_dt;
    pos_dtdt = other.pos_dtdt;
    inertia = other.inertia;
    fixed = other.fixed;
    sleeping = other.sleeping;
    limitspeed = other.limitspeed;

    variables = other.variables;

    max_speed = other.max_speed;

    sleep_time = other.sleep_time;
    sleep_starttime = other.sleep_starttime;
    sleep_minspeed = other.sleep_minspeed;
    sleep_minwvel = other.sleep_minwvel;
}

void ChShaft::SetInertia(double newJ) {
    assert(newJ > 0.);
    if (newJ <= 0.)
        return;
    inertia = newJ;
    variables.SetInertia(newJ);
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaft::IntStateGather(const unsigned int off_x,  // offset in x state vector
                             ChState& x,                // state vector, position part
                             const unsigned int off_v,  // offset in v state vector
                             ChStateDelta& v,           // state vector, speed part
                             double& T                  // time
                             ) {
    x(off_x) = pos;
    v(off_v) = pos_dt;
    T = GetChTime();
}

void ChShaft::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                              const ChState& x,          // state vector, position part
                              const unsigned int off_v,  // offset in v state vector
                              const ChStateDelta& v,     // state vector, speed part
                              const double T             // time
                              ) {
    SetPos(x(off_x));
    SetPos_dt(v(off_v));
    Update(T);
}

void ChShaft::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = pos_dtdt;
}

void ChShaft::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPos_dtdt(a(off_a));
}

void ChShaft::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                const double c           // a scaling factor
                                ) {
    // add applied forces to 'fb' vector
    R(off) += torque * c;
}

void ChShaft::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                 ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  // the w vector
                                 const double c               // a scaling factor
                                 ) {
    R(off) += c * inertia * w(off);
}

void ChShaft::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                              const ChStateDelta& v,
                              const ChVectorDynamic<>& R,
                              const unsigned int off_L,  // offset in L, Qc
                              const ChVectorDynamic<>& L,
                              const ChVectorDynamic<>& Qc) {
    variables.Get_qb()(0, 0) = v(off_v);
    variables.Get_fb()(0, 0) = R(off_v);
}

void ChShaft::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                ChStateDelta& v,
                                const unsigned int off_L,  // offset in L
                                ChVectorDynamic<>& L) {
    v(off_v) = variables.Get_qb()(0, 0);
}

////
void ChShaft::InjectVariables(ChSystemDescriptor& mdescriptor) {
    variables.SetDisabled(!IsActive());

    mdescriptor.InsertVariables(&variables);
}

void ChShaft::VariablesFbReset() {
    variables.Get_fb().FillElem(0.0);
}

void ChShaft::VariablesFbLoadForces(double factor) {
    // add applied torques to 'fb' vector
    variables.Get_fb().ElementN(0) += torque * factor;
}

void ChShaft::VariablesFbIncrementMq() {
    variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
}

void ChShaft::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variables.Get_qb().SetElement(0, 0, pos_dt);
}

void ChShaft::VariablesQbSetSpeed(double step) {
    double old_dt = pos_dt;

    // from 'qb' vector, sets body speed, and updates auxiliary data
    pos_dt = variables.Get_qb().GetElement(0, 0);

    // apply limits (if in speed clamping mode) to speeds.
    ClampSpeed();

    // Compute accel. by BDF (approximate by differentiation);
    if (step) {
        pos_dtdt = (pos_dt - old_dt) / step;
    }
}

void ChShaft::VariablesQbIncrementPosition(double dt_step) {
    if (!IsActive())
        return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

    double newspeed = variables.Get_qb().GetElement(0, 0);

    // ADVANCE POSITION: pos' = pos + dt * vel
    pos = pos + newspeed * dt_step;
}

void ChShaft::SetNoSpeedNoAcceleration() {
    pos_dt = 0;
    pos_dtdt = 0;
}

////
void ChShaft::ClampSpeed() {
    if (GetLimitSpeed()) {
        if (pos_dt > max_speed)
            pos_dt = max_speed;
        if (pos_dt < -max_speed)
            pos_dt = -max_speed;
    }
}

bool ChShaft::TrySleeping() {
    if (GetUseSleeping()) {
        if (GetSleeping())
            return true;

        if (fabs(pos_dt) < sleep_minspeed) {
            if ((GetChTime() - sleep_starttime) > sleep_time) {
                SetSleeping(true);
                return true;
            }
        } else {
            sleep_starttime = float(GetChTime());
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

void ChShaft::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaft>();

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
void ChShaft::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaft>();

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

}  // end namespace chrono
