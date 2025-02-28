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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaft)
CH_UPCASTING(ChShaft, ChPhysicsItem)
CH_UPCASTING(ChShaft, ChLoadable)

ChShaft::ChShaft()
    : load(0),
      pos(0),
      pos_dt(0),
      pos_dtdt(0),
      inertia(1),
      max_speed(10.0f),
      sleep_time(0.6f),
      sleep_minspeed(0.1f),
      sleep_starttime(0),
      fixed(false),
      limitspeed(false),
      sleeping(false) {
    SetSleepingAllowed(true);
    variables.SetShaft(this);
}

ChShaft::ChShaft(const ChShaft& other) : ChPhysicsItem(other) {
    load = other.load;
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
}

void ChShaft::SetInertia(double newJ) {
    assert(newJ > 0.);
    if (newJ <= 0.)
        return;
    inertia = newJ;
    variables.SetInertia(newJ);
}

void ChShaft::SetFixed(bool state) {
    fixed = state;
    variables.SetDisabled(fixed);
}

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
                              const double T,            // time
                              bool full_update           // perform complete update
) {
    SetPos(x(off_x));
    SetPosDt(v(off_v));
    Update(T, full_update);
}

void ChShaft::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a(off_a) = pos_dtdt;
}

void ChShaft::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPosDt2(a(off_a));
}

void ChShaft::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                const double c           // a scaling factor
) {
    // add applied forces to 'fb' vector
    R(off) += load * c;
}

void ChShaft::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                 ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                 const ChVectorDynamic<>& w,  // the w vector
                                 const double c               // a scaling factor
) {
    R(off) += c * inertia * w(off);
}

void ChShaft::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    Md(off) += c * inertia;
}

void ChShaft::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                              const ChStateDelta& v,
                              const ChVectorDynamic<>& R,
                              const unsigned int off_L,  // offset in L, Qc
                              const ChVectorDynamic<>& L,
                              const ChVectorDynamic<>& Qc) {
    variables.State()(0, 0) = v(off_v);
    variables.Force()(0, 0) = R(off_v);
}

void ChShaft::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                ChStateDelta& v,
                                const unsigned int off_L,  // offset in L
                                ChVectorDynamic<>& L) {
    v(off_v) = variables.State()(0, 0);
}

void ChShaft::InjectVariables(ChSystemDescriptor& descriptor) {
    variables.SetDisabled(!IsActive());

    descriptor.InsertVariables(&variables);
}

void ChShaft::VariablesFbReset() {
    variables.Force().setZero();
}

void ChShaft::VariablesFbLoadForces(double factor) {
    // add applied loads to 'fb' vector
    variables.Force()(0) += load * factor;
}

void ChShaft::VariablesFbIncrementMq() {
    variables.AddMassTimesVector(variables.Force(), variables.State());
}

void ChShaft::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variables.State()(0) = pos_dt;
}

void ChShaft::VariablesQbSetSpeed(double step) {
    double old_dt = pos_dt;

    // from 'qb' vector, sets body speed, and updates auxiliary data
    pos_dt = variables.State()(0);

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
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Euler step.

    double newspeed = variables.State()(0);

    // ADVANCE POSITION: pos' = pos + dt * vel
    pos = pos + newspeed * dt_step;
}

void ChShaft::ForceToRest() {
    pos_dt = 0;
    pos_dtdt = 0;
}

void ChShaft::ClampSpeed() {
    if (limitspeed) {
        if (pos_dt > max_speed)
            pos_dt = max_speed;
        if (pos_dt < -max_speed)
            pos_dt = -max_speed;
    }
}

bool ChShaft::TrySleeping() {
    if (IsSleepingAllowed()) {
        if (IsSleeping())
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

    // TrySleeping();    // See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();  // Apply limits (if in speed clamping mode) to speeds.
}

void ChShaft::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaft>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(load);
    archive_out << CHNVP(pos);
    archive_out << CHNVP(pos_dt);
    archive_out << CHNVP(pos_dtdt);
    archive_out << CHNVP(inertia);
    archive_out << CHNVP(fixed);
    archive_out << CHNVP(limitspeed);
    archive_out << CHNVP(max_speed);
    archive_out << CHNVP(sleep_time);
    archive_out << CHNVP(sleep_starttime);
    archive_out << CHNVP(sleep_minspeed);
    archive_out << CHNVP(use_sleeping);
}

void ChShaft::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaft>();

    // deserialize parent class:
    ChPhysicsItem::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(load);
    archive_in >> CHNVP(pos);
    archive_in >> CHNVP(pos_dt);
    archive_in >> CHNVP(pos_dtdt);
    archive_in >> CHNVP(inertia);
    archive_in >> CHNVP(fixed);
    archive_in >> CHNVP(limitspeed);
    archive_in >> CHNVP(max_speed);
    archive_in >> CHNVP(sleep_time);
    archive_in >> CHNVP(sleep_starttime);
    archive_in >> CHNVP(sleep_minspeed);
    archive_in >> CHNVP(use_sleeping);
}

}  // end namespace chrono
