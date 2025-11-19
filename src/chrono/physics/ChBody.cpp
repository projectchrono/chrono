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

#include <cstdlib>
#include <algorithm>

#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChBody)
CH_UPCASTING(ChBody, ChPhysicsItem)
CH_UPCASTING(ChBody, ChBodyFrame)
CH_UPCASTING(ChBody, ChContactable)
CH_UPCASTING(ChBody, ChLoadableUVW)

ChBody::ChBody()
    : index(0),
      fixed(false),
      collide(false),
      limit_speed(false),
      disable_gyrotorque(false),
      is_sleeping(false),
      allow_sleeping(true),
      candidate_sleeping(false),
      Xforce(VNULL),
      Xtorque(VNULL) {
    marklist.clear();
    forcelist.clear();

    max_speed = 0.5f;
    max_wvel = 2.0f * float(CH_PI);

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;

    variables.SetUserData((void*)this);

    // Load contactable variables list
    m_contactable_variables.push_back(&variables);
}

ChBody::ChBody(const ChBody& other) : ChPhysicsItem(other), ChBodyFrame(other) {
    fixed = other.fixed;
    collide = other.collide;
    limit_speed = other.limit_speed;
    disable_gyrotorque = other.disable_gyrotorque;
    is_sleeping = other.is_sleeping;
    allow_sleeping = other.allow_sleeping;
    candidate_sleeping = other.candidate_sleeping;

    variables = other.variables;
    variables.SetUserData((void*)this);

    m_contactable_variables.clear();
    m_contactable_variables.push_back(&variables);

    gyro = other.gyro;

    RemoveAllForces();   // also copy-duplicate the forces? Let the user handle this..
    RemoveAllMarkers();  // also copy-duplicate the markers? Let the user handle this..

    // Copy the collision model if any
    if (other.collision_model) {
        collision_model = chrono_types::make_shared<ChCollisionModel>(*other.collision_model);
        collision_model->SetContactable(this);
    }

    max_speed = other.max_speed;
    max_wvel = other.max_wvel;

    sleep_time = other.sleep_time;
    sleep_starttime = other.sleep_starttime;
    sleep_minspeed = other.sleep_minspeed;
    sleep_minwvel = other.sleep_minwvel;
}

ChBody::~ChBody() {
    RemoveAllForces();
    RemoveAllMarkers();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChBody::IntStateGather(const unsigned int off_x,  // offset in x state vector
                            ChState& x,                // state vector, position part
                            const unsigned int off_v,  // offset in v state vector
                            ChStateDelta& v,           // state vector, speed part
                            double& T                  // time
) {
    x.segment(off_x + 0, 3) = GetPos().eigen();
    x.segment(off_x + 3, 4) = GetRot().eigen();
    v.segment(off_v + 0, 3) = GetPosDt().eigen();
    v.segment(off_v + 3, 3) = GetAngVelLocal().eigen();
    T = GetChTime();
}

void ChBody::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                             const ChState& x,          // state vector, position part
                             const unsigned int off_v,  // offset in v state vector
                             const ChStateDelta& v,     // state vector, speed part
                             const double T,            // time
                             bool full_update           // perform complete update
) {
    SetCoordsys(x.segment(off_x, 7));
    SetPosDt(v.segment(off_v + 0, 3));
    SetAngVelLocal(v.segment(off_v + 3, 3));
    SetChTime(T);
    Update(T, full_update);
}

void ChBody::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a + 0, 3) = GetPosDt2().eigen();
    a.segment(off_a + 3, 3) = GetAngAccLocal().eigen();
}

void ChBody::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPosDt2(a.segment(off_a + 0, 3));
    SetAngAccLocal(a.segment(off_a + 3, 3));
}

void ChBody::IntStateIncrement(const unsigned int off_x,  // offset in x state vector
                               ChState& x_new,            // state vector, position part, incremented result
                               const ChState& x,          // state vector, initial position part
                               const unsigned int off_v,  // offset in v state vector
                               const ChStateDelta& Dv     // state vector, increment
) {
    // ADVANCE POSITION:
    x_new(off_x) = x(off_x) + Dv(off_v);
    x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
    x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);

    // ADVANCE ROTATION: R_new = DR_a * R_old
    // (using quaternions, local or abs:  q_new = Dq_a * q_old =  q_old * Dq_l  )
    ChQuaternion<> q_old(x.segment(off_x + 3, 4));
    ChQuaternion<> rel_q;
    rel_q.SetFromRotVec(Dv.segment(off_v + 3, 3));
    ChQuaternion<> q_new = q_old * rel_q;
    x_new.segment(off_x + 3, 4) = q_new.eigen();
}

void ChBody::IntStateGetIncrement(const unsigned int off_x,  // offset in x state vector
                                  const ChState& x_new,      // state vector, position part, incremented result
                                  const ChState& x,          // state vector, initial position part
                                  const unsigned int off_v,  // offset in v state vector
                                  ChStateDelta& Dv           // state vector, increment
) {
    // POSITION:
    Dv(off_v) = x_new(off_x) - x(off_x);
    Dv(off_v + 1) = x_new(off_x + 1) - x(off_x + 1);
    Dv(off_v + 2) = x_new(off_x + 2) - x(off_x + 2);

    // ROTATION (quaternions): Dq_loc = q_old^-1 * q_new,
    //  because   q_new = Dq_abs * q_old   = q_old * Dq_loc
    ChQuaternion<> q_old(x.segment(off_x + 3, 4));
    ChQuaternion<> q_new(x_new.segment(off_x + 3, 4));
    ChQuaternion<> rel_q = q_old.GetConjugate() * q_new;
    Dv.segment(off_v + 3, 3) = rel_q.GetRotVec().eigen();
}

void ChBody::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                               ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                               const double c           // a scaling factor
) {
    // add applied forces to 'fb' vector
    R.segment(off, 3) += c * Xforce.eigen();

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (!IsUsingGyroTorque())
        R.segment(off + 3, 3) += c * Xtorque.eigen();
    else
        R.segment(off + 3, 3) += c * (Xtorque - gyro).eigen();
}

void ChBody::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                const ChVectorDynamic<>& w,  // the w vector
                                const double c               // a scaling factor
) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    ChVector3d Iw = GetInertia() * ChVector3d(w.segment(off + 3, 3));
    Iw *= c;
    R.segment(off + 3, 3) += Iw.eigen();
}

void ChBody::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    Md(off + 0) += c * GetMass();
    Md(off + 1) += c * GetMass();
    Md(off + 2) += c * GetMass();
    Md(off + 3) += c * GetInertia()(0, 0);
    Md(off + 4) += c * GetInertia()(1, 1);
    Md(off + 5) += c * GetInertia()(2, 2);
    // if there is off-diagonal inertia, add to error, as lumping can give inconsistent results
    err += GetInertia()(0, 1) + GetInertia()(0, 2) + GetInertia()(1, 2);
}

void ChBody::IntToDescriptor(const unsigned int off_v,
                             const ChStateDelta& v,
                             const ChVectorDynamic<>& R,
                             const unsigned int off_L,
                             const ChVectorDynamic<>& L,
                             const ChVectorDynamic<>& Qc) {
    variables.State() = v.segment(off_v, 6);
    variables.Force() = R.segment(off_v, 6);
}

void ChBody::IntFromDescriptor(const unsigned int off_v,  // offset in v
                               ChStateDelta& v,
                               const unsigned int off_L,  // offset in L
                               ChVectorDynamic<>& L) {
    v.segment(off_v, 6) = variables.State();
}

////

void ChBody::InjectVariables(ChSystemDescriptor& descriptor) {
    variables.SetDisabled(!IsActive());

    descriptor.InsertVariables(&variables);
}

void ChBody::VariablesFbReset() {
    variables.Force().setZero();
}

void ChBody::VariablesFbLoadForces(double factor) {
    // add applied forces to 'fb' vector
    variables.Force().segment(0, 3) += factor * Xforce.eigen();

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (!IsUsingGyroTorque())
        variables.Force().segment(3, 3) += factor * Xtorque.eigen();
    else
        variables.Force().segment(3, 3) += factor * (Xtorque - gyro).eigen();
}

void ChBody::VariablesFbIncrementMq() {
    variables.AddMassTimesVector(variables.Force(), variables.State());
}

void ChBody::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variables.State().segment(0, 3) = GetCoordsysDt().pos.eigen();
    variables.State().segment(3, 3) = GetAngVelLocal().eigen();
}

void ChBody::VariablesQbSetSpeed(double step) {
    ChCoordsys<> old_coord_dt = GetCoordsysDt();

    // from 'qb' vector, sets body speed, and updates auxiliary data
    SetPosDt(variables.State().segment(0, 3));
    SetAngVelLocal(variables.State().segment(3, 3));

    // apply limits (if in speed clamping mode) to speeds.
    ClampSpeed();

    // compute auxiliary gyroscopic forces
    ComputeGyro();

    // Compute accel. by BDF (approximate by differentiation);
    if (step) {
        SetPosDt2((GetCoordsysDt().pos - old_coord_dt.pos) / step);
        SetRotDt2((GetCoordsysDt().rot - old_coord_dt.rot) / step);
    }
}

void ChBody::VariablesQbIncrementPosition(double dt_step) {
    if (!IsActive())
        return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Euler step.

    ChVector3d newspeed(variables.State().segment(0, 3));
    ChVector3d newwel(variables.State().segment(3, 3));

    // ADVANCE POSITION: pos' = pos + dt * vel
    SetPos(GetPos() + newspeed * dt_step);

    // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot = GetRot();
    ChVector3d newwel_abs = GetRotMat() * newwel;
    double mangle = newwel_abs.Length() * dt_step;
    newwel_abs.Normalize();
    mdeltarot.SetFromAngleAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot * moldrot;
    SetRot(mnewrot);
}

void ChBody::ForceToRest() {
    SetPosDt(VNULL);
    SetAngVelLocal(VNULL);
    SetPosDt2(VNULL);
    SetRotDt2(QNULL);
}

void ChBody::ClampSpeed() {
    if (limit_speed) {
        double w = 2.0 * GetRotDt().Length();
        if (w > max_wvel)
            SetRotDt(GetRotDt() * (max_wvel / w));

        double v = GetPosDt().Length();
        if (v > max_speed)
            SetPosDt(GetPosDt() * (max_speed / v));
    }
}

// The inertia tensor functions

void ChBody::SetInertia(const ChMatrix33<>& newXInertia) {
    variables.SetBodyInertia(newXInertia);
}

void ChBody::SetInertiaXX(const ChVector3d& iner) {
    variables.GetBodyInertia()(0, 0) = iner.x();
    variables.GetBodyInertia()(1, 1) = iner.y();
    variables.GetBodyInertia()(2, 2) = iner.z();
    variables.GetBodyInvInertia() = variables.GetBodyInertia().inverse();
}

void ChBody::SetInertiaXY(const ChVector3d& iner) {
    variables.GetBodyInertia()(0, 1) = iner.x();
    variables.GetBodyInertia()(0, 2) = iner.y();
    variables.GetBodyInertia()(1, 2) = iner.z();
    variables.GetBodyInertia()(1, 0) = iner.x();
    variables.GetBodyInertia()(2, 0) = iner.y();
    variables.GetBodyInertia()(2, 1) = iner.z();
    variables.GetBodyInvInertia() = variables.GetBodyInertia().inverse();
}

ChVector3d ChBody::GetInertiaXX() const {
    ChVector3d iner;
    iner.x() = variables.GetBodyInertia()(0, 0);
    iner.y() = variables.GetBodyInertia()(1, 1);
    iner.z() = variables.GetBodyInertia()(2, 2);
    return iner;
}

ChVector3d ChBody::GetInertiaXY() const {
    ChVector3d iner;
    iner.x() = variables.GetBodyInertia()(0, 1);
    iner.y() = variables.GetBodyInertia()(0, 2);
    iner.z() = variables.GetBodyInertia()(1, 2);
    return iner;
}

void ChBody::ComputeQInertia(ChMatrix44<>& mQInertia) {
    // [Iq]=[G'][Ix][G]
    ChGlMatrix34<> Gl(GetRot());
    mQInertia = Gl.transpose() * GetInertia() * Gl;
}

// -----------------------------------------------------------------------------

unsigned int ChBody::AddAccumulator() {
    auto idx = accumulators.size();
    accumulators.push_back(WrenchAccumulator());
    return (unsigned int)idx;
}

void ChBody::EmptyAccumulator(unsigned int idx) {
    accumulators[idx].force = VNULL;
    accumulators[idx].torque = VNULL;
}

void ChBody::AccumulateForce(unsigned int idx, const ChVector3d& force, const ChVector3d& appl_point, bool local) {
    ChWrenchd w_abs = local ? AppliedForceLocalToWrenchParent(force, appl_point)
                            : AppliedForceParentToWrenchParent(force, appl_point);
    accumulators[idx].force += w_abs.force;
    accumulators[idx].torque += TransformDirectionParentToLocal(w_abs.torque);
}

void ChBody::AccumulateTorque(unsigned int idx, const ChVector3d& torque, bool local) {
    if (local)
        accumulators[idx].torque += torque;
    else
        accumulators[idx].torque += TransformDirectionParentToLocal(torque);
}

const ChVector3d& ChBody::GetAccumulatedForce(unsigned int idx) const {
    return accumulators[idx].force;
}

const ChVector3d& ChBody::GetAccumulatedTorque(unsigned int idx) const {
    return accumulators[idx].torque;
}

//// TODO - get rid of this as soon as ChModalAssembly is dealt with (fixed or removed)
const ChWrenchd ChBody::GetAccumulatorWrench() const {
    ChWrenchd wrench;
    for (const auto& a : accumulators) {
        wrench.force += a.force;
        wrench.torque += a.torque;
    }
    return wrench;
}

// -----------------------------------------------------------------------------

void ChBody::ComputeGyro() {
    ChVector3d Wvel = GetAngVelLocal();
    gyro = Vcross(Wvel, variables.GetBodyInertia() * Wvel);
}

bool ChBody::TrySleeping() {
    candidate_sleeping = false;

    if (IsSleepingAllowed()) {
        if (!IsActive())
            return false;

        // if not yet sleeping:
        if ((GetPosDt().LengthInf() < sleep_minspeed) && (2.0 * GetRotDt().LengthInf() < sleep_minwvel)) {
            if ((GetChTime() - sleep_starttime) > sleep_time) {
                candidate_sleeping = true;
                return true;  // could go to sleep!
            }
        } else {
            sleep_starttime = float(GetChTime());
        }
    }
    return false;
}

void ChBody::AddMarker(std::shared_ptr<ChMarker> amarker) {
    // don't allow double insertion of same object
    assert(std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), amarker) ==
           marklist.end());

    amarker->SetBody(this);
    marklist.push_back(amarker);

    // If the body is already added to a system, mark the system uninitialized and out-of-date
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
}

void ChBody::AddForce(std::shared_ptr<ChForce> aforce) {
    // don't allow double insertion of same object
    assert(std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), aforce) ==
           forcelist.end());

    aforce->SetBody(this);
    forcelist.push_back(aforce);

    // If the body is already added to a system, mark the system uninitialized and out-of-date
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
}

void ChBody::RemoveForce(std::shared_ptr<ChForce> mforce) {
    // trying to remove objects not previously added?
    assert(std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), mforce) !=
           forcelist.end());

    // warning! linear time search
    forcelist.erase(
        std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), mforce));

    mforce->SetBody(0);

    // If the body is already added to a system, mark the system out-of-date
    if (system) {
        system->is_updated = false;
    }
}

void ChBody::RemoveMarker(std::shared_ptr<ChMarker> mmarker) {
    // trying to remove objects not previously added?
    assert(std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), mmarker) !=
           marklist.end());

    // warning! linear time search
    marklist.erase(
        std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), mmarker));

    mmarker->SetBody(0);

    // If the body is already added to a system, mark the system out-of-date
    if (system) {
        system->is_updated = false;
    }
}

void ChBody::RemoveAllForces() {
    for (auto& force : forcelist) {
        force->SetBody(NULL);
    }
    forcelist.clear();
}

void ChBody::RemoveAllMarkers() {
    for (auto& marker : marklist) {
        marker->SetBody(NULL);
    }
    marklist.clear();
}

std::shared_ptr<ChMarker> ChBody::SearchMarker(const std::string& name) const {
    auto marker = std::find_if(std::begin(marklist), std::end(marklist),
                               [name](std::shared_ptr<ChMarker> marker) { return marker->GetName() == name; });
    return (marker != std::end(marklist)) ? *marker : nullptr;
}

std::shared_ptr<ChMarker> ChBody::SearchMarker(int id) const {
    auto marker = std::find_if(std::begin(marklist), std::end(marklist),
                               [id](std::shared_ptr<ChMarker> marker) { return marker->GetIdentifier() == id; });
    return (marker != std::end(marklist)) ? *marker : nullptr;
}

std::shared_ptr<ChForce> ChBody::SearchForce(const std::string& name) const {
    auto force = std::find_if(std::begin(forcelist), std::end(forcelist),
                              [name](std::shared_ptr<ChForce> force) { return force->GetName() == name; });
    return (force != std::end(forcelist)) ? *force : nullptr;
}

// -----------------------------------------------------------------------------

void ChBody::UpdateMarkers(double time, bool update_assets) {
    for (auto& marker : marklist) {
        marker->Update(time, update_assets);
    }
}

void ChBody::UpdateForces(double time, bool update_assets) {
    // Initialize body forces with gravitational forces (if included in a system)
    Xforce = system ? system->GetGravitationalAcceleration() * GetMass() : VNULL;
    Xtorque = VNULL;

    // Add forces and torques from body accumulators (if any)
    for (const auto& a : accumulators) {
        Xforce += a.force;
        Xtorque += a.torque;
    }

    // Accumulate applied generalized forces (if any)
    for (auto& f : forcelist) {
        f->Update(time, update_assets);

        ChVector3d force;
        ChVector3d torque;
        f->GetBodyForceTorque(force, torque);

        Xforce += force;
        Xtorque += torque;
    }
}

void ChBody::Update(double time, bool update_assets) {
    // Update time and assets
    ChObj::Update(time, update_assets);

    // Test if body can be set asleep and if so, put it to sleeping
    ////TrySleeping();

    // Apply limits (if in speed clamping mode) to speeds
    ClampSpeed();

    // Set the gyroscopic momentum
    ComputeGyro();

    // Updated associated markers at current body state
    UpdateMarkers(time, update_assets);

    // Update applied forces at current body state
    UpdateForces(time, update_assets);
}

// -----------------------------------------------------------------------------

void ChBody::SetFixed(bool state) {
    variables.SetDisabled(state);
    fixed = state;
}

bool ChBody::IsFixed() const {
    return fixed;
}

void ChBody::SetLimitSpeed(bool state) {
    limit_speed = state;
}

void ChBody::SetUseGyroTorque(bool state) {
    disable_gyrotorque = !state;
}

bool ChBody::IsUsingGyroTorque() const {
    return !disable_gyrotorque;
}

void ChBody::SetSleepingAllowed(bool state) {
    allow_sleeping = state;
}

bool ChBody::IsSleepingAllowed() const {
    return allow_sleeping;
}

void ChBody::SetSleeping(bool state) {
    is_sleeping = state;
}

bool ChBody::IsSleeping() const {
    return is_sleeping;
}

bool ChBody::IsActive() const {
    return !is_sleeping && !fixed;
}

// ---------------------------------------------------------------------------
// Collision-related functions

void ChBody::EnableCollision(bool state) {
    // Nothing to do if no change in state
    if (state == collide)
        return;

    collide = state;

    // Nothing to do if body has no collision model
    if (!collision_model)
        return;

    // Nothing to do if not attached to a system
    if (!GetSystem())
        return;

    // Nothing to do if no collision system or the system was not initialized
    // (in the latter case, the collision model will be processed at initialization)
    auto coll_sys = GetSystem()->GetCollisionSystem();
    if (!coll_sys || !coll_sys->IsInitialized())
        return;

    // If enabling collision, add to collision system if not already processed
    if (collide && !collision_model->HasImplementation()) {
        coll_sys->Add(collision_model);
        return;
    }

    // If disabling collision, remove from collision system if already processed
    if (!collide && collision_model->HasImplementation()) {
        coll_sys->Remove(collision_model);
        return;
    }
}

bool ChBody::IsCollisionEnabled() const {
    return collide;
}

void ChBody::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    if (collide && collision_model)
        coll_sys->Add(collision_model);
}

void ChBody::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    if (collision_model)
        coll_sys->Remove(collision_model);
}

void ChBody::SyncCollisionModels() {
    // Sync model only if
    //    (1) a collision model was specified for the body
    //    (2) the body is set to participate in collisions
    // ChCollisionModel::SyncPosition will further check that the collision model was actually processed (through
    // BindAll or BindItem) by the current collision system.

    if (GetCollisionModel() && IsCollisionEnabled())
        GetCollisionModel()->SyncPosition();
}

// ---------------------------------------------------------------------------

ChAABB ChBody::GetTotalAABB() const {
    if (GetCollisionModel())
        return GetCollisionModel()->GetBoundingBox();

    return ChAABB();  // default: inverted bounding box
}

void ChBody::ContactableGetStateBlockPosLevel(ChState& x) {
    x.segment(0, 3) = GetCoordsys().pos.eigen();
    x.segment(3, 4) = GetCoordsys().rot.eigen();
}

void ChBody::ContactableGetStateBlockVelLevel(ChStateDelta& w) {
    w.segment(0, 3) = GetPosDt().eigen();
    w.segment(3, 3) = GetAngVelLocal().eigen();
}

void ChBody::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    IntStateIncrement(0, x_new, x, 0, dw);
}

ChVector3d ChBody::GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    return csys.TransformPointLocalToParent(loc_point);
}

ChVector3d ChBody::GetContactPointSpeed(const ChVector3d& loc_point,
                                        const ChState& state_x,
                                        const ChStateDelta& state_w) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    ChVector3d abs_vel(state_w.segment(0, 3));
    ChVector3d loc_omg(state_w.segment(3, 3));
    ChVector3d abs_omg = csys.TransformDirectionLocalToParent(loc_omg);

    return abs_vel + Vcross(abs_omg, loc_point);
}

ChVector3d ChBody::GetContactPointSpeed(const ChVector3d& abs_point) {
    ChVector3d point_loc = TransformPointParentToLocal(abs_point);
    return PointSpeedLocalToParent(point_loc);
}

ChFrame<> ChBody::GetCollisionModelFrame() {
    return GetFrameRefToAbs();
}

void ChBody::ContactForceLoadResidual_F(const ChVector3d& F,
                                        const ChVector3d& T,
                                        const ChVector3d& abs_point,
                                        ChVectorDynamic<>& R) {
    ChVector3d point_loc = TransformPointParentToLocal(abs_point);
    ChVector3d force1_loc = this->TransformDirectionParentToLocal(F);
    ChVector3d torque1_loc = Vcross(point_loc, force1_loc);
    if (!T.IsNull())
        torque1_loc += this->TransformDirectionParentToLocal(T);
    R.segment(this->GetOffset_w() + 0, 3) += F.eigen();
    R.segment(this->GetOffset_w() + 3, 3) += torque1_loc.eigen();
}

void ChBody::ContactComputeQ(const ChVector3d& F,
                             const ChVector3d& T,
                             const ChVector3d& point,
                             const ChState& state_x,
                             ChVectorDynamic<>& Q,
                             int offset) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    ChVector3d point_loc = csys.TransformPointParentToLocal(point);
    ChVector3d force_loc = csys.TransformDirectionParentToLocal(F);
    ChVector3d torque_loc = Vcross(point_loc, force_loc);
    if (!T.IsNull())
        torque_loc += csys.TransformDirectionParentToLocal(T);
    Q.segment(offset + 0, 3) = F.eigen();
    Q.segment(offset + 3, 3) = torque_loc.eigen();
}

void ChBody::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                           ChMatrix33<>& contact_plane,
                                           ChConstraintTuple* jacobian_tuple_N,
                                           ChConstraintTuple* jacobian_tuple_U,
                                           ChConstraintTuple* jacobian_tuple_V,
                                           bool second) {
    /*
    ChVector3d p1 = TransformPointParentToLocal(abs_point);
    ChStarMatrix33<> Ps1(p1);

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    ChMatrix33<> Jr1 = contact_plane.transpose() * GetRotMat() * Ps1;
    if (!second)
        Jr1 *= -1;

    jacobian_tuple_N.Get_Cq().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq().segment(0, 3) = Jx1.row(2);

    jacobian_tuple_N.Get_Cq().segment(3, 3) = Jr1.row(0);
    jacobian_tuple_U.Get_Cq().segment(3, 3) = Jr1.row(1);
    jacobian_tuple_V.Get_Cq().segment(3, 3) = Jr1.row(2);
    */

    // UNROLLED VERSION - FASTER
    ChVector3d p1 = TransformPointParentToLocal(abs_point);
    double temp00 =
        m_rmat(0, 2) * contact_plane(0, 0) + m_rmat(1, 2) * contact_plane(1, 0) + m_rmat(2, 2) * contact_plane(2, 0);
    double temp01 =
        m_rmat(0, 2) * contact_plane(0, 1) + m_rmat(1, 2) * contact_plane(1, 1) + m_rmat(2, 2) * contact_plane(2, 1);
    double temp02 =
        m_rmat(0, 2) * contact_plane(0, 2) + m_rmat(1, 2) * contact_plane(1, 2) + m_rmat(2, 2) * contact_plane(2, 2);
    double temp10 =
        m_rmat(0, 1) * contact_plane(0, 0) + m_rmat(1, 1) * contact_plane(1, 0) + m_rmat(2, 1) * contact_plane(2, 0);
    double temp11 =
        m_rmat(0, 1) * contact_plane(0, 1) + m_rmat(1, 1) * contact_plane(1, 1) + m_rmat(2, 1) * contact_plane(2, 1);
    double temp12 =
        m_rmat(0, 1) * contact_plane(0, 2) + m_rmat(1, 1) * contact_plane(1, 2) + m_rmat(2, 1) * contact_plane(2, 2);
    double temp20 =
        m_rmat(0, 0) * contact_plane(0, 0) + m_rmat(1, 0) * contact_plane(1, 0) + m_rmat(2, 0) * contact_plane(2, 0);
    double temp21 =
        m_rmat(0, 0) * contact_plane(0, 1) + m_rmat(1, 0) * contact_plane(1, 1) + m_rmat(2, 0) * contact_plane(2, 1);
    double temp22 =
        m_rmat(0, 0) * contact_plane(0, 2) + m_rmat(1, 0) * contact_plane(1, 2) + m_rmat(2, 0) * contact_plane(2, 2);

    // Jx1 =
    // [ c00, c10, c20]
    // [ c01, c11, c21]
    // [ c02, c12, c22]
    // Jr1 = [ p1y*(temp00) - p1z*(temp10), p1z*(temp20) - p1x*(temp00), p1x*(temp10) - p1y*(temp20);
    //       p1y*(temp01) - p1z*(temp11), p1z*(temp21) - p1x*(temp01), p1x*(temp11) - p1y*(temp21);
    //       p1y*(temp02) - p1z*(temp12), p1z*(temp22) - p1x*(temp02), p1x*(temp12) - p1y*(temp22)];

    auto tuple_N = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_V);

    auto Cq_N = tuple_N->Cq1();
    auto Cq_U = tuple_U->Cq1();
    auto Cq_V = tuple_V->Cq1();

    if (!second) {
        Cq_N(0) = -contact_plane(0, 0);
        Cq_N(1) = -contact_plane(1, 0);
        Cq_N(2) = -contact_plane(2, 0);
        Cq_N(3) = -p1.y() * temp00 + p1.z() * temp10;
        Cq_N(4) = -p1.z() * temp20 + p1.x() * temp00;
        Cq_N(5) = -p1.x() * temp10 + p1.y() * temp20;

        Cq_U(0) = -contact_plane(0, 1);
        Cq_U(1) = -contact_plane(1, 1);
        Cq_U(2) = -contact_plane(2, 1);
        Cq_U(3) = -p1.y() * temp01 + p1.z() * temp11;
        Cq_U(4) = -p1.z() * temp21 + p1.x() * temp01;
        Cq_U(5) = -p1.x() * temp11 + p1.y() * temp21;

        Cq_V(0) = -contact_plane(0, 2);
        Cq_V(1) = -contact_plane(1, 2);
        Cq_V(2) = -contact_plane(2, 2);
        Cq_V(3) = -p1.y() * temp02 + p1.z() * temp12;
        Cq_V(4) = -p1.z() * temp22 + p1.x() * temp02;
        Cq_V(5) = -p1.x() * temp12 + p1.y() * temp22;
    } else {
        Cq_N(0) = contact_plane(0, 0);
        Cq_N(1) = contact_plane(1, 0);
        Cq_N(2) = contact_plane(2, 0);
        Cq_N(3) = p1.y() * temp00 - p1.z() * temp10;
        Cq_N(4) = p1.z() * temp20 - p1.x() * temp00;
        Cq_N(5) = p1.x() * temp10 - p1.y() * temp20;

        Cq_U(0) = contact_plane(0, 1);
        Cq_U(1) = contact_plane(1, 1);
        Cq_U(2) = contact_plane(2, 1);
        Cq_U(3) = p1.y() * temp01 - p1.z() * temp11;
        Cq_U(4) = p1.z() * temp21 - p1.x() * temp01;
        Cq_U(5) = p1.x() * temp11 - p1.y() * temp21;

        Cq_V(0) = contact_plane(0, 2);
        Cq_V(1) = contact_plane(1, 2);
        Cq_V(2) = contact_plane(2, 2);
        Cq_V(3) = p1.y() * temp02 - p1.z() * temp12;
        Cq_V(4) = p1.z() * temp22 - p1.x() * temp02;
        Cq_V(5) = p1.x() * temp12 - p1.y() * temp22;
    }
}

void ChBody::ComputeJacobianForRollingContactPart(const ChVector3d& abs_point,
                                                  ChMatrix33<>& contact_plane,
                                                  ChConstraintTuple* jacobian_tuple_N,
                                                  ChConstraintTuple* jacobian_tuple_U,
                                                  ChConstraintTuple* jacobian_tuple_V,
                                                  bool second) {
    ChMatrix33<> Jr1 = contact_plane.transpose() * GetRotMat();
    if (!second)
        Jr1 *= -1;

    auto tuple_N = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_V);

    auto Cq_N = tuple_N->Cq1();
    auto Cq_U = tuple_U->Cq1();
    auto Cq_V = tuple_V->Cq1();

    Cq_N.segment(0, 3).setZero();
    Cq_U.segment(0, 3).setZero();
    Cq_V.segment(0, 3).setZero();

    Cq_N.segment(3, 3) = Jr1.row(0);
    Cq_U.segment(3, 3) = Jr1.row(1);
    Cq_V.segment(3, 3) = Jr1.row(2);
}

ChVector3d ChBody::GetAppliedForce() {
    return GetSystem()->GetBodyAppliedForce(this);
}

ChVector3d ChBody::GetAppliedTorque() {
    return GetSystem()->GetBodyAppliedTorque(this);
}

ChVector3d ChBody::GetContactForce() {
    return GetSystem()->GetContactContainer()->GetContactableForce(this);
}

ChVector3d ChBody::GetContactTorque() {
    return GetSystem()->GetContactContainer()->GetContactableTorque(this);
}

// ---------------------------------------------------------------------------

void ChBody::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&Variables());
}

void ChBody::LoadableStateIncrement(const unsigned int off_x,
                                    ChState& x_new,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& Dv) {
    IntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChBody::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = GetCoordsys().pos.eigen();
    mD.segment(block_offset + 3, 4) = GetCoordsys().rot.eigen();
}

void ChBody::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = GetPosDt().eigen();
    mD.segment(block_offset + 3, 3) = GetAngVelLocal().eigen();
}

void ChBody::ComputeNF(
    const double U,              // x coordinate of application point in absolute space
    const double V,              // y coordinate of application point in absolute space
    const double W,              // z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is 6, it is {Force,Torque} in absolute coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChVector3d abs_pos(U, V, W);
    ChVector3d absF(F.segment(0, 3));
    ChVector3d absT(F.segment(3, 3));
    ChVector3d body_absF;
    ChVector3d body_locT;
    ChCoordsys<> bodycoord;
    if (state_x)
        bodycoord = state_x->segment(0, 7);  // the numerical jacobian algo might change state_x
    else
        bodycoord = m_csys;

    // compute Q components F,T, given current state of body 'bodycoord'. Note T in Q is in local csys, F is an abs csys
    body_absF = absF;
    body_locT = bodycoord.rot.RotateBack(absT + ((abs_pos - bodycoord.pos) % absF));
    Qi.segment(0, 3) = body_absF.eigen();
    Qi.segment(3, 3) = body_locT.eigen();
    detJ = 1;  // not needed because not used in quadrature.
}

// ---------------------------------------------------------------------------
// FILE I/O

void ChBody::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChBody>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(archive_out);
    // serialize parent class
    ChBodyFrame::ArchiveOut(archive_out);

    // serialize all member data:

    archive_out << CHNVP(fixed);
    archive_out << CHNVP(collide);

    archive_out << CHNVP(limit_speed);
    archive_out << CHNVP(disable_gyrotorque);
    archive_out << CHNVP(is_sleeping);
    archive_out << CHNVP(allow_sleeping);
    archive_out << CHNVP(candidate_sleeping);

    archive_out << CHNVP(marklist, "markers");
    archive_out << CHNVP(forcelist, "forces");

    archive_out << CHNVP(collision_model);
    archive_out << CHNVP(gyro);
    archive_out << CHNVP(Xforce);
    archive_out << CHNVP(Xtorque);
    archive_out << CHNVP(variables);
    archive_out << CHNVP(max_speed);
    archive_out << CHNVP(max_wvel);
    archive_out << CHNVP(sleep_time);
    archive_out << CHNVP(sleep_minspeed);
    archive_out << CHNVP(sleep_minwvel);
    archive_out << CHNVP(sleep_starttime);
}

/// Method to allow de serialization of transient data from archives.
void ChBody::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChBody>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIn(archive_in);
    // deserialize parent class
    ChBodyFrame::ArchiveIn(archive_in);

    // stream in all member data:

    archive_in >> CHNVP(fixed);
    archive_in >> CHNVP(collide);

    archive_in >> CHNVP(limit_speed);
    archive_in >> CHNVP(disable_gyrotorque);
    archive_in >> CHNVP(is_sleeping);
    archive_in >> CHNVP(allow_sleeping);
    archive_in >> CHNVP(candidate_sleeping);

    std::vector<std::shared_ptr<ChMarker>> tempmarkers;
    std::vector<std::shared_ptr<ChForce>> tempforces;
    archive_in >> CHNVP(tempmarkers, "markers");
    archive_in >> CHNVP(tempforces, "forces");
    // trick needed because the "Add...() functions are required
    RemoveAllMarkers();
    for (auto& i : tempmarkers) {
        AddMarker(i);
    }
    RemoveAllForces();
    for (auto& i : tempforces) {
        AddForce(i);
    }

    std::shared_ptr<ChCollisionModel> collision_model_temp;  ///< pointer to the collision model
    archive_in >> CHNVP(collision_model_temp, "collision_model");
    if (collision_model_temp)
        AddCollisionModel(collision_model_temp);

    archive_in >> CHNVP(gyro);
    archive_in >> CHNVP(Xforce);
    archive_in >> CHNVP(Xtorque);
    archive_in >> CHNVP(variables);
    archive_in >> CHNVP(max_speed);
    archive_in >> CHNVP(max_wvel);
    archive_in >> CHNVP(sleep_time);
    archive_in >> CHNVP(sleep_minspeed);
    archive_in >> CHNVP(sleep_minwvel);
    archive_in >> CHNVP(sleep_starttime);
}

}  // end namespace chrono
