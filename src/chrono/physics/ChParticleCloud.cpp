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

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/collision/ChCollisionSystem.h"

namespace chrono {

// -----------------------------------------------------------------------------
// CLASS FOR A PARTICLE
// -----------------------------------------------------------------------------

ChParticle::ChParticle() : container(NULL), UserForce(VNULL), UserTorque(VNULL) {
    // Load contactable variables list
    m_contactable_variables.push_back(&variables);
}

ChParticle::ChParticle(const ChParticle& other) : ChParticleBase(other) {
    container = other.container;
    UserForce = other.UserForce;
    UserTorque = other.UserTorque;
    variables = other.variables;

    m_contactable_variables.clear();
    m_contactable_variables.push_back(&variables);
}

ChParticle::~ChParticle() {}

ChParticle& ChParticle::operator=(const ChParticle& other) {
    if (&other == this)
        return *this;

    // parent class copy
    ChParticleBase::operator=(other);

    collision_model->Clear();
    collision_model->AddShapes(other.collision_model);
    collision_model->SetContactable(this);

    container = other.container;
    UserForce = other.UserForce;
    UserTorque = other.UserTorque;
    variables = other.variables;

    return *this;
}

void ChParticle::ContactableGetStateBlockVelLevel(ChStateDelta& w) {
    w.segment(0, 3) = GetPosDt().eigen();
    w.segment(3, 3) = GetAngVelLocal().eigen();
}

void ChParticle::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    // Increment position
    x_new(0) = x(0) + dw(0);
    x_new(1) = x(1) + dw(1);
    x_new(2) = x(2) + dw(2);

    // Increment rotation: rot' = delta*rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot(x.segment(3, 4));
    ChVector3d newwel_abs = m_rmat * ChVector3d(dw.segment(3, 3));
    double mangle = newwel_abs.Length();
    newwel_abs.Normalize();
    mdeltarot.SetFromAngleAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot * moldrot;  // quaternion product
    x_new.segment(3, 4) = mnewrot.eigen();
}

ChVector3d ChParticle::GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    return csys.TransformPointLocalToParent(loc_point);
}

ChVector3d ChParticle::GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    ChVector3d abs_vel(state_w.segment(0, 3));
    ChVector3d loc_omg(state_w.segment(3, 3));
    ChVector3d abs_omg = csys.TransformDirectionLocalToParent(loc_omg);

    return abs_vel + Vcross(abs_omg, loc_point);
}

ChVector3d ChParticle::GetContactPointSpeed(const ChVector3d& abs_point) {
    ChVector3d m_p1_loc = this->TransformPointParentToLocal(abs_point);
    return this->PointSpeedLocalToParent(m_p1_loc);
}

void ChParticle::ContactForceLoadResidual_F(const ChVector3d& F,
                                            const ChVector3d& T,
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) {
    ChVector3d m_p1_loc = this->TransformPointParentToLocal(abs_point);
    ChVector3d force1_loc = this->TransformDirectionParentToLocal(F);
    ChVector3d torque1_loc = Vcross(m_p1_loc, force1_loc);
    if (!T.IsNull())
        torque1_loc += this->TransformDirectionParentToLocal(T);
    R.segment(variables.GetOffset() + 0, 3) += F.eigen();
    R.segment(variables.GetOffset() + 3, 3) += torque1_loc.eigen();
}

void ChParticle::ContactComputeQ(const ChVector3d& F,
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

void ChParticle::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChConstraintTuple* jacobian_tuple_N,
                                               ChConstraintTuple* jacobian_tuple_U,
                                               ChConstraintTuple* jacobian_tuple_V,
                                               bool second) {
    ChVector3d m_p1_loc = this->TransformPointParentToLocal(abs_point);

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    ChStarMatrix33<> Ps1(m_p1_loc);
    ChMatrix33<> Jr1 = contact_plane.transpose() * GetRotMat() * Ps1;
    if (second)
        Jr1 *= -1;

    auto tuple_N = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_V);

    tuple_N->Cq1().segment(0, 3) = Jx1.row(0);
    tuple_U->Cq1().segment(0, 3) = Jx1.row(1);
    tuple_V->Cq1().segment(0, 3) = Jx1.row(2);

    tuple_N->Cq1().segment(3, 3) = Jr1.row(0);
    tuple_U->Cq1().segment(3, 3) = Jr1.row(1);
    tuple_V->Cq1().segment(3, 3) = Jr1.row(2);
}

void ChParticle::ComputeJacobianForRollingContactPart(const ChVector3d& abs_point,
                                                      ChMatrix33<>& contact_plane,
                                                      ChConstraintTuple* jacobian_tuple_N,
                                                      ChConstraintTuple* jacobian_tuple_U,
                                                      ChConstraintTuple* jacobian_tuple_V,
                                                      bool second) {
    ChMatrix33<> Jr1 = contact_plane.transpose() * this->GetRotMat();
    if (!second)
        Jr1 *= -1;

    auto tuple_N = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<6>*>(jacobian_tuple_V);

    tuple_N->Cq1().segment(0, 3).setZero();
    tuple_U->Cq1().segment(0, 3).setZero();
    tuple_V->Cq1().segment(0, 3).setZero();

    tuple_N->Cq1().segment(3, 3) = Jr1.row(0);
    tuple_U->Cq1().segment(3, 3) = Jr1.row(1);
    tuple_V->Cq1().segment(3, 3) = Jr1.row(2);
}

ChPhysicsItem* ChParticle::GetPhysicsItem() {
    return container;
}

void ChParticle::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChParticle>();

    // serialize parent class
    ChParticleBase::ArchiveOut(archive_out);

    // serialize all member data:
    // archive_out << CHNVP(container);
    archive_out << CHNVP(collision_model);
    archive_out << CHNVP(UserForce);
    archive_out << CHNVP(UserTorque);
}

/// Method to allow de serialization of transient data from archives.
void ChParticle::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChParticle>();

    // deserialize parent class:
    ChParticleBase::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(collision_model);
    archive_in >> CHNVP(UserForce);
    archive_in >> CHNVP(UserTorque);
}

// -----------------------------------------------------------------------------
// CLASS FOR PARTICLE CLUSTER
// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChParticleCloud)

ChParticleCloud::ChParticleCloud()
    : collide(false),
      limit_speed(false),
      fixed(false),
      max_speed(0.5f),
      max_wvel((float)CH_2PI),
      sleep_time(0.6f),
      sleep_starttime(0),
      sleep_minspeed(0.1f),
      sleep_minwvel(0.04f),
      particle_collision_model(nullptr) {
    SetMass(1.0);
    SetInertiaXX(ChVector3d(1.0, 1.0, 1.0));
    SetInertiaXY(ChVector3d(0, 0, 0));

    particles.clear();
    // ResizeNparticles(num_particles); // caused memory corruption.. why?
}

ChParticleCloud::ChParticleCloud(const ChParticleCloud& other) : ChIndexedParticles(other) {
    collide = other.collide;
    limit_speed = other.limit_speed;

    SetMass(other.GetMass());
    SetInertiaXX(other.GetInertiaXX());
    SetInertiaXY(other.GetInertiaXY());

    if (other.particle_collision_model)
        particle_collision_model = chrono_types::make_shared<ChCollisionModel>(*other.particle_collision_model);
    else
        particle_collision_model = nullptr;

    ResizeNparticles((unsigned int)other.GetNumParticles());

    max_speed = other.max_speed;
    max_wvel = other.max_wvel;

    sleep_time = other.sleep_time;
    sleep_starttime = other.sleep_starttime;
    sleep_minspeed = other.sleep_minspeed;
    sleep_minwvel = other.sleep_minwvel;
}

ChParticleCloud::~ChParticleCloud() {
    ResizeNparticles(0);
}

void ChParticleCloud::AddCollisionShape(std::shared_ptr<ChCollisionShape> shape, const ChFrame<>& frame) {
    if (!particle_collision_model) {
        particle_collision_model = chrono_types::make_shared<ChCollisionModel>();
    }
    particle_collision_model->AddShape(shape, frame);
}

void ChParticleCloud::ResizeNparticles(int newsize) {
    //// TODO REVISIT THIS

    bool oldcoll = IsCollisionEnabled();
    EnableCollision(false);

    for (unsigned int j = 0; j < particles.size(); j++) {
        delete (particles[j]);
        particles[j] = 0;
    }

    particles.resize(newsize);

    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j] = new ChParticle;

        particles[j]->SetContainer(this);

        particles[j]->variables.SetSharedMass(&particle_mass);
        particles[j]->variables.SetUserData((void*)this);

        if (particle_collision_model) {
            auto collision_model = chrono_types::make_shared<ChCollisionModel>();
            collision_model->AddShapes(particle_collision_model);
            particles[j]->AddCollisionModel(collision_model);
        }
    }

    EnableCollision(oldcoll);
}

void ChParticleCloud::AddParticle(ChCoordsys<double> initial_state) {
    ChParticle* newp = new ChParticle;
    newp->SetCoordsys(initial_state);

    newp->SetContainer(this);

    newp->variables.SetSharedMass(&particle_mass);
    newp->variables.SetUserData((void*)this);

    if (particle_collision_model) {
        auto collision_model = chrono_types::make_shared<ChCollisionModel>();
        collision_model->AddShapes(particle_collision_model);
        newp->AddCollisionModel(collision_model);
    }

    particles.push_back(newp);
}

ChColor ChParticleCloud::GetVisualColor(unsigned int n) const {
    if (m_color_fun)
        return m_color_fun->get(n, *this);
    return ChColor();
}

bool ChParticleCloud::UseDynamicColors() const {
    return m_color_fun != nullptr;
}

bool ChParticleCloud::IsVisible(unsigned int n) const {
    if (m_vis_fun)
        return m_vis_fun->get(n, *this);
    return true;
}

// STATE BOOKKEEPING FUNCTIONS

void ChParticleCloud::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                     ChState& x,                // state vector, position part
                                     const unsigned int off_v,  // offset in v state vector
                                     ChStateDelta& v,           // state vector, speed part
                                     double& T                  // time
) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        x.segment(off_x + 7 * j + 0, 3) = particles[j]->GetPos().eigen();
        x.segment(off_x + 7 * j + 3, 4) = particles[j]->GetRot().eigen();

        v.segment(off_v + 6 * j + 0, 3) = particles[j]->GetPosDt().eigen();
        v.segment(off_v + 6 * j + 3, 3) = particles[j]->GetAngVelLocal().eigen();

        T = GetChTime();
    }
}

void ChParticleCloud::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                      const ChState& x,          // state vector, position part
                                      const unsigned int off_v,  // offset in v state vector
                                      const ChStateDelta& v,     // state vector, speed part
                                      const double T,            // time
                                      bool full_update           // perform complete update
) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetCoordsys(x.segment(off_x + 7 * j, 7));
        particles[j]->SetPosDt(v.segment(off_v + 6 * j, 3));
        particles[j]->SetAngVelLocal(v.segment(off_v + 6 * j + 3, 3));
    }
    SetChTime(T);
    Update(T, full_update);
}

void ChParticleCloud::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        a.segment(off_a + 6 * j + 0, 3) = particles[j]->GetPosDt2().eigen();
        a.segment(off_a + 6 * j + 3, 3) = particles[j]->GetAngAccLocal().eigen();
    }
}

void ChParticleCloud::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetPosDt2(a.segment(off_a + 6 * j, 3));
        particles[j]->SetAngAccLocal(a.segment(off_a + 6 * j + 3, 3));
    }
}

void ChParticleCloud::IntStateIncrement(const unsigned int off_x,  // offset in x state vector
                                        ChState& x_new,            // state vector, position part, incremented result
                                        const ChState& x,          // state vector, initial position part
                                        const unsigned int off_v,  // offset in v state vector
                                        const ChStateDelta& Dv     // state vector, increment
) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        // ADVANCE POSITION:
        x_new(off_x + 7 * j) = x(off_x + 7 * j) + Dv(off_v + 6 * j);
        x_new(off_x + 7 * j + 1) = x(off_x + 7 * j + 1) + Dv(off_v + 6 * j + 1);
        x_new(off_x + 7 * j + 2) = x(off_x + 7 * j + 2) + Dv(off_v + 6 * j + 2);

        // ADVANCE ROTATION: R_new = DR_a * R_old
        // (using quaternions, local or abs:  q_new = Dq_a * q_old =  q_old * Dq_l  )
        ChQuaternion<> q_old(x.segment(off_x + 7 * j + 3, 4));
        ChQuaternion<> rel_q;
        rel_q.SetFromRotVec(Dv.segment(off_v + 6 * j + 3, 3));
        ChQuaternion<> q_new = q_old * rel_q;
        x_new.segment(off_x + 7 * j + 3, 4) = q_new.eigen();
    }
}

void ChParticleCloud::IntStateGetIncrement(const unsigned int off_x,  // offset in x state vector
                                           const ChState& x_new,      // state vector, position part, incremented result
                                           const ChState& x,          // state vector, initial position part
                                           const unsigned int off_v,  // offset in v state vector
                                           ChStateDelta& Dv           // state vector, increment
) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        // POSITION:
        Dv(off_v + 6 * j) = x_new(off_x + 7 * j) - x(off_x + 7 * j);
        Dv(off_v + 6 * j + 1) = x_new(off_x + 7 * j + 1) - x(off_x + 7 * j + 1);
        Dv(off_v + 6 * j + 2) = x_new(off_x + 7 * j + 2) - x(off_x + 7 * j + 2);

        // ROTATION (quaternions): Dq_loc = q_old^-1 * q_new,
        //  because   q_new = Dq_abs * q_old   = q_old * Dq_loc
        ChQuaternion<> q_old(x.segment(off_x + 7 * j + 3, 4));
        ChQuaternion<> q_new(x_new.segment(off_x + 7 * j + 3, 4));
        ChQuaternion<> rel_q = q_old.GetConjugate() * q_new;
        Dv.segment(off_v + 6 * j + 3, 3) = rel_q.GetRotVec().eigen();
    }
}

void ChParticleCloud::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                        ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                        const double c           // a scaling factor
) {
    ChVector3d Gforce;
    if (GetSystem())
        Gforce = GetSystem()->GetGravitationalAcceleration() * particle_mass.GetBodyMass();

    for (unsigned int j = 0; j < particles.size(); j++) {
        // particle gyroscopic force:
        ChVector3d Wvel = particles[j]->GetAngVelLocal();
        ChVector3d gyro = Vcross(Wvel, particle_mass.GetBodyInertia() * Wvel);

        // add applied forces and torques (and also the gyroscopic torque and gravity!) to 'fb' vector
        R.segment(off + 6 * j + 0, 3) += c * (particles[j]->UserForce + Gforce).eigen();
        R.segment(off + 6 * j + 3, 3) += c * (particles[j]->UserTorque - gyro).eigen();
    }
}

void ChParticleCloud::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                         const ChVectorDynamic<>& w,  // the w vector
                                         const double c               // a scaling factor
) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        R(off + 6 * j + 0) += c * GetMass() * w(off + 6 * j + 0);
        R(off + 6 * j + 1) += c * GetMass() * w(off + 6 * j + 1);
        R(off + 6 * j + 2) += c * GetMass() * w(off + 6 * j + 2);
        ChVector3d Iw = c * (particle_mass.GetBodyInertia() * ChVector3d(w.segment(off + 6 * j + 3, 3)));
        R.segment(off + 6 * j + 3, 3) += Iw.eigen();
    }
}
void ChParticleCloud::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        Md(off + 6 * j + 0) += c * particle_mass.GetBodyMass();
        Md(off + 6 * j + 1) += c * particle_mass.GetBodyMass();
        Md(off + 6 * j + 2) += c * particle_mass.GetBodyMass();
        Md(off + 6 * j + 3) += c * particle_mass.GetBodyInertia()(0, 0);
        Md(off + 6 * j + 4) += c * particle_mass.GetBodyInertia()(1, 1);
        Md(off + 6 * j + 5) += c * particle_mass.GetBodyInertia()(2, 2);
    }
    // if there is off-diagonal inertia, add to error, as lumping can give inconsistent results
    err += particles.size() * (particle_mass.GetBodyInertia()(0, 1) + particle_mass.GetBodyInertia()(0, 2) +
                               particle_mass.GetBodyInertia()(1, 2));
}

void ChParticleCloud::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                      const ChStateDelta& v,
                                      const ChVectorDynamic<>& R,
                                      const unsigned int off_L,  // offset in L, Qc
                                      const ChVectorDynamic<>& L,
                                      const ChVectorDynamic<>& Qc) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.State() = v.segment(off_v + 6 * j, 6);
        particles[j]->variables.Force() = R.segment(off_v + 6 * j, 6);
    }
}

void ChParticleCloud::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                        ChStateDelta& v,
                                        const unsigned int off_L,  // offset in L
                                        ChVectorDynamic<>& L) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        v.segment(off_v + 6 * j, 6) = particles[j]->variables.State();
    }
}

void ChParticleCloud::InjectVariables(ChSystemDescriptor& descriptor) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.SetDisabled(!IsActive());
        descriptor.InsertVariables(&(particles[j]->variables));
    }
}

void ChParticleCloud::VariablesFbReset() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.Force().setZero();
    }
}

void ChParticleCloud::VariablesFbLoadForces(double factor) {
    ChVector3d Gforce;
    if (GetSystem())
        Gforce = GetSystem()->GetGravitationalAcceleration() * particle_mass.GetBodyMass();

    for (unsigned int j = 0; j < particles.size(); j++) {
        // particle gyroscopic force:
        ChVector3d Wvel = particles[j]->GetAngVelLocal();
        ChVector3d gyro = Vcross(Wvel, particle_mass.GetBodyInertia() * Wvel);

        // add applied forces and torques (and also the gyroscopic torque and gravity!) to 'fb' vector
        particles[j]->variables.Force().segment(0, 3) += factor * (particles[j]->UserForce + Gforce).eigen();
        particles[j]->variables.Force().segment(3, 3) += factor * (particles[j]->UserTorque - gyro).eigen();
    }
}

void ChParticleCloud::VariablesQbLoadSpeed() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        // set current speed in 'qb', it can be used by the solver when working in incremental mode
        particles[j]->variables.State().segment(0, 3) = particles[j]->GetCoordsysDt().pos.eigen();
        particles[j]->variables.State().segment(3, 3) = particles[j]->GetAngVelLocal().eigen();
    }
}

void ChParticleCloud::VariablesFbIncrementMq() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.AddMassTimesVector(particles[j]->variables.Force(), particles[j]->variables.State());
    }
}

void ChParticleCloud::VariablesQbSetSpeed(double step) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        ChCoordsys<> old_coord_dt = particles[j]->GetCoordsysDt();

        // from 'qb' vector, sets body speed, and updates auxiliary data
        particles[j]->SetPosDt(particles[j]->variables.State().segment(0, 3));
        particles[j]->SetAngVelLocal(particles[j]->variables.State().segment(3, 3));

        // apply limits (if in speed clamping mode) to speeds.
        // ClampSpeed(); NO - do only per-particle, here.. (but.. really needed here?)

        // Compute accel. by BDF (approximate by differentiation);
        if (step) {
            particles[j]->SetPosDt2((particles[j]->GetCoordsysDt().pos - old_coord_dt.pos) / step);
            particles[j]->SetRotDt2((particles[j]->GetCoordsysDt().rot - old_coord_dt.rot) / step);
        }
    }
}

void ChParticleCloud::VariablesQbIncrementPosition(double dt_step) {
    if (!IsActive())
        return;

    for (unsigned int j = 0; j < particles.size(); j++) {
        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Euler step.

        ChVector3d newspeed(particles[j]->variables.State().segment(0, 3));
        ChVector3d newwel(particles[j]->variables.State().segment(3, 3));

        // ADVANCE POSITION: pos' = pos + dt * vel
        particles[j]->SetPos(particles[j]->GetPos() + newspeed * dt_step);

        // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
        ChQuaternion<> mdeltarot;
        ChQuaternion<> moldrot = particles[j]->GetRot();
        ChVector3d newwel_abs = particles[j]->GetRotMat() * newwel;
        double mangle = newwel_abs.Length() * dt_step;
        newwel_abs.Normalize();
        mdeltarot.SetFromAngleAxis(mangle, newwel_abs);
        ChQuaternion<> mnewrot = mdeltarot * moldrot;
        particles[j]->SetRot(mnewrot);
    }
}

void ChParticleCloud::ForceToRest() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetPosDt(VNULL);
        particles[j]->SetAngVelLocal(VNULL);
        particles[j]->SetPosDt2(VNULL);
        particles[j]->SetRotDt2(QNULL);
    }
}

void ChParticleCloud::ClampSpeed() {
    if (limit_speed) {
        for (unsigned int j = 0; j < particles.size(); j++) {
            double w = 2.0 * particles[j]->GetRotDt().Length();
            if (w > max_wvel)
                particles[j]->SetRotDt(particles[j]->GetRotDt() * max_wvel / w);

            double v = particles[j]->GetPosDt().Length();
            if (v > max_speed)
                particles[j]->SetPosDt(particles[j]->GetPosDt() * max_speed / v);
        }
    }
}

// The inertia tensor functions

void ChParticleCloud::SetInertia(const ChMatrix33<>& newXInertia) {
    particle_mass.SetBodyInertia(newXInertia);
}

void ChParticleCloud::SetInertiaXX(const ChVector3d& iner) {
    particle_mass.GetBodyInertia()(0, 0) = iner.x();
    particle_mass.GetBodyInertia()(1, 1) = iner.y();
    particle_mass.GetBodyInertia()(2, 2) = iner.z();
    particle_mass.GetBodyInvInertia() = particle_mass.GetBodyInertia().inverse();
}
void ChParticleCloud::SetInertiaXY(const ChVector3d& iner) {
    particle_mass.GetBodyInertia()(0, 1) = iner.x();
    particle_mass.GetBodyInertia()(0, 2) = iner.y();
    particle_mass.GetBodyInertia()(1, 2) = iner.z();
    particle_mass.GetBodyInertia()(1, 0) = iner.x();
    particle_mass.GetBodyInertia()(2, 0) = iner.y();
    particle_mass.GetBodyInertia()(2, 1) = iner.z();
    particle_mass.GetBodyInvInertia() = particle_mass.GetBodyInertia().inverse();
}

ChVector3d ChParticleCloud::GetInertiaXX() const {
    ChVector3d iner;
    iner.x() = particle_mass.GetBodyInertia()(0, 0);
    iner.y() = particle_mass.GetBodyInertia()(1, 1);
    iner.z() = particle_mass.GetBodyInertia()(2, 2);
    return iner;
}

ChVector3d ChParticleCloud::GetInertiaXY() const {
    ChVector3d iner;
    iner.x() = particle_mass.GetBodyInertia()(0, 1);
    iner.y() = particle_mass.GetBodyInertia()(0, 2);
    iner.z() = particle_mass.GetBodyInertia()(1, 2);
    return iner;
}

void ChParticleCloud::Update(double time, bool update_assets) {
    ChPhysicsItem::Update(time, update_assets);

    // TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();  // Apply limits (if in speed clamping mode) to speeds.
}

void ChParticleCloud::EnableCollision(bool state) {
    // Nothing to do if no change in state
    if (state == collide)
        return;

    collide = state;

    // Nothing to do if body has no collision model
    if (!particle_collision_model)
        return;

    // Nothing to do if not attached to a system
    if (!GetSystem())
        return;

    // Nothing to do if no collision system or the system was not initialized
    // (in the latter case, the collsion model will be processed at initialization)
    auto coll_sys = GetSystem()->GetCollisionSystem();
    if (!coll_sys || !coll_sys->IsInitialized())
        return;

    // If enabling collision, add to collision system if not already processed
    if (collide && !particles.empty() && !particles[0]->GetCollisionModel()->HasImplementation()) {
        for (auto particle : particles)
            coll_sys->Add(particle->GetCollisionModel());
        return;
    }

    // If disabling collision, remove the from collision system if already processed
    if (!collide && !particles.empty() && particles[0]->GetCollisionModel()->HasImplementation()) {
        for (auto particle : particles)
            coll_sys->Remove(particle->GetCollisionModel());
        return;
    }
}

void ChParticleCloud::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    if (collide && particle_collision_model) {
        for (const auto& p : particles)
            coll_sys->Add(p->GetCollisionModel());
    }
}

void ChParticleCloud::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    if (particle_collision_model) {
        for (const auto& p : particles)
            coll_sys->Remove(p->GetCollisionModel());
    }
}

void ChParticleCloud::SyncCollisionModels() {
    // Sync model only if a collision model was specified for the particle cloud.
    // ChCollisionModel::SyncPosition will further check that the collision model was actually processed (through
    // BindAll or BindItem) by the current collision system.

    if (!particle_collision_model)
        return;

    for (auto particle : particles)
        particle->GetCollisionModel()->SyncPosition();
}

void ChParticleCloud::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChParticleCloud>();

    // serialize parent class
    ChIndexedParticles::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(particles);
    // archive_out << CHNVP(particle_mass); //// TODO
    archive_out << CHNVP(particle_collision_model);
    archive_out << CHNVP(collide);
    archive_out << CHNVP(limit_speed);
    archive_out << CHNVP(max_speed);
    archive_out << CHNVP(max_wvel);
    archive_out << CHNVP(sleep_time);
    archive_out << CHNVP(sleep_minspeed);
    archive_out << CHNVP(sleep_minwvel);
    archive_out << CHNVP(sleep_starttime);
}

void ChParticleCloud::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChParticleCloud>();

    // deserialize parent class:
    ChIndexedParticles::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(particles);
    // archive_in >> CHNVP(particle_mass); //// TODO
    archive_in >> CHNVP(particle_collision_model);
    archive_in >> CHNVP(collide);
    archive_in >> CHNVP(limit_speed);
    archive_in >> CHNVP(max_speed);
    archive_in >> CHNVP(max_wvel);
    archive_in >> CHNVP(sleep_time);
    archive_in >> CHNVP(sleep_minspeed);
    archive_in >> CHNVP(sleep_minwvel);
    archive_in >> CHNVP(sleep_starttime);

    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetContainer(this);
    }
}

}  // end namespace chrono
