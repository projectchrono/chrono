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

#include "chrono/collision/ChCModelBullet.h"
#include "chrono/core/ChLinearAlgebra.h"
#include "chrono/core/ChTransform.h"
#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// -----------------------------------------------------------------------------
// CLASS FOR A PARTICLE
// -----------------------------------------------------------------------------

ChAparticle::ChAparticle() : container(NULL), UserForce(VNULL), UserTorque(VNULL) {
    collision_model = new ChModelBullet;
    collision_model->SetContactable(this);
}

ChAparticle::ChAparticle(const ChAparticle& other) : ChParticleBase(other) {
    collision_model = new ChModelBullet;
    collision_model->AddCopyOfAnotherModel(other.collision_model);
    collision_model->SetContactable(this);

    container = other.container;
    UserForce = other.UserForce;
    UserTorque = other.UserTorque;
    variables = other.variables;
}

ChAparticle::~ChAparticle() {
    delete collision_model;
}

ChAparticle& ChAparticle::operator=(const ChAparticle& other) {
    if (&other == this)
        return *this;

    // parent class copy
    ChParticleBase::operator=(other);

    collision_model->ClearModel();
    collision_model->AddCopyOfAnotherModel(other.collision_model);
    collision_model->SetContactable(this);

    container = other.container;
    UserForce = other.UserForce;
    UserTorque = other.UserTorque;
    variables = other.variables;

    return *this;
}

std::shared_ptr<ChMaterialSurface>& ChAparticle::GetMaterialSurface() {
    return container->GetMaterialSurface();
}

void ChAparticle::ContactableGetStateBlock_w(ChStateDelta& w) {
    w.PasteVector(this->GetPos_dt(), 0, 0);
    w.PasteVector(this->GetWvel_loc(), 3, 0);
}

void ChAparticle::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    // Increment position
    x_new(0) = x(0) + dw(0);
    x_new(1) = x(1) + dw(1);
    x_new(2) = x(2) + dw(2);

    // Increment rotation: rot' = delta*rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot = x.ClipQuaternion(3, 0);
    ChVector<> newwel_abs = Amatrix * dw.ClipVector(3, 0);
    double mangle = newwel_abs.Length();
    newwel_abs.Normalize();
    mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot * moldrot;  // quaternion product
    x_new.PasteQuaternion(mnewrot, 3, 0);
}

ChVector<> ChAparticle::GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) {
    ChCoordsys<> csys = state_x.ClipCoordsys(0, 0);
    return csys.TransformPointLocalToParent(loc_point);
}

ChVector<> ChAparticle::GetContactPointSpeed(const ChVector<>& loc_point,
                                             const ChState& state_x,
                                             const ChStateDelta& state_w) {
    ChCoordsys<> csys = state_x.ClipCoordsys(0, 0);
    ChVector<> abs_vel = state_w.ClipVector(0, 0);
    ChVector<> loc_omg = state_w.ClipVector(3, 0);
    ChVector<> abs_omg = csys.TransformDirectionLocalToParent(loc_omg);

    return abs_vel + Vcross(abs_omg, loc_point);
}

ChVector<> ChAparticle::GetContactPointSpeed(const ChVector<>& abs_point) {
    ChVector<> m_p1_loc = this->TransformPointParentToLocal(abs_point);
    return this->PointSpeedLocalToParent(m_p1_loc);
}

void ChAparticle::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, ChVectorDynamic<>& R) {
    ChVector<> m_p1_loc = this->TransformPointParentToLocal(abs_point);
    ChVector<> force1_loc = this->TransformDirectionParentToLocal(F);
    ChVector<> torque1_loc = Vcross(m_p1_loc, force1_loc);
    R.PasteSumVector(F, Variables().GetOffset() + 0, 0);            //***TODO*** implement NodeGetOffset_w()
    R.PasteSumVector(torque1_loc, Variables().GetOffset() + 3, 0);  //***TODO*** implement NodeGetOffset_w()
}

void ChAparticle::ContactForceLoadQ(const ChVector<>& F,
                                    const ChVector<>& point,
                                    const ChState& state_x,
                                    ChVectorDynamic<>& Q,
                                    int offset) {
    ChCoordsys<> csys = state_x.ClipCoordsys(0, 0);
    ChVector<> point_loc = csys.TransformPointParentToLocal(point);
    ChVector<> force_loc = csys.TransformDirectionParentToLocal(F);
    ChVector<> torque_loc = Vcross(point_loc, force_loc);
    Q.PasteVector(F, offset + 0, 0);
    Q.PasteVector(torque_loc, offset + 3, 0);
}

void ChAparticle::ComputeJacobianForContactPart(
    const ChVector<>& abs_point,
    ChMatrix33<>& contact_plane,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
    bool second) {
    ChVector<> m_p1_loc = this->TransformPointParentToLocal(abs_point);
    ChMatrix33<> Jx1, Jr1;
    ChMatrix33<> Ps1, Jtemp;
    Ps1.Set_X_matrix(m_p1_loc);

    Jx1.CopyFromMatrixT(contact_plane);
    if (!second)
        Jx1.MatrNeg();

    Jtemp.MatrMultiply(this->GetA(), Ps1);
    Jr1.MatrTMultiply(contact_plane, Jtemp);
    if (second)
        Jr1.MatrNeg();

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(Jr1, 0, 0, 1, 3, 0, 3);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(Jr1, 1, 0, 1, 3, 0, 3);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(Jr1, 2, 0, 1, 3, 0, 3);
}

void ChAparticle::ComputeJacobianForRollingContactPart(
    const ChVector<>& abs_point,
    ChMatrix33<>& contact_plane,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
    bool second) {
    ChMatrix33<> Jx1, Jr1;

    Jr1.MatrTMultiply(contact_plane, this->GetA());
    if (!second)
        Jr1.MatrNeg();

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(Jr1, 0, 0, 1, 3, 0, 3);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(Jr1, 1, 0, 1, 3, 0, 3);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(Jr1, 2, 0, 1, 3, 0, 3);
}

ChPhysicsItem* ChAparticle::GetPhysicsItem() {
    return container;
}

void ChAparticle::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChAparticle>();

    // serialize parent class
    ChParticleBase::ArchiveOUT(marchive);

    // serialize all member data:
    // marchive << CHNVP(container);
    marchive << CHNVP(collision_model);
    marchive << CHNVP(UserForce);
    marchive << CHNVP(UserTorque);
}

/// Method to allow de serialization of transient data from archives.
void ChAparticle::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChAparticle>();

    // deserialize parent class:
    ChParticleBase::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(collision_model);
    marchive >> CHNVP(UserForce);
    marchive >> CHNVP(UserTorque);
}

// -----------------------------------------------------------------------------
// CLASS FOR PARTICLE CLUSTER
// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChParticlesClones)

ChParticlesClones::ChParticlesClones()
    : do_collide(false),
      do_limit_speed(false),
      do_sleep(false),
      max_speed(0.5f),
      max_wvel((float)CH_C_2PI),
      sleep_time(0.6f),
      sleep_starttime(0),
      sleep_minspeed(0.1f),
      sleep_minwvel(0.04f) {
    SetMass(1.0);
    SetInertiaXX(ChVector<double>(1.0, 1.0, 1.0));
    SetInertiaXY(ChVector<double>(0, 0, 0));

    particle_collision_model = new ChModelBullet();
    particle_collision_model->SetContactable(0);

    particles.clear();
    // ResizeNparticles(num_particles); // caused memory corruption.. why?

    // default non-smooth contact material
    matsurface = std::make_shared<ChMaterialSurfaceNSC>();
}

ChParticlesClones::ChParticlesClones(const ChParticlesClones& other) : ChIndexedParticles(other) {
    do_collide = other.do_collide;
    do_sleep = other.do_sleep;
    do_limit_speed = other.do_limit_speed;

    SetMass(other.GetMass());
    SetInertiaXX(other.GetInertiaXX());
    SetInertiaXY(other.GetInertiaXY());

    particle_collision_model->ClearModel();

    matsurface = std::shared_ptr<ChMaterialSurface>(other.matsurface->Clone());  // deep copy

    ResizeNparticles((int)other.GetNparticles());

    max_speed = other.max_speed;
    max_wvel = other.max_wvel;

    sleep_time = other.sleep_time;
    sleep_starttime = other.sleep_starttime;
    sleep_minspeed = other.sleep_minspeed;
    sleep_minwvel = other.sleep_minwvel;
}

ChParticlesClones::~ChParticlesClones() {
    ResizeNparticles(0);

    if (particle_collision_model)
        delete particle_collision_model;
    particle_collision_model = 0;
}

void ChParticlesClones::ResizeNparticles(int newsize) {
    bool oldcoll = GetCollide();
    SetCollide(false);  // this will remove old particle coll.models from coll.engine, if previously added

    for (unsigned int j = 0; j < particles.size(); j++) {
        delete (particles[j]);
        particles[j] = 0;
    }

    particles.resize(newsize);

    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j] = new ChAparticle;

        particles[j]->SetContainer(this);

        particles[j]->variables.SetSharedMass(&particle_mass);
        particles[j]->variables.SetUserData((void*)this);  // UserData unuseful in future parallel solver?

        particles[j]->collision_model->SetContactable(particles[j]);
        // articles[j]->collision_model->ClearModel();
        particles[j]->collision_model->AddCopyOfAnotherModel(particle_collision_model);
        particles[j]->collision_model->BuildModel();
    }

    SetCollide(oldcoll);  // this will also add particle coll.models to coll.engine, if already in a ChSystem
}

void ChParticlesClones::AddParticle(ChCoordsys<double> initial_state) {
    ChAparticle* newp = new ChAparticle;
    newp->SetCoord(initial_state);

    newp->SetContainer(this);

    particles.push_back(newp);

    newp->variables.SetSharedMass(&particle_mass);
    newp->variables.SetUserData((void*)this);  // UserData unuseful in future parallel solver?

    newp->collision_model->SetContactable(newp);
    // newp->collision_model->ClearModel(); // wasn't already added to system, no need to remove
    newp->collision_model->AddCopyOfAnotherModel(particle_collision_model);
    newp->collision_model->BuildModel();  // will also add to system, if collision is on.
}

// STATE BOOKKEEPING FUNCTIONS

void ChParticlesClones::IntStateGather(const unsigned int off_x,  // offset in x state vector
                                       ChState& x,                // state vector, position part
                                       const unsigned int off_v,  // offset in v state vector
                                       ChStateDelta& v,           // state vector, speed part
                                       double& T                  // time
                                       ) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        x.PasteCoordsys(particles[j]->coord, off_x + 7 * j, 0);
        v.PasteVector(particles[j]->coord_dt.pos, off_v + 6 * j, 0);
        v.PasteVector(particles[j]->GetWvel_loc(), off_v + 6 * j + 3, 0);
        T = GetChTime();
    }
}

void ChParticlesClones::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                                        const ChState& x,          // state vector, position part
                                        const unsigned int off_v,  // offset in v state vector
                                        const ChStateDelta& v,     // state vector, speed part
                                        const double T             // time
                                        ) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetCoord(x.ClipCoordsys(off_x + 7 * j, 0));
        particles[j]->SetPos_dt(v.ClipVector(off_v + 6 * j, 0));
        particles[j]->SetWvel_loc(v.ClipVector(off_v + 6 * j + 3, 0));
    }
    SetChTime(T);
    Update();
}

void ChParticlesClones::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        a.PasteVector(particles[j]->coord_dtdt.pos, off_a + 6 * j, 0);
        a.PasteVector(particles[j]->GetWacc_loc(), off_a + 6 * j + 3, 0);
    }
}

void ChParticlesClones::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetPos_dtdt(a.ClipVector(off_a + 6 * j, 0));
        particles[j]->SetWacc_loc(a.ClipVector(off_a + 6 * j + 3, 0));
    }
}

void ChParticlesClones::IntStateIncrement(const unsigned int off_x,  // offset in x state vector
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

        // ADVANCE ROTATION: rot' = delta*rot  (use quaternion for delta rotation)
        ChQuaternion<> mdeltarot;
        ChQuaternion<> moldrot = x.ClipQuaternion(off_x + 7 * j + 3, 0);
        ChVector<> newwel_abs = particles[j]->Amatrix * Dv.ClipVector(off_v + 6 * j + 3, 0);
        double mangle = newwel_abs.Length();
        newwel_abs.Normalize();
        mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
        ChQuaternion<> mnewrot = mdeltarot * moldrot;  // quaternion product
        x_new.PasteQuaternion(mnewrot, off_x + 7 * j + 3, 0);
    }
}

void ChParticlesClones::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                          ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                          const double c           // a scaling factor
                                          ) {
    ChVector<> Gforce;
    if (GetSystem())
        Gforce = GetSystem()->Get_G_acc() * particle_mass.GetBodyMass();

    for (unsigned int j = 0; j < particles.size(); j++) {
        // particle gyroscopic force:
        ChVector<> Wvel = particles[j]->GetWvel_loc();
        ChVector<> gyro = Vcross(Wvel, (particle_mass.GetBodyInertia().Matr_x_Vect(Wvel)));

        // add applied forces and torques (and also the gyroscopic torque and gravity!) to 'fb' vector
        R.PasteSumVector((particles[j]->UserForce + Gforce) * c, off + 6 * j, 0);
        R.PasteSumVector((particles[j]->UserTorque - gyro) * c, off + 6 * j + 3, 0);
    }
}

void ChParticlesClones::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                           ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                           const ChVectorDynamic<>& w,  // the w vector
                                           const double c               // a scaling factor
                                           ) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        R(off + 6 * j + 0) += c * GetMass() * w(off + 6 * j + 0);
        R(off + 6 * j + 1) += c * GetMass() * w(off + 6 * j + 1);
        R(off + 6 * j + 2) += c * GetMass() * w(off + 6 * j + 2);
        ChVector<> Iw = particle_mass.GetBodyInertia() * w.ClipVector(off + 6 * j + 3, 0);
        Iw *= c;
        R.PasteSumVector(Iw, off + 6 * j + 3, 0);
    }
}

void ChParticlesClones::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                        const ChStateDelta& v,
                                        const ChVectorDynamic<>& R,
                                        const unsigned int off_L,  // offset in L, Qc
                                        const ChVectorDynamic<>& L,
                                        const ChVectorDynamic<>& Qc) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.Get_qb().PasteClippedMatrix(v, off_v + 6 * j, 0, 6, 1, 0, 0);
        particles[j]->variables.Get_fb().PasteClippedMatrix(R, off_v + 6 * j, 0, 6, 1, 0, 0);
    }
}

void ChParticlesClones::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                          ChStateDelta& v,
                                          const unsigned int off_L,  // offset in L
                                          ChVectorDynamic<>& L) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        v.PasteMatrix(particles[j]->variables.Get_qb(), off_v + 6 * j, 0);
    }
}

void ChParticlesClones::InjectVariables(ChSystemDescriptor& mdescriptor) {
    // variables.SetDisabled(!IsActive());
    for (unsigned int j = 0; j < particles.size(); j++) {
        mdescriptor.InsertVariables(&(particles[j]->variables));
    }
}

void ChParticlesClones::VariablesFbReset() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.Get_fb().FillElem(0.0);
    }
}

void ChParticlesClones::VariablesFbLoadForces(double factor) {
    ChVector<> Gforce;
    if (GetSystem())
        Gforce = GetSystem()->Get_G_acc() * particle_mass.GetBodyMass();

    for (unsigned int j = 0; j < particles.size(); j++) {
        // particle gyroscopic force:
        ChVector<> Wvel = particles[j]->GetWvel_loc();
        ChVector<> gyro = Vcross(Wvel, (particle_mass.GetBodyInertia().Matr_x_Vect(Wvel)));

        // add applied forces and torques (and also the gyroscopic torque and gravity!) to 'fb' vector
        particles[j]->variables.Get_fb().PasteSumVector((particles[j]->UserForce + Gforce) * factor, 0, 0);
        particles[j]->variables.Get_fb().PasteSumVector((particles[j]->UserTorque - gyro) * factor, 3, 0);
    }
}

void ChParticlesClones::VariablesQbLoadSpeed() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        // set current speed in 'qb', it can be used by the solver when working in incremental mode
        particles[j]->variables.Get_qb().PasteVector(particles[j]->GetCoord_dt().pos, 0, 0);
        particles[j]->variables.Get_qb().PasteVector(particles[j]->GetWvel_loc(), 3, 0);
    }
}

void ChParticlesClones::VariablesFbIncrementMq() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->variables.Compute_inc_Mb_v(particles[j]->variables.Get_fb(), particles[j]->variables.Get_qb());
    }
}

void ChParticlesClones::VariablesQbSetSpeed(double step) {
    for (unsigned int j = 0; j < particles.size(); j++) {
        ChCoordsys<> old_coord_dt = particles[j]->GetCoord_dt();

        // from 'qb' vector, sets body speed, and updates auxiliary data
        particles[j]->SetPos_dt(particles[j]->variables.Get_qb().ClipVector(0, 0));
        particles[j]->SetWvel_loc(particles[j]->variables.Get_qb().ClipVector(3, 0));

        // apply limits (if in speed clamping mode) to speeds.
        // ClampSpeed(); NO - do only per-particle, here.. (but.. really needed here?)

        // Compute accel. by BDF (approximate by differentiation);
        if (step) {
            particles[j]->SetPos_dtdt((particles[j]->GetCoord_dt().pos - old_coord_dt.pos) / step);
            particles[j]->SetRot_dtdt((particles[j]->GetCoord_dt().rot - old_coord_dt.rot) / step);
        }
    }
}

void ChParticlesClones::VariablesQbIncrementPosition(double dt_step) {
    // if (!IsActive())
    //	return;

    for (unsigned int j = 0; j < particles.size(); j++) {
        // Updates position with incremental action of speed contained in the
        // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

        ChVector<> newspeed = particles[j]->variables.Get_qb().ClipVector(0, 0);
        ChVector<> newwel = particles[j]->variables.Get_qb().ClipVector(3, 0);

        // ADVANCE POSITION: pos' = pos + dt * vel
        particles[j]->SetPos(particles[j]->GetPos() + newspeed * dt_step);

        // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
        ChQuaternion<> mdeltarot;
        ChQuaternion<> moldrot = particles[j]->GetRot();
        ChVector<> newwel_abs = particles[j]->GetA() * newwel;
        double mangle = newwel_abs.Length() * dt_step;
        newwel_abs.Normalize();
        mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
        ChQuaternion<> mnewrot = mdeltarot % moldrot;
        particles[j]->SetRot(mnewrot);
    }
}

void ChParticlesClones::SetNoSpeedNoAcceleration() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetPos_dt(VNULL);
        particles[j]->SetWvel_loc(VNULL);
        particles[j]->SetPos_dtdt(VNULL);
        particles[j]->SetRot_dtdt(QNULL);
    }
}

void ChParticlesClones::ClampSpeed() {
    if (GetLimitSpeed()) {
        for (unsigned int j = 0; j < particles.size(); j++) {
            double w = 2.0 * particles[j]->GetRot_dt().Length();
            if (w > max_wvel)
                particles[j]->SetRot_dt(particles[j]->GetRot_dt() * max_wvel / w);

            double v = particles[j]->GetPos_dt().Length();
            if (v > max_speed)
                particles[j]->SetPos_dt(particles[j]->GetPos_dt() * max_speed / v);
        }
    }
}

// The inertia tensor functions

void ChParticlesClones::SetInertia(const ChMatrix33<>& newXInertia) {
    particle_mass.SetBodyInertia(newXInertia);
}

void ChParticlesClones::SetInertiaXX(const ChVector<>& iner) {
    particle_mass.GetBodyInertia().SetElement(0, 0, iner.x());
    particle_mass.GetBodyInertia().SetElement(1, 1, iner.y());
    particle_mass.GetBodyInertia().SetElement(2, 2, iner.z());
    particle_mass.GetBodyInertia().FastInvert(particle_mass.GetBodyInvInertia());
}
void ChParticlesClones::SetInertiaXY(const ChVector<>& iner) {
    particle_mass.GetBodyInertia().SetElement(0, 1, iner.x());
    particle_mass.GetBodyInertia().SetElement(0, 2, iner.y());
    particle_mass.GetBodyInertia().SetElement(1, 2, iner.z());
    particle_mass.GetBodyInertia().SetElement(1, 0, iner.x());
    particle_mass.GetBodyInertia().SetElement(2, 0, iner.y());
    particle_mass.GetBodyInertia().SetElement(2, 1, iner.z());
    particle_mass.GetBodyInertia().FastInvert(particle_mass.GetBodyInvInertia());
}

ChVector<> ChParticlesClones::GetInertiaXX() const {
    ChVector<> iner;
    iner.x() = particle_mass.GetBodyInertia().GetElement(0, 0);
    iner.y() = particle_mass.GetBodyInertia().GetElement(1, 1);
    iner.z() = particle_mass.GetBodyInertia().GetElement(2, 2);
    return iner;
}

ChVector<> ChParticlesClones::GetInertiaXY() const {
    ChVector<> iner;
    iner.x() = particle_mass.GetBodyInertia().GetElement(0, 1);
    iner.y() = particle_mass.GetBodyInertia().GetElement(0, 2);
    iner.z() = particle_mass.GetBodyInertia().GetElement(1, 2);
    return iner;
}

void ChParticlesClones::Update(bool update_assets) {
    ChParticlesClones::Update(GetChTime(), update_assets);
}

void ChParticlesClones::Update(double mytime, bool update_assets) {
    ChTime = mytime;

    // TrySleeping();			// See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();  // Apply limits (if in speed clamping mode) to speeds.
}

// collision stuff
void ChParticlesClones::SetCollide(bool mcoll) {
    if (mcoll == do_collide)
        return;

    if (mcoll) {
        do_collide = true;
        if (GetSystem()) {
            for (unsigned int j = 0; j < particles.size(); j++) {
                GetSystem()->GetCollisionSystem()->Add(particles[j]->collision_model);
            }
        }
    } else {
        do_collide = false;
        if (GetSystem()) {
            for (unsigned int j = 0; j < particles.size(); j++) {
                GetSystem()->GetCollisionSystem()->Remove(particles[j]->collision_model);
            }
        }
    }
}

void ChParticlesClones::SyncCollisionModels() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->collision_model->SyncPosition();
    }
}

void ChParticlesClones::AddCollisionModelsToSystem() {
    assert(GetSystem());
    SyncCollisionModels();
    for (unsigned int j = 0; j < particles.size(); j++) {
        GetSystem()->GetCollisionSystem()->Add(particles[j]->collision_model);
    }
}

void ChParticlesClones::RemoveCollisionModelsFromSystem() {
    assert(GetSystem());
    for (unsigned int j = 0; j < particles.size(); j++) {
        GetSystem()->GetCollisionSystem()->Remove(particles[j]->collision_model);
    }
}

//

void ChParticlesClones::UpdateParticleCollisionModels() {
    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->collision_model->ClearModel();
        particles[j]->collision_model->AddCopyOfAnotherModel(particle_collision_model);
        particles[j]->collision_model->BuildModel();
    }
}

// FILE I/O

void ChParticlesClones::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChParticlesClones>();

    // serialize parent class
    ChIndexedParticles::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(particles);
    // marchive << CHNVP(particle_mass); //***TODO***
    marchive << CHNVP(particle_collision_model);
    marchive << CHNVP(matsurface);
    marchive << CHNVP(do_collide);
    marchive << CHNVP(do_limit_speed);
    marchive << CHNVP(do_sleep);
    marchive << CHNVP(max_speed);
    marchive << CHNVP(max_wvel);
    marchive << CHNVP(sleep_time);
    marchive << CHNVP(sleep_minspeed);
    marchive << CHNVP(sleep_minwvel);
    marchive << CHNVP(sleep_starttime);
}

void ChParticlesClones::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChParticlesClones>();

    // deserialize parent class:
    ChIndexedParticles::ArchiveIN(marchive);

    // deserialize all member data:

    RemoveCollisionModelsFromSystem();

    marchive >> CHNVP(particles);
    // marchive >> CHNVP(particle_mass); //***TODO***
    marchive >> CHNVP(particle_collision_model);
    marchive >> CHNVP(matsurface);
    marchive >> CHNVP(do_collide);
    marchive >> CHNVP(do_limit_speed);
    marchive >> CHNVP(do_sleep);
    marchive >> CHNVP(max_speed);
    marchive >> CHNVP(max_wvel);
    marchive >> CHNVP(sleep_time);
    marchive >> CHNVP(sleep_minspeed);
    marchive >> CHNVP(sleep_minwvel);
    marchive >> CHNVP(sleep_starttime);

    for (unsigned int j = 0; j < particles.size(); j++) {
        particles[j]->SetContainer(this);
    }
    AddCollisionModelsToSystem();
}

}  // end namespace chrono