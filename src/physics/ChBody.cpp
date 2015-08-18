//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <stdlib.h>
#include <algorithm>

#include "core/ChTransform.h"
#include "physics/ChBody.h"
#include "physics/ChGlobal.h"
#include "physics/ChMarker.h"
#include "physics/ChForce.h"
#include "physics/ChSystem.h"

#include "collision/ChCModelBullet.h"
#include "core/ChLinearAlgebra.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBody> a_registration_ChBody;

// Hierarchy-handling shortcuts

#define MARKpointer (*imarker)
#define HIER_MARKER_INIT std::vector<ChMarker*>::iterator imarker = marklist.begin();
#define HIER_MARKER_NOSTOP (imarker != marklist.end())
#define HIER_MARKER_NEXT imarker++;

#define FORCEpointer (*iforce)
#define HIER_FORCE_INIT std::vector<ChForce*>::iterator iforce = forcelist.begin();
#define HIER_FORCE_NOSTOP (iforce != forcelist.end())
#define HIER_FORCE_NEXT iforce++;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES

ChBody::ChBody(ChMaterialSurfaceBase::ContactMethod contact_method) {
    marklist.clear();
    forcelist.clear();

    BFlagsSetAllOFF();  // no flags

    Xforce = VNULL;
    Xtorque = VNULL;

    Force_acc = VNULL;
    Torque_acc = VNULL;
    Scr_force = VNULL;
    Scr_torque = VNULL;

    collision_model = InstanceCollisionModel();

    switch (contact_method) {
        case ChMaterialSurfaceBase::DVI:
            matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
            break;
        case ChMaterialSurfaceBase::DEM:
            matsurface = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
            break;
    }

    density = 1000.0f;

    last_coll_pos = CSYSNORM;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID

    max_speed = 0.5f;
    max_wvel = 2.0f * float(CH_C_PI);

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;
    SetUseSleeping(true);

    variables.SetUserData((void*)this);

    body_id = 0;
}

ChBody::ChBody(ChCollisionModel* new_collision_model, ChMaterialSurfaceBase::ContactMethod contact_method) {
    marklist.clear();
    forcelist.clear();

    BFlagsSetAllOFF();  // no flags

    Xforce = VNULL;
    Xtorque = VNULL;

    Force_acc = VNULL;
    Torque_acc = VNULL;
    Scr_force = VNULL;
    Scr_torque = VNULL;

    collision_model = new_collision_model;
    collision_model->SetContactable(this);

    switch (contact_method) {
        case ChMaterialSurfaceBase::DVI:
            matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
            break;
        case ChMaterialSurfaceBase::DEM:
            matsurface = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
            break;
    }

    density = 1000.0f;

    last_coll_pos = CSYSNORM;

    SetIdentifier(GetUniqueIntID());  // mark with unique ID

    max_speed = 0.5f;
    max_wvel = 2.0f * float(CH_C_PI);

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;
    SetUseSleeping(true);

    variables.SetUserData((void*)this);

    body_id = 0;
}

ChBody::~ChBody() {
    RemoveAllForces();
    RemoveAllMarkers();

    if (collision_model)
        delete collision_model;
}

void ChBody::Copy(ChBody* source) {
    // copy the parent class data...
    ChPhysicsItem::Copy(source);

    // copy the parent class data...
    ChFrameMoving<double>::operator=(*source);

    bflag = source->bflag;

    variables = source->variables;
    variables.SetUserData((void*)this);

    gyro = source->Get_gyro();

    RemoveAllForces();   // also copy-duplicate the forces? Let the user handle this..
    RemoveAllMarkers();  // also copy-duplicate the markers? Let the user handle this..

    ChTime = source->ChTime;

    collision_model->ClearModel();  // also copy-duplicate the collision model? Let the user handle this..

    this->matsurface = source->matsurface;  // also copy-duplicate the material? Let the user handle this..

    density = source->density;

    Scr_force = source->Scr_force;
    Scr_torque = source->Scr_torque;

    last_coll_pos = source->last_coll_pos;

    max_speed = source->max_speed;
    max_wvel = source->max_wvel;

    sleep_time = source->sleep_time;
    sleep_starttime = source->sleep_starttime;
    sleep_minspeed = source->sleep_minspeed;
    sleep_minwvel = source->sleep_minwvel;
}

ChCollisionModel* ChBody::InstanceCollisionModel() {
    ChCollisionModel* collision_model_t = (ChModelBullet*)new ChModelBullet();
    collision_model_t->SetContactable(this);
    return collision_model_t;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChBody::IntStateGather(const unsigned int off_x,  ///< offset in x state vector
                            ChState& x,                ///< state vector, position part
                            const unsigned int off_v,  ///< offset in v state vector
                            ChStateDelta& v,           ///< state vector, speed part
                            double& T)                 ///< time
{
    x.PasteCoordsys(this->coord, off_x, 0);
    v.PasteVector(this->coord_dt.pos, off_v, 0);
    v.PasteVector(this->GetWvel_loc(), off_v + 3, 0);
    T = this->GetChTime();
}

void ChBody::IntStateScatter(const unsigned int off_x,  ///< offset in x state vector
                             const ChState& x,          ///< state vector, position part
                             const unsigned int off_v,  ///< offset in v state vector
                             const ChStateDelta& v,     ///< state vector, speed part
                             const double T)            ///< time
{
    this->SetCoord(x.ClipCoordsys(off_x, 0));
    this->SetPos_dt(v.ClipVector(off_v, 0));
    this->SetWvel_loc(v.ClipVector(off_v + 3, 0));
    this->SetChTime(T);
    this->Update();
}

void ChBody::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.PasteVector(this->coord_dtdt.pos, off_a, 0);
    a.PasteVector(this->GetWacc_loc(), off_a + 3, 0);
}

void ChBody::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    this->SetPos_dtdt(a.ClipVector(off_a, 0));
    this->SetWacc_loc(a.ClipVector(off_a + 3, 0));
}

void ChBody::IntStateIncrement(const unsigned int off_x,  ///< offset in x state vector
                               ChState& x_new,            ///< state vector, position part, incremented result
                               const ChState& x,          ///< state vector, initial position part
                               const unsigned int off_v,  ///< offset in v state vector
                               const ChStateDelta& Dv)    ///< state vector, increment
{
    // ADVANCE POSITION:
    x_new(off_x) = x(off_x) + Dv(off_v);
    x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
    x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);

    // ADVANCE ROTATION: rot' = delta*rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot = x.ClipQuaternion(off_x + 3, 0);
    ChVector<> newwel_abs = Amatrix * Dv.ClipVector(off_v + 3, 0);
    double mangle = newwel_abs.Length();
    newwel_abs.Normalize();
    mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot * moldrot;  // quaternion product
    x_new.PasteQuaternion(mnewrot, off_x + 3, 0);
}

void ChBody::IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                               ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                               const double c           ///< a scaling factor
                               ) {
    // add applied forces to 'fb' vector
    R.PasteSumVector(Xforce * c, off, 0);

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (this->GetNoGyroTorque())
        R.PasteSumVector((Xtorque)*c, off + 3, 0);
    else
        R.PasteSumVector((Xtorque - gyro) * c, off + 3, 0);
}

void ChBody::IntLoadResidual_Mv(const unsigned int off,      ///< offset in R residual
                                ChVectorDynamic<>& R,        ///< result: the R residual, R += c*M*v
                                const ChVectorDynamic<>& w,  ///< the w vector
                                const double c               ///< a scaling factor
                                ) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    ChVector<> Iw = GetInertia() * w.ClipVector(off + 3, 0);
    Iw *= c;
    R.PasteSumVector(Iw, off + 3, 0);
}

void ChBody::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                      const ChStateDelta& v,
                      const ChVectorDynamic<>& R,
                      const unsigned int off_L,  ///< offset in L, Qc
                      const ChVectorDynamic<>& L,
                      const ChVectorDynamic<>& Qc) {
    this->variables.Get_qb().PasteClippedMatrix(&v, off_v, 0, 6, 1, 0, 0);  // for LCP warm starting only
    this->variables.Get_fb().PasteClippedMatrix(&R, off_v, 0, 6, 1, 0, 0);  // LCP known term
}

void ChBody::IntFromLCP(const unsigned int off_v,  ///< offset in v
                        ChStateDelta& v,
                        const unsigned int off_L,  ///< offset in L
                        ChVectorDynamic<>& L) {
    v.PasteMatrix(&this->variables.Get_qb(), off_v, 0);
}

////

void ChBody::InjectVariables(ChLcpSystemDescriptor& mdescriptor) {
    this->variables.SetDisabled(!this->IsActive());

    mdescriptor.InsertVariables(&this->variables);
}

void ChBody::VariablesFbReset() {
    this->variables.Get_fb().FillElem(0.0);
}

void ChBody::VariablesFbLoadForces(double factor) {
    // add applied forces to 'fb' vector
    this->variables.Get_fb().PasteSumVector(Xforce * factor, 0, 0);

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (this->GetNoGyroTorque())
        this->variables.Get_fb().PasteSumVector((Xtorque)*factor, 3, 0);
    else
        this->variables.Get_fb().PasteSumVector((Xtorque - gyro) * factor, 3, 0);
}

void ChBody::VariablesFbIncrementMq() {
    this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
}

void ChBody::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
    this->variables.Get_qb().PasteVector(GetCoord_dt().pos, 0, 0);
    this->variables.Get_qb().PasteVector(GetWvel_loc(), 3, 0);
}

void ChBody::VariablesQbSetSpeed(double step) {
    ChCoordsys<> old_coord_dt = this->GetCoord_dt();

    // from 'qb' vector, sets body speed, and updates auxiliary data
    this->SetPos_dt(this->variables.Get_qb().ClipVector(0, 0));
    this->SetWvel_loc(this->variables.Get_qb().ClipVector(3, 0));

    // apply limits (if in speed clamping mode) to speeds.
    ClampSpeed();

    // compute auxiliary gyroscopic forces
    ComputeGyro();

    // Compute accel. by BDF (approximate by differentiation);
    if (step) {
        this->SetPos_dtdt((this->GetCoord_dt().pos - old_coord_dt.pos) / step);
        this->SetRot_dtdt((this->GetCoord_dt().rot - old_coord_dt.rot) / step);
    }
}

void ChBody::VariablesQbIncrementPosition(double dt_step) {
    if (!this->IsActive())
        return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

    ChVector<> newspeed = variables.Get_qb().ClipVector(0, 0);
    ChVector<> newwel = variables.Get_qb().ClipVector(3, 0);

    // ADVANCE POSITION: pos' = pos + dt * vel
    this->SetPos(this->GetPos() + newspeed * dt_step);

    // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot = this->GetRot();
    ChVector<> newwel_abs = Amatrix * newwel;
    double mangle = newwel_abs.Length() * dt_step;
    newwel_abs.Normalize();
    mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot % moldrot;
    this->SetRot(mnewrot);
}

void ChBody::SetNoSpeedNoAcceleration() {
    this->SetPos_dt(VNULL);
    this->SetWvel_loc(VNULL);
    this->SetPos_dtdt(VNULL);
    this->SetRot_dtdt(QNULL);
}

////
void ChBody::ClampSpeed() {
    if (this->GetLimitSpeed()) {
        double w = 2.0 * this->coord_dt.rot.Length();
        if (w > max_wvel)
            coord_dt.rot *= max_wvel / w;

        double v = this->coord_dt.pos.Length();
        if (v > max_speed)
            coord_dt.pos *= max_speed / v;
    }
}

//// Utilities for coordinate transformations
///
ChVector<> ChBody::Point_World2Body(const ChVector<>& mpoint) {
    return ChFrame<double>::TransformParentToLocal(mpoint);
}

ChVector<> ChBody::Point_Body2World(const ChVector<>& mpoint) {
    return ChFrame<double>::TransformLocalToParent(mpoint);
}

ChVector<> ChBody::Dir_World2Body(const ChVector<>& mpoint) {
    return Amatrix.MatrT_x_Vect(mpoint);
}

ChVector<> ChBody::Dir_Body2World(const ChVector<>& mpoint) {
    return Amatrix.Matr_x_Vect(mpoint);
}

ChVector<> ChBody::RelPoint_AbsSpeed(const ChVector<>& mrelpoint) {
    return PointSpeedLocalToParent(mrelpoint);
}

ChVector<> ChBody::RelPoint_AbsAcc(const ChVector<>& mrelpoint) {
    return PointAccelerationLocalToParent(mrelpoint);
}

////
// The inertia tensor functions

void ChBody::SetInertia(const ChMatrix33<>& newXInertia) {
    variables.SetBodyInertia(newXInertia);
}

void ChBody::SetInertiaXX(const ChVector<>& iner) {
    variables.GetBodyInertia().SetElement(0, 0, iner.x);
    variables.GetBodyInertia().SetElement(1, 1, iner.y);
    variables.GetBodyInertia().SetElement(2, 2, iner.z);
    variables.GetBodyInertia().FastInvert(&variables.GetBodyInvInertia());
}
void ChBody::SetInertiaXY(const ChVector<>& iner) {
    variables.GetBodyInertia().SetElement(0, 1, iner.x);
    variables.GetBodyInertia().SetElement(0, 2, iner.y);
    variables.GetBodyInertia().SetElement(1, 2, iner.z);
    variables.GetBodyInertia().SetElement(1, 0, iner.x);
    variables.GetBodyInertia().SetElement(2, 0, iner.y);
    variables.GetBodyInertia().SetElement(2, 1, iner.z);
    variables.GetBodyInertia().FastInvert(&variables.GetBodyInvInertia());
}

ChVector<> ChBody::GetInertiaXX() {
    ChVector<> iner;
    iner.x = variables.GetBodyInertia().GetElement(0, 0);
    iner.y = variables.GetBodyInertia().GetElement(1, 1);
    iner.z = variables.GetBodyInertia().GetElement(2, 2);
    return iner;
}

ChVector<> ChBody::GetInertiaXY() {
    ChVector<> iner;
    iner.x = variables.GetBodyInertia().GetElement(0, 1);
    iner.y = variables.GetBodyInertia().GetElement(0, 2);
    iner.z = variables.GetBodyInertia().GetElement(1, 2);
    return iner;
}

void ChBody::ComputeQInertia(ChMatrixNM<double, 4, 4>* mQInertia) {
    ChMatrixNM<double, 3, 4> res;
    ChMatrixNM<double, 3, 4> Gl;
    ChMatrixNM<double, 4, 3> GlT;

    SetMatrix_Gl(Gl, coord.rot);
    GlT.CopyFromMatrixT(Gl);

    res.MatrMultiply(this->GetInertia(), Gl);
    mQInertia->MatrMultiply(GlT, res);  // [Iq]=[G'][Ix][G]
}

//////

void ChBody::Add_as_lagrangian_force(const ChVector<>& force,
                                     const ChVector<>& appl_point,
                                     int local,
                                     ChMatrixNM<double, 7, 1>* mQf) {
    ChVector<> mabsforce;
    ChVector<> mabstorque;
    To_abs_forcetorque(force, appl_point, local, mabsforce, mabstorque);
    mQf->PasteSumVector(mabsforce, 0, 0);
    mQf->PasteSumQuaternion(ChFrame<>::GlT_x_Vect(coord.rot, Dir_World2Body(mabstorque)), 3, 0);
}

void ChBody::Add_as_lagrangian_torque(const ChVector<>& torque, int local, ChMatrixNM<double, 7, 1>* mQf) {
    ChVector<> mabstorque;
    To_abs_torque(torque, local, mabstorque);
    mQf->PasteSumQuaternion(ChFrame<>::GlT_x_Vect(coord.rot, Dir_World2Body(mabstorque)), 3, 0);
}

void ChBody::From_lagrangian_to_forcetorque(const ChMatrixNM<double, 7, 1>& mQf,
                                            ChVector<>& mforce,
                                            ChVector<>& mtorque) {
    mforce = mQf.ClipVector(0, 0);
    ChQuaternion<> tempq;
    tempq = mQf.ClipQuaternion(3, 0);
    mtorque = ChFrame<>::Gl_x_Quat(coord.rot, tempq);
    mtorque *= 0.25;
}

void ChBody::From_forcetorque_to_lagrangian(const ChVector<>& mforce,
                                            const ChVector<>& mtorque,
                                            ChMatrixNM<double, 7, 1>& mQf) {
    mQf.PasteVector(mforce, 0, 0);
    ChQuaternion<> tempq;
    tempq = ChFrame<>::GlT_x_Vect(coord.rot, mtorque);
    mQf.PasteQuaternion(tempq, 3, 0);
}

//////

void ChBody::Accumulate_force(const ChVector<>& force, const ChVector<>& appl_point, int local) {
    ChVector<> mabsforce;
    ChVector<> mabstorque;
    To_abs_forcetorque(force, appl_point, local, mabsforce, mabstorque);

    Force_acc += mabsforce;
    Torque_acc += mabstorque;
}

void ChBody::Accumulate_torque(const ChVector<>& torque, int local) {
    ChVector<> mabstorque;
    To_abs_torque(torque, local, mabstorque);
    Torque_acc += mabstorque;
}

void ChBody::Accumulate_script_force(const ChVector<>& force, const ChVector<>& appl_point, int local) {
    ChVector<> mabsforce;
    ChVector<> mabstorque;
    To_abs_forcetorque(force, appl_point, local, mabsforce, mabstorque);

    Scr_force += mabsforce;
    Scr_torque += mabstorque;
}

void ChBody::Accumulate_script_torque(const ChVector<>& torque, int local) {
    ChVector<> mabstorque;
    To_abs_torque(torque, local, mabstorque);

    Scr_torque += mabstorque;
}

////////

void ChBody::ComputeGyro() {
    ChVector<> Wvel = this->GetWvel_loc();
    gyro = Vcross(Wvel, (variables.GetBodyInertia().Matr_x_Vect(Wvel)));
}

bool ChBody::TrySleeping() {
    if (this->GetUseSleeping()) {
        if (this->GetSleeping())
            return true;

        if ((this->coord_dt.pos.LengthInf() < this->sleep_minspeed) &&
            (2.0 * this->coord_dt.rot.LengthInf() < this->sleep_minwvel)) {
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

void ChBody::AddMarker(ChSharedPtr<ChMarker> amarker) {
    // don't allow double insertion of same object
    assert(std::find<std::vector<ChMarker*>::iterator>(marklist.begin(), marklist.end(), amarker.get_ptr()) ==
           marklist.end());

    amarker->SetBody(this);
    amarker->AddRef();
    marklist.push_back((amarker).get_ptr());
}

void ChBody::AddForce(ChSharedPtr<ChForce> aforce) {
    // don't allow double insertion of same object
    assert(std::find<std::vector<ChForce*>::iterator>(forcelist.begin(), forcelist.end(), aforce.get_ptr()) ==
           forcelist.end());

    aforce->SetBody(this);
    aforce->AddRef();
    forcelist.push_back((aforce).get_ptr());
}

void ChBody::RemoveForce(ChSharedPtr<ChForce> mforce) {
    // trying to remove objects not previously added?
    assert(std::find<std::vector<ChForce*>::iterator>(forcelist.begin(), forcelist.end(), mforce.get_ptr()) !=
           forcelist.end());

    // warning! linear time search
    forcelist.erase(std::find<std::vector<ChForce*>::iterator>(forcelist.begin(), forcelist.end(), mforce.get_ptr()));

    mforce->SetBody(0);
    mforce->RemoveRef();
}

void ChBody::RemoveMarker(ChSharedPtr<ChMarker> mmarker) {
    // trying to remove objects not previously added?
    assert(std::find<std::vector<ChMarker*>::iterator>(marklist.begin(), marklist.end(), mmarker.get_ptr()) !=
           marklist.end());

    // warning! linear time search
    marklist.erase(std::find<std::vector<ChMarker*>::iterator>(marklist.begin(), marklist.end(), mmarker.get_ptr()));

    mmarker->SetBody(0);
    mmarker->RemoveRef();
}

void ChBody::RemoveAllForces() {
    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP) {
        FORCEpointer->SetBody(0);
        FORCEpointer->RemoveRef();
        HIER_FORCE_NEXT
    }
    forcelist.clear();
}

void ChBody::RemoveAllMarkers() {
    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP) {
        MARKpointer->SetBody(0);
        MARKpointer->RemoveRef();
        HIER_MARKER_NEXT
    }

    marklist.clear();
}

ChSharedPtr<ChMarker> ChBody::SearchMarker(const char* m_name) {
    ChMarker* mmark =
        ChContainerSearchFromName<ChMarker, std::vector<ChMarker*>::iterator>(m_name, marklist.begin(), marklist.end());
    if (mmark) {
        mmark->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChMarker>(
            mmark));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChMarker>());  // not found? return a void shared ptr.
}
ChSharedPtr<ChForce> ChBody::SearchForce(const char* m_name) {
    ChForce* mforce =
        ChContainerSearchFromName<ChForce, std::vector<ChForce*>::iterator>(m_name, forcelist.begin(), forcelist.end());
    if (mforce) {
        mforce->AddRef();  // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChForce>(
            mforce));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChForce>());  // not found? return a void shared ptr.
}

// These are the members used to UPDATE
// the body coordinates during the animation
// Also the coordinates of forces and markers
// linked to the body will be updated.

void ChBody::UpdateMarkers(double mytime) {
    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP) {
        MARKpointer->Update(mytime);

        HIER_MARKER_NEXT
    }
}

void ChBody::UpdateForces(double mytime) {
    // COMPUTE LAGRANGIAN FORCES APPLIED TO BODY

    // 1a- force caused by accumulation of forces in body's accumulator Force_acc
    Xforce = Force_acc;

    // 1b- force caused by accumulation of torques in body's accumulator Force_acc
    if (Vnotnull(&Torque_acc)) {
        Xtorque = Dir_World2Body(Torque_acc);
    } else {
        Xtorque = VNULL;
    }

    // 2 - accumulation of other applied forces
    ChVector<> mforce;
    ChVector<> mtorque;
    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP) {
        // update positions, f=f(t,q)
        FORCEpointer->Update(mytime);

        FORCEpointer->GetBodyForceTorque(&mforce, &mtorque);
        Xforce += mforce;
        Xtorque += mtorque;

        HIER_FORCE_NEXT
    }

    // 3 - accumulation of script forces
    Xforce += Scr_force;

    if (Vnotnull(&Scr_torque)) {
        Xtorque += Dir_World2Body(Scr_torque);
    }

    if (GetSystem()) {
        Xforce += GetSystem()->Get_G_acc() * this->GetMass();
    }
}

void ChBody::UpdateTime(double mytime) {
    ChTime = mytime;
}

void ChBody::UpdateState(const ChCoordsys<>& mypos, const ChCoordsys<>& mypos_dt) {
    SetCoord(mypos);        // Set the state coordsys,
    SetCoord_dt(mypos_dt);  // automatically updating auxiliary variables

    // TrySleeping();          // See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();   // Apply limits (if in speed clamping mode) to speeds.
    ComputeGyro();  // Set the gyroscopic momentum.
}

void ChBody::UpdateStateTime(const ChCoordsys<>& mypos, const ChCoordsys<>& mypos_dt, double mytime) {
    UpdateTime(mytime);
    UpdateState(mypos, mypos_dt);
}

void ChBody::Update(const ChCoordsys<>& mypos, const ChCoordsys<>& mypos_dt, double mytime) {
    UpdateTime(mytime);

    ChCoordsys<> pos = mypos;
    pos.rot.Normalize();
    UpdateState(pos, mypos_dt);

    Update();
}

// UpdateALL updates the state and time
// of the object AND the dependant (linked)
// markers and forces.

void ChBody::Update(bool update_assets) {
    // TrySleeping();         // See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();   // Apply limits (if in speed clamping mode) to speeds.
    ComputeGyro();  // Set the gyroscopic momentum.

    // Also update the children "markers" and
    // "forces" depending on the body current state.
    UpdateMarkers(ChTime);

    UpdateForces(ChTime);

    // This will update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

// As before, but keeps the current state.
// Mostly used for world reference body.
void ChBody::Update(double mytime, bool update_assets) {
    // For the body:
    UpdateTime(mytime);

    Update(update_assets);
}

void ChBody::SetBodyFixed(bool mev) {
    variables.SetDisabled(mev);
    if (mev == BFlagGet(BF_FIXED))
        return;
    BFlagSet(BF_FIXED, mev);
    // RecomputeCollisionModel(); // because one may use different model types for static or dynamic coll.shapes
}

// collision stuff
void ChBody::SetCollide(bool mcoll) {
    if (mcoll == BFlagGet(BF_COLLIDE))
        return;

    if (mcoll) {
        SyncCollisionModels();
        BFlagSetON(BF_COLLIDE);
        if (GetSystem())
            GetSystem()->GetCollisionSystem()->Add(this->GetCollisionModel());
    } else {
        BFlagSetOFF(BF_COLLIDE);
        if (GetSystem())
            GetSystem()->GetCollisionSystem()->Remove(this->GetCollisionModel());
    }
}

void ChBody::ChangeCollisionModel(ChCollisionModel* new_collision_model) {
    if (collision_model) {
        if (system)
            system->GetCollisionSystem()->Remove(collision_model);

        delete collision_model;
    }

    collision_model = new_collision_model;
    collision_model->SetContactable(this);
}

// forward reference
int coll_model_from_r3d(ChCollisionModel* chmodel, ChBody* mbody, int lod, Vector* mt, ChMatrix33<>* mr);

int ChBody::RecomputeCollisionModel() {
    if (!GetCollide())
        return FALSE;  // do nothing unless collision enabled

    collision_model->ClearModel();  // ++++ start geometry definition

    // ... external geometry fetch shapes?

    collision_model->BuildModel();  // ++++ complete geometry definition

    return TRUE;
}

void ChBody::SyncCollisionModels() {
    this->GetCollisionModel()->SyncPosition();
}

void ChBody::AddCollisionModelsToSystem() {
    assert(this->GetSystem());
    SyncCollisionModels();
    this->GetSystem()->GetCollisionSystem()->Add(this->GetCollisionModel());
}

void ChBody::RemoveCollisionModelsFromSystem() {
    assert(this->GetSystem());
    this->GetSystem()->GetCollisionSystem()->Remove(this->GetCollisionModel());
}

void ChBody::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax) {
    if (this->GetCollisionModel())
        this->GetCollisionModel()->GetAABB(bbmin, bbmax);
    else
        ChPhysicsItem::GetTotalAABB(bbmin, bbmax);  // default: infinite aabb
}



ChVector<> ChBody::GetContactPointSpeed(const ChVector<>& abs_point) {
    ChVector<> m_p1_loc = this->Point_World2Body(abs_point);
    return this->PointSpeedLocalToParent(m_p1_loc);
}

void ChBody::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                             ChVectorDynamic<>& R) {
    ChVector<> m_p1_loc = this->Point_World2Body(abs_point);
    ChVector<> force1_loc = this->Dir_World2Body(F);
    ChVector<> torque1_loc = Vcross(m_p1_loc, force1_loc);
    R.PasteSumVector(F, this->GetOffset_w() + 0, 0);
    R.PasteSumVector(torque1_loc, this->GetOffset_w() + 3, 0);
}

void ChBody::ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
            type_constraint_tuple& jacobian_tuple_N, 
            type_constraint_tuple& jacobian_tuple_U, 
            type_constraint_tuple& jacobian_tuple_V, 
            bool second) {
    ChVector<> m_p1_loc = this->Point_World2Body(abs_point);
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

    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jx1, 2, 0, 1, 3, 0, 0);
    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jr1, 0, 0, 1, 3, 0, 3);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jr1, 1, 0, 1, 3, 0, 3);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jr1, 2, 0, 1, 3, 0, 3);
}

void ChBody::ComputeJacobianForRollingContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
            type_constraint_tuple& jacobian_tuple_N, 
            type_constraint_tuple& jacobian_tuple_U, 
            type_constraint_tuple& jacobian_tuple_V, 
            bool second) {
    ChMatrix33<> Jx1, Jr1;

    Jr1.MatrTMultiply(contact_plane, this->GetA());
    if (!second)
        Jr1.MatrNeg();
    
    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jx1, 0, 0, 1, 3, 0, 0);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jx1, 1, 0, 1, 3, 0, 0);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jx1, 2, 0, 1, 3, 0, 0);
    jacobian_tuple_N.Get_Cq()->PasteClippedMatrix(&Jr1, 0, 0, 1, 3, 0, 3);
    jacobian_tuple_U.Get_Cq()->PasteClippedMatrix(&Jr1, 1, 0, 1, 3, 0, 3);
    jacobian_tuple_V.Get_Cq()->PasteClippedMatrix(&Jr1, 2, 0, 1, 3, 0, 3);
}


//////// FILE I/O

void ChBody::ArchiveOUT(ChArchiveOut& marchive)
{
    // version number
    marchive.VersionWrite(1);

    // serialize parent class
    ChPhysicsItem::ArchiveOUT(marchive);
    // serialize parent class
    ChBodyFrame::ArchiveOUT(marchive);

    // serialize all member data:
    

    marchive << CHNVP(bflag);
    bool mflag; // more readable flag output in case of ASCII in/out
    mflag = BFlagGet(BF_FIXED);
    marchive << CHNVP(mflag,"is_fixed");
    mflag = BFlagGet(BF_COLLIDE);
    marchive << CHNVP(mflag,"collide");
    mflag = BFlagGet(BF_LIMITSPEED);
    marchive << CHNVP(mflag,"limit_speed");
    mflag = BFlagGet(BF_NOGYROTORQUE);
    marchive << CHNVP(mflag,"no_gyro_torque");
    mflag = BFlagGet(BF_USESLEEPING);
    marchive << CHNVP(mflag,"use_sleeping");
    mflag = BFlagGet(BF_SLEEPING);
    marchive << CHNVP(mflag,"is_sleeping");

    //marchive << CHNVP(marklist);
    // do rather a custom array save:
    marchive.out_array_pre("markers", marklist.size(), "ChMarker");
    for (int i = 0; i < marklist.size(); i++) {
        marklist[i]->AddRef(); // hack: since in list are not as shared pointers
        ChSharedPtr<ChMarker> a_marker(marklist[i]); // wrap into shared ptr
        marchive << CHNVP(a_marker,"");
        marchive.out_array_between(marklist.size(), "markers");
    }
    marchive.out_array_end(marklist.size(), "markers");

    //marchive << CHNVP(forcelist);
    // do rather a custom array save:
    marchive.out_array_pre("forces", forcelist.size(), "ChForce");
    for (int i = 0; i < forcelist.size(); i++) {
        forcelist[i]->AddRef(); // hack: since in list are not as shared pointers
        ChSharedPtr<ChForce> a_force(forcelist[i]); // wrap into shared ptr
        marchive << CHNVP(a_force,"");
        marchive.out_array_between(forcelist.size(), "forces");
    }
    marchive.out_array_end(forcelist.size(), "forces");

    marchive << CHNVP(body_id);
    marchive << CHNVP(collision_model);
    marchive << CHNVP(gyro);
    marchive << CHNVP(Xforce);
    marchive << CHNVP(Xtorque);
    //marchive << CHNVP(Force_acc); // not useful in serialization
    //marchive << CHNVP(Torque_acc);// not useful in serialization
    //marchive << CHNVP(Scr_force); // not useful in serialization
    //marchive << CHNVP(Scr_torque);// not useful in serialization
    marchive << CHNVP(matsurface);
    //marchive << CHNVP(last_coll_pos);// not useful in serialization
    marchive << CHNVP(density);
    marchive << CHNVP(variables);
    marchive << CHNVP(max_speed);
    marchive << CHNVP(max_wvel);
    marchive << CHNVP(sleep_time);
    marchive << CHNVP(sleep_minspeed);
    marchive << CHNVP(sleep_minwvel);
    marchive << CHNVP(sleep_starttime);
}

/// Method to allow de serialization of transient data from archives.
void ChBody::ArchiveIN(ChArchiveIn& marchive) 
{
    // version number
    int version = marchive.VersionRead();

    // deserialize parent class
    ChPhysicsItem::ArchiveIN(marchive);
    // deserialize parent class
    ChBodyFrame::ArchiveIN(marchive);

    // stream in all member data:

    marchive >> CHNVP(bflag);
    bool mflag; // more readable flag output in case of ASCII in/out
    marchive >> CHNVP(mflag,"is_fixed");
    BFlagSet(BF_FIXED,mflag);
    marchive >> CHNVP(mflag,"collide");
    BFlagSet(BF_COLLIDE,mflag);
    marchive >> CHNVP(mflag,"limit_speed");
    BFlagSet(BF_LIMITSPEED,mflag);
    marchive >> CHNVP(mflag,"no_gyro_torque");
    BFlagSet(BF_NOGYROTORQUE,mflag);
    marchive >> CHNVP(mflag,"use_sleeping");
    BFlagSet(BF_USESLEEPING,mflag);
    marchive >> CHNVP(mflag,"is_sleeping");
    BFlagSet(BF_SLEEPING,mflag);

    //marchive >> CHNVP(marklist);
    // do rather a custom array load:
    this->RemoveAllMarkers();
    size_t nummarkers;
    marchive.in_array_pre("markers", nummarkers);
    for (int i = 0; i < nummarkers; i++) {
        ChSharedPtr<ChMarker> a_marker;
        marchive >> CHNVP(a_marker,"");
        this->AddMarker(a_marker);
        marchive.in_array_between("markers");
    }
    marchive.in_array_end("markers");

    //marchive >> CHNVP(forcelist);
    // do rather a custom array load:
    this->RemoveAllForces();
    size_t numforces;
    marchive.in_array_pre("forces", numforces);
    for (int i = 0; i < numforces; i++) {
        ChSharedPtr<ChForce> a_force;
        marchive >> CHNVP(a_force,"");
        this->AddForce(a_force);
        marchive.in_array_between("forces");
    }
    marchive.in_array_end("forces");

    marchive >> CHNVP(body_id);
    marchive >> CHNVP(collision_model);
     collision_model->SetContactable(this);
    marchive >> CHNVP(gyro);
    marchive >> CHNVP(Xforce);
    marchive >> CHNVP(Xtorque);
    //marchive << CHNVP(Force_acc); // not useful in serialization
    //marchive << CHNVP(Torque_acc);// not useful in serialization
    //marchive << CHNVP(Scr_force); // not useful in serialization
    //marchive << CHNVP(Scr_torque);// not useful in serialization
    marchive >> CHNVP(matsurface);
    //marchive << CHNVP(last_coll_pos);// not useful in serialization
    marchive >> CHNVP(density);
    marchive >> CHNVP(variables);
    marchive >> CHNVP(max_speed);
    marchive >> CHNVP(max_wvel);
    marchive >> CHNVP(sleep_time);
    marchive >> CHNVP(sleep_minspeed);
    marchive >> CHNVP(sleep_minwvel);
    marchive >> CHNVP(sleep_starttime);
}

void ChBody::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(8);

    // serialize parent class too
    ChPhysicsItem::StreamOUT(mstream);

    // serialize parent class too
    ChFrameMoving<double>::StreamOUT(mstream);

    // stream out all member data
    ChVector<> vfoo = VNULL;
    double dfoo = 0;
    mstream << variables.GetBodyMass();
    vfoo = GetInertiaXX();
    mstream << vfoo;
    vfoo = GetInertiaXY();
    mstream << vfoo;

    mstream << dfoo;
    mstream << bflag;
    dfoo = (double)density;
    mstream << dfoo;

    mstream << max_speed;
    mstream << max_wvel;
    mstream << sleep_time;
    dfoo = (double)sleep_starttime;
    mstream << dfoo;
    mstream << sleep_minspeed;
    mstream << sleep_minwvel;

    //this->collision_model->StreamOUT(mstream);  // also  mstream << (*this->collision_model);

    this->matsurface->StreamOUT(mstream);
}

void ChBody::StreamIN(ChStreamInBinary& mstream) {
    // Class version number
    int version = mstream.VersionRead();

    // Deserialize parent class too
    if (version < 4)
        ChObj::StreamIN(mstream);
    else
        ChPhysicsItem::StreamIN(mstream);

    collision_model->ClearModel();

    if (version == 1) {
        // stream in all member data
        int mlock;
        double dfoo;
        mstream >> mlock;
        mstream >> coord;
        SetCoord(coord);
        mstream >> coord_dt;
        SetCoord_dt(coord_dt);
        mstream >> coord_dtdt;
        SetCoord_dtdt(coord_dtdt);
        mstream >> dfoo;
        SetMass(dfoo);
        ChVector<> vfoo;
        mstream >> vfoo;
        SetInertiaXX(vfoo);
        mstream >> vfoo;
        SetInertiaXY(vfoo);
        mstream >> dfoo;  // bdamper;
        mstream >> dfoo;
        GetMaterialSurface()->SetRestitution((float)dfoo);
        mstream >> dfoo;  // GetMaterialSurface()->SetRestitutionT((float)dfoo);
        mstream >> dfoo;
        GetMaterialSurface()->SetKfriction((float)dfoo);
        mstream >> dfoo;
        GetMaterialSurface()->SetSfriction((float)dfoo);
        mstream >> bflag;
        mstream >> dfoo;
        density = (float)dfoo;
        SetBodyFixed(mlock != 0);
    }
    if (version >= 2) {
        // deserialize parent class too
        ChFrameMoving<double>::StreamIN(mstream);

        // stream in all member data
        double dfoo;
        mstream >> dfoo;
        SetMass(dfoo);
        ChVector<> vfoo;
        mstream >> vfoo;
        SetInertiaXX(vfoo);
        mstream >> vfoo;
        SetInertiaXY(vfoo);
        mstream >> dfoo;  // bdamper;
        if (version < 7) {
            mstream >> dfoo;
            GetMaterialSurface()->SetRestitution((float)dfoo);
            mstream >> dfoo;  // GetMaterialSurface()->SetRestitutionT((float)dfoo);
            mstream >> dfoo;
            GetMaterialSurface()->SetKfriction((float)dfoo);
            mstream >> dfoo;
            GetMaterialSurface()->SetSfriction((float)dfoo);
        }
        mstream >> bflag;
        mstream >> dfoo;
        density = (float)dfoo;
        if (this->GetBodyFixed())
            SetBodyFixed(true);
        else
            SetBodyFixed(false);
    }
    if (version < 3)
        SetUseSleeping(true);
    if (version >= 3) {
        double dfoo;
        mstream >> max_speed;
        mstream >> max_wvel;
        mstream >> sleep_time;
        mstream >> dfoo;
        sleep_starttime = (float)dfoo;
        mstream >> sleep_minspeed;
        mstream >> sleep_minwvel;
    }
    if ((version >= 5) && (version < 7)) {
        double dfoo;
        mstream >> dfoo;
        GetMaterialSurface()->SetRollingFriction((float)dfoo);
        mstream >> dfoo;
        GetMaterialSurface()->SetSpinningFriction((float)dfoo);
    }
    if (version >= 6) {
        //this->collision_model->StreamIN(mstream);  // also   mstream >> (*collision_model);
        this->collision_model->BuildModel();       // because previously removed from ChSystem, if any.
    }
    if (version >= 7) {
        this->matsurface->StreamIN(mstream);
    }
}

void ChBody::StreamOUTstate(ChStreamOutBinary& mstream) {
    // Do not serialize parent classes and do not
    // implement versioning, because this must be efficient
    // and will be used just for domain decomposition.
    this->GetCoord().StreamOUT(mstream);
    this->GetCoord_dt().StreamOUT(mstream);
}

void ChBody::StreamINstate(ChStreamInBinary& mstream) {
    // Do not serialize parent classes and do not
    // implement versioning, because this must be efficient
    // and will be used just for domain decomposition.
    ChCoordsys<> mcoord;
    mstream >> mcoord;
    this->SetCoord(mcoord);
    mstream >> mcoord;
    this->SetCoord_dt(mcoord);

    this->Update();
    this->SyncCollisionModels();
}

#define CH_CHUNK_END 1234

int ChBody::StreamINall(ChStreamInBinary& m_file) {
    int mchunk = 0;
    ChMarker* newmarker = NULL;
    // ChForce*  newforce= NULL;

    // 0) reset body child lists
    RemoveAllMarkers();
    RemoveAllForces();

    // 1) read body class data...

    m_file >> *this;

    m_file >> mchunk;

    // 2) read child markers
    while (mchunk == CHCLASS_MARKER) {
        ChSharedPtr<ChMarker> newmarker(new ChMarker);
        this->AddMarker(newmarker);

        m_file >> *newmarker;

        newmarker->Impose_Abs_Coord(newmarker->GetAbsCoord());

        m_file >> mchunk;
    }

    // 3) read child links
    while (mchunk == CHCLASS_FORCE) {
        ChSharedPtr<ChForce> newforce(new ChForce);
        this->AddForce(newforce);

        m_file >> *newforce;

        m_file >> mchunk;
    }

    if (mchunk != CH_CHUNK_END)
        return 0;

    return 1;
}

int ChBody::StreamOUTall(ChStreamOutBinary& m_file) {
    // 1) read body class data...
    m_file << *this;

    // 2) read child bodies
    HIER_MARKER_INIT
    while
        HIER_MARKER_NOSTOP
    {
        m_file << (int)CHCLASS_MARKER;
        m_file << *MARKpointer;
        HIER_MARKER_NEXT
    }

    // 3) read child links
    HIER_FORCE_INIT
    while
        HIER_FORCE_NOSTOP
    {
        m_file << (int)CHCLASS_FORCE;
        m_file << *FORCEpointer;
        HIER_FORCE_NEXT
    }

    m_file << (int)CH_CHUNK_END;

    return 1;
}

void ChBody::StreamOUT(ChStreamOutAscii& mstream) {
    mstream << "BODY   " << GetName() << "\n";

    ChFrameMoving<double>::StreamOUT(mstream);

    //***TO DO***
}

int ChBody::StreamOUTall(ChStreamOutAscii& mstream)  // dump rigidbody and childrens (markers.forces)
{
    StreamOUT(mstream);  // 1) dump the body attrs

    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP)  // 2) dump the markers
    {
        MARKpointer->StreamOUT(mstream);
        HIER_MARKER_NEXT
    }

    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP)  // 3) dump the forces
    {
        FORCEpointer->StreamOUT(mstream);
        HIER_FORCE_NEXT
    }

    return 1;
}

}  // END_OF_NAMESPACE____

/////////////////////
