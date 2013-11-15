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

///////////////////////////////////////////////////
//
//   ChBody.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTrasform.h"
#include "physics/ChBody.h"
#include "physics/ChGlobal.h"
#include "physics/ChMarker.h"
#include "physics/ChForce.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletBody.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBody> a_registration_ChBody;


// Hierarchy-handling shortcuts
 
#define MARKpointer         (*imarker)
#define HIER_MARKER_INIT    std::vector<ChMarker*>::iterator imarker = marklist.begin();
#define HIER_MARKER_NOSTOP  (imarker != marklist.end())
#define HIER_MARKER_NEXT    imarker++;

#define FORCEpointer        (*iforce)
#define HIER_FORCE_INIT     std::vector<ChForce*>::iterator iforce = forcelist.begin();
#define HIER_FORCE_NOSTOP   (iforce != forcelist.end())
#define HIER_FORCE_NEXT     iforce++;   



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR SOLID BODIES


ChBody::ChBody ()
{
    marklist.clear();
    forcelist.clear();

    BFlagsSetAllOFF();      // no flags

    Xforce = VNULL;
    Xtorque = VNULL;

    Force_acc = VNULL;
    Torque_acc = VNULL;
    Scr_force = VNULL;
    Scr_torque = VNULL;
    cdim = VNULL;

    collision_model=InstanceCollisionModel();

    matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);

    density = 1000.0f;
    conductivity = 0.2f;

    last_coll_pos = CSYSNORM;

    SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

    max_speed = 0.5f;
    max_wvel  = 2.0f*float(CH_C_PI);

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;
    SetUseSleeping(true); 

    variables.SetUserData((void*)this);

    body_id = 0;
}

ChBody::ChBody (ChCollisionModel* new_collision_model)
{
    marklist.clear();
    forcelist.clear();

    BFlagsSetAllOFF();      // no flags

    Xforce = VNULL;
    Xtorque = VNULL;

    Force_acc = VNULL;
    Torque_acc = VNULL;
    Scr_force = VNULL;
    Scr_torque = VNULL;
    cdim = VNULL;

    collision_model=new_collision_model;
    collision_model->SetBody(this);

    matsurface = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);

    density = 1000.0f;
    conductivity = 0.2f;

    last_coll_pos = CSYSNORM;

    SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

    max_speed = 0.5f;
    max_wvel  = 2.0f*float(CH_C_PI);

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;
    SetUseSleeping(true);

    variables.SetUserData((void*)this);

    body_id = 0;
}

ChBody::~ChBody ()
{
    RemoveAllForces(); 
    RemoveAllMarkers();

    if (collision_model) delete collision_model;
}

void ChBody::Copy(ChBody* source)
{
        // copy the parent class data...
    ChPhysicsItem::Copy(source);

        // copy the parent class data...
    ChFrameMoving<double>::operator=(*source);


    bflag       = source->bflag;

    variables = source->variables;
    variables.SetUserData((void*)this);

    gyro    = source->Get_gyro();

    RemoveAllForces();  // also copy-duplicate the forces? Let the user handle this..
    RemoveAllMarkers();  // also copy-duplicate the markers? Let the user handle this..

    ChTime = source->ChTime;

    collision_model->ClearModel(); // also copy-duplicate the collision model? Let the user handle this..

    this->matsurface = source->matsurface;  // also copy-duplicate the material? Let the user handle this..

    density = source->density;
    conductivity = source->conductivity;

    Scr_force = source->Scr_force;
    Scr_torque = source->Scr_torque;
    cdim = source->cdim;

    last_coll_pos = source->last_coll_pos;

    max_speed = source->max_speed;
    max_wvel  = source->max_wvel;

    sleep_time = source->sleep_time;
    sleep_starttime = source->sleep_starttime;
    sleep_minspeed = source->sleep_minspeed;
    sleep_minwvel = source->sleep_minwvel;
}


ChCollisionModel* ChBody::InstanceCollisionModel(){
    ChCollisionModel* collision_model_t= (ChModelBulletBody*) new ChModelBulletBody();
    ((ChModelBulletBody*)collision_model_t)->SetBody(this);
    return collision_model_t;
}


//// 
void ChBody::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{   
    this->variables.SetDisabled(!this->IsActive());

    mdescriptor.InsertVariables(&this->variables);
}


void ChBody::VariablesFbReset()
{
    this->variables.Get_fb().FillElem(0.0);
}

void ChBody::VariablesFbLoadForces(double factor)
{
    // add applied forces to 'fb' vector
    this->variables.Get_fb().PasteSumVector( Xforce * factor ,0,0);

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (this->GetNoGyroTorque())
        this->variables.Get_fb().PasteSumVector((Xtorque)* factor ,3,0);
    else
        this->variables.Get_fb().PasteSumVector((Xtorque - gyro)* factor ,3,0);
}


void ChBody::VariablesFbIncrementMq()
{
	this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
}


void ChBody::VariablesQbLoadSpeed()
{
    // set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
    this->variables.Get_qb().PasteVector(GetCoord_dt().pos,0,0);
    this->variables.Get_qb().PasteVector(GetWvel_loc()    ,3,0);
}


void ChBody::VariablesQbSetSpeed(double step)
{
    ChCoordsys<> old_coord_dt = this->GetCoord_dt();

    // from 'qb' vector, sets body speed, and updates auxiliary data
    this->SetPos_dt(   this->variables.Get_qb().ClipVector(0,0) );
    this->SetWvel_loc( this->variables.Get_qb().ClipVector(3,0) );

    // apply limits (if in speed clamping mode) to speeds.
    ClampSpeed(); 

    // compute auxiliary gyroscopic forces
    ComputeGyro ();

    // Compute accel. by BDF (approximate by differentiation);
    if (step)
    {
        this->SetPos_dtdt( (this->GetCoord_dt().pos - old_coord_dt.pos)  / step);
        this->SetRot_dtdt( (this->GetCoord_dt().rot - old_coord_dt.rot)  / step);
    }
}

void ChBody::VariablesQbIncrementPosition(double dt_step)
{
    if (!this->IsActive()) 
        return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

    ChVector<> newspeed = variables.Get_qb().ClipVector(0,0);
    ChVector<> newwel   = variables.Get_qb().ClipVector(3,0);

    // ADVANCE POSITION: pos' = pos + dt * vel
    this->SetPos( this->GetPos() + newspeed * dt_step);

    // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot = this->GetRot();
    ChVector<> newwel_abs = Amatrix * newwel;
    double mangle = newwel_abs.Length() * dt_step;
    newwel_abs.Normalize();
    mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot % moldrot;
    this->SetRot( mnewrot );
}



void ChBody::SetNoSpeedNoAcceleration()
{
    this->SetPos_dt(VNULL);
    this->SetWvel_loc(VNULL);
    this->SetPos_dtdt(VNULL);
    this->SetRot_dtdt(QNULL);
}


////
void ChBody::ClampSpeed()
{
    if (this->GetLimitSpeed())
    {
        double w = 2.0*this->coord_dt.rot.Length();
        if (w > max_wvel)
            coord_dt.rot *= max_wvel/w;

        double v = this->coord_dt.pos.Length();
        if (v > max_speed)
            coord_dt.pos *= max_speed/v;
    }
}


//// Utilities for coordinate transformations
///
Vector ChBody::Point_World2Body (Vector* mpoint)
{
    return ChFrame<double>::TrasformParentToLocal(*mpoint);
}

Vector ChBody::Point_Body2World (Vector* mpoint)
{
    return ChFrame<double>::TrasformLocalToParent(*mpoint);
}

Vector ChBody::Dir_World2Body (Vector* mpoint)
{
    return Amatrix.MatrT_x_Vect (*mpoint);
}

Vector ChBody::Dir_Body2World (Vector* mpoint)
{
    return Amatrix.Matr_x_Vect (*mpoint);
}

Vector ChBody::RelPoint_AbsSpeed(Vector* mrelpoint)
{
    return PointSpeedLocalToParent(*mrelpoint);
}

Vector ChBody::RelPoint_AbsAcc(Vector* mrelpoint)
{
    return PointAccelerationLocalToParent(*mrelpoint);
}

////
// The inertia tensor functions

void ChBody::SetInertia (ChMatrix33<>* newXInertia)
{
    variables.SetBodyInertia(newXInertia);
}

void ChBody::SetInertiaXX (Vector iner)
{
    variables.GetBodyInertia().SetElement(0,0,iner.x);
    variables.GetBodyInertia().SetElement(1,1,iner.y);
    variables.GetBodyInertia().SetElement(2,2,iner.z);
    variables.GetBodyInertia().FastInvert(&variables.GetBodyInvInertia());
}
void ChBody::SetInertiaXY (Vector iner)
{
    variables.GetBodyInertia().SetElement(0,1,iner.x);
    variables.GetBodyInertia().SetElement(0,2,iner.y);
    variables.GetBodyInertia().SetElement(1,2,iner.z);
    variables.GetBodyInertia().SetElement(1,0,iner.x);
    variables.GetBodyInertia().SetElement(2,0,iner.y);
    variables.GetBodyInertia().SetElement(2,1,iner.z);
    variables.GetBodyInertia().FastInvert(&variables.GetBodyInvInertia());
}

Vector ChBody::GetInertiaXX()
{
    ChVector<> iner;
    iner.x= variables.GetBodyInertia().GetElement(0,0);
    iner.y= variables.GetBodyInertia().GetElement(1,1);
    iner.z= variables.GetBodyInertia().GetElement(2,2);
    return iner;
}

Vector ChBody::GetInertiaXY()
{
    ChVector<> iner;
    iner.x= variables.GetBodyInertia().GetElement(0,1);
    iner.y= variables.GetBodyInertia().GetElement(0,2);
    iner.z= variables.GetBodyInertia().GetElement(1,2);
    return iner;
}

void ChBody::ComputeQInertia (ChMatrixNM<double,4,4>* mQInertia)
{
    ChMatrixNM<double,3,4> res ;
    ChMatrixNM<double,3,4> Gl  ;
    ChMatrixNM<double,4,3> GlT ;

    SetMatrix_Gl(Gl, coord.rot);
    GlT.CopyFromMatrixT(Gl);

    res.MatrMultiply (*this->GetXInertia(), Gl);
    mQInertia->MatrMultiply (GlT, res); // [Iq]=[G'][Ix][G]
}

//////


void ChBody::To_abs_forcetorque  (Vector force, Vector appl_point, int local, Vector& resultforce, Vector& resulttorque)
{
    if (local)
    {
        // local space
        ChVector<> mforce_abs = Dir_Body2World(&force);
        resultforce = mforce_abs;
        resulttorque = Vcross (Dir_Body2World(&appl_point), mforce_abs) ;
    }
    else
    {
        // absolute space
        resultforce = force;
        resulttorque = Vcross (Vsub(appl_point, coord.pos), force) ;
    }
}
void ChBody::To_abs_torque (Vector torque, int local, Vector& resulttorque)
{
    if (local)
    {
        // local space
        resulttorque = Dir_Body2World(&torque);
    }
    else
    {
        // absolute space
        resulttorque = torque;
    }
}


void ChBody::Add_as_lagrangian_force(Vector force, Vector appl_point, int local, ChMatrixNM<double,7,1>* mQf)
{
    ChVector<> mabsforce;
    ChVector<> mabstorque;
    To_abs_forcetorque (force, appl_point, local, mabsforce, mabstorque);
    mQf->PasteSumVector(mabsforce,0,0);
    mQf->PasteSumQuaternion( ChFrame<>::GlT_x_Vect(coord.rot, Dir_World2Body(&mabstorque)) , 3,0);
}

void ChBody::Add_as_lagrangian_torque(Vector torque, int local, ChMatrixNM<double,7,1>* mQf)
{
    ChVector<> mabstorque;
    To_abs_torque (torque, local, mabstorque);
    mQf->PasteSumQuaternion( ChFrame<>::GlT_x_Vect(coord.rot, Dir_World2Body(&mabstorque)), 3,0);
}

void ChBody::From_lagrangian_to_forcetorque(ChMatrixNM<double,7,1>* mQf, Vector* mforce, Vector* mtorque)
{
    *mforce = mQf->ClipVector(0,0);
    ChQuaternion<> tempq;
    tempq = mQf->ClipQuaternion(3,0);
    *mtorque = ChFrame<>::Gl_x_Quat(coord.rot, tempq);
    *mtorque = Vmul (*mtorque, 0.25);
}

void ChBody::From_forcetorque_to_lagrangian(Vector* mforce, Vector* mtorque, ChMatrixNM<double,7,1>* mQf)
{
    mQf->PasteVector(*mforce, 0,0);
    ChQuaternion<> tempq;
    tempq = ChFrame<>::GlT_x_Vect(coord.rot, *mtorque);
    mQf->PasteQuaternion(tempq,3,0);
}

//////

void ChBody::Accumulate_force  (Vector force, Vector appl_point, int local)
{
    ChVector<> mabsforce;
    ChVector<> mabstorque;
    To_abs_forcetorque (force, appl_point, local, mabsforce, mabstorque);

    Force_acc = Vadd (Force_acc, mabsforce);
    Torque_acc = Vadd (Torque_acc, mabstorque);
}

void ChBody::Accumulate_torque (Vector torque, int local)
{
    ChVector<> mabstorque;
    To_abs_torque (torque, local, mabstorque);
    Torque_acc = Vadd (Torque_acc, mabstorque);
}

void ChBody::Accumulate_script_force  (Vector force, Vector appl_point, int local)
{
    ChVector<> mabsforce;
    ChVector<> mabstorque;
    To_abs_forcetorque (force, appl_point, local, mabsforce, mabstorque);

    Scr_force = Vadd (Scr_force, mabsforce);
    Scr_torque = Vadd (Scr_torque, mabstorque);
}
/*
void ChBody::SetCdim (Vector mcdim)
{
    ChVector<> cdim;

    cdim = mcdim;
}
*/
void ChBody::Accumulate_script_torque (Vector torque, int local)
{
    ChVector<> mabstorque;
    To_abs_torque (torque, local, mabstorque);
    Scr_torque = Vadd (Scr_torque, mabstorque);
}


////////


void ChBody::ComputeGyro ()
{
    ChVector<> Wvel = this->GetWvel_loc();
    gyro = Vcross ( Wvel, (variables.GetBodyInertia().Matr_x_Vect (Wvel))); 
}

bool ChBody::TrySleeping()
{
    if (this->GetUseSleeping())
    {   
        if (this->GetSleeping()) 
            return true;

        if ( (this->coord_dt.pos.LengthInf() < this->sleep_minspeed) &&
             ( 2.0*this->coord_dt.rot.LengthInf() < this->sleep_minwvel) )
        {
                if ((this->GetChTime() - this->sleep_starttime) > this->sleep_time)
                {
                    SetSleeping(true);
                    return true;
                }
        }
        else
        {
            this->sleep_starttime = float(this->GetChTime());
        }
    }
    return false;
}


void ChBody::AddMarker (ChSharedPtr<ChMarker> amarker)
{
    // don't allow double insertion of same object
    assert(std::find<std::vector<ChMarker*>::iterator>(marklist.begin(), marklist.end(), amarker.get_ptr())==marklist.end());

    amarker->SetBody (this);
    amarker->AddRef();
    marklist.push_back((amarker).get_ptr());
    
}

void ChBody::AddForce (ChSharedPtr<ChForce> aforce)
{
    // don't allow double insertion of same object
    assert(std::find<std::vector<ChForce*>::iterator>(forcelist.begin(), forcelist.end(), aforce.get_ptr())==forcelist.end());

    aforce->SetBody (this);
    aforce->AddRef();
    forcelist.push_back((aforce).get_ptr());
}
   

void ChBody::RemoveForce (ChSharedPtr<ChForce> mforce)
{
    // trying to remove objects not previously added?
    assert(std::find<std::vector<ChForce*>::iterator>(forcelist.begin(), forcelist.end(), mforce.get_ptr() )!=forcelist.end());

    // warning! linear time search
    forcelist.erase(std::find<std::vector<ChForce*>::iterator>(forcelist.begin(), forcelist.end(), mforce.get_ptr() ) );

    mforce->SetBody(0);
    mforce->RemoveRef();
}

void ChBody::RemoveMarker (ChSharedPtr<ChMarker> mmarker)
{
    // trying to remove objects not previously added?
    assert(std::find<std::vector<ChMarker*>::iterator>(marklist.begin(), marklist.end(), mmarker.get_ptr() )!=marklist.end());

    // warning! linear time search
    marklist.erase(std::find<std::vector<ChMarker*>::iterator>(marklist.begin(), marklist.end(), mmarker.get_ptr() ) );

    mmarker->SetBody(0);
    mmarker->RemoveRef();
}


void ChBody::RemoveAllForces() 
{ 
    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP)
    {
        FORCEpointer->SetBody(0);   
        FORCEpointer->RemoveRef();
        HIER_FORCE_NEXT
    }   
    forcelist.clear();
};

void ChBody::RemoveAllMarkers() 
{ 
    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP)
    {
        MARKpointer->SetBody(0);       
        MARKpointer->RemoveRef();
        HIER_MARKER_NEXT
    }

    marklist.clear();
};

/* old
ChMarker* ChBody::SearchMarker (char* m_name)
{
    return ChContainerSearchFromName<ChMarker, std::vector<ChMarker*>::iterator>
                (m_name, 
                marklist.begin(), 
                marklist.end());
}
ChForce* ChBody::SearchForce (char* m_name)
{
    return ChContainerSearchFromName<ChForce, std::vector<ChForce*>::iterator>
                (m_name, 
                forcelist.begin(), 
                forcelist.end());
}
*/
ChSharedPtr<ChMarker> ChBody::SearchMarker (char* m_name)
{
    ChMarker* mmark= ChContainerSearchFromName<ChMarker, std::vector<ChMarker*>::iterator>
                (m_name, 
                marklist.begin(), 
                marklist.end());
    if (mmark)
    {
        mmark->AddRef(); // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChMarker>(mmark));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChMarker>()); // not found? return a void shared ptr.
}
ChSharedPtr<ChForce> ChBody::SearchForce (char* m_name)
{
    ChForce* mforce = ChContainerSearchFromName<ChForce, std::vector<ChForce*>::iterator>
                (m_name, 
                forcelist.begin(), 
                forcelist.end());
    if (mforce)
    {
        mforce->AddRef(); // in that container pointers were not stored as ChSharedPtr, so this is needed..
        return (ChSharedPtr<ChForce>(mforce));  // ..here I am not getting a new() data, but a reference to something created elsewhere
    }
    return (ChSharedPtr<ChForce>()); // not found? return a void shared ptr.
}


                    //These are the members used to UPDATE
                    //the body coordinates during the animation
                    //Also the coordinates of forces and markers
                    //linked to the body will be updated.

void ChBody::UpdateMarkers (double mytime)
{
    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP)
    {
        MARKpointer->Update (mytime);

        HIER_MARKER_NEXT
    }
}

void ChBody::UpdateForces (double mytime)
{
    // COMPUTE LAGRANGIAN FORCES APPLIED TO BODY

    Xforce = VNULL;
    Xtorque = VNULL;

    // 1 - force caused by stabilizing damper ***OFF***

    // 2a- force caused by accumulation of forces in body's accumulator Force_acc
    if (Vnotnull(&Force_acc))
    {
        Xforce = Force_acc;     
    }

    // 2b- force caused by accumulation of torques in body's accumulator Force_acc
    if (Vnotnull(&Torque_acc))
    {
        Xtorque = Dir_World2Body(&Torque_acc); 
    }

    // 3 - accumulation of other applied forces
    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP)
    {   
          // update positions, f=f(t,q)
        FORCEpointer->Update (mytime);

        ChVector<> mforce; 
        ChVector<> mtorque;
        FORCEpointer->GetBodyForceTorque(&mforce,&mtorque);
        Xforce  += mforce;
        Xtorque += mtorque;
        
        HIER_FORCE_NEXT
    }

    // 4 - accumulation of script forces
    if (Vnotnull(&Scr_force))
    {
        Xforce += Scr_force;    
    }
    if (Vnotnull(&Scr_torque))
    {
        Xtorque += Dir_World2Body(&Scr_torque);
    }

    if (GetSystem())
    {
        Xforce += GetSystem()->Get_G_acc() * this->GetMass();
    }

}

void ChBody::UpdateTime (double mytime)
{
    ChTime = mytime;
}


void ChBody::UpdateState (Coordsys mypos, Coordsys mypos_dt)
{
    SetCoord (mypos);       // Set the state coordsys,
    SetCoord_dt (mypos_dt); // automatically updating auxiliary variables

    //TrySleeping();            // See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();           // Apply limits (if in speed clamping mode) to speeds.
    ComputeGyro ();         // Set the gyroscopic momentum.
}


void ChBody::UpdateStateTime (Coordsys mypos, Coordsys mypos_dt, double mytime)
{
    UpdateTime (mytime);

    UpdateState (mypos, mypos_dt);
}


void ChBody::Update (Coordsys mypos, Coordsys mypos_dt, double mytime)
{
    UpdateTime (mytime);

     mypos.rot.Normalize();
    UpdateState (mypos, mypos_dt);

    Update();

}



                            // UpdateALL updates the state and time
                            // of the object AND the dependant (linked)
                            // markers and forces.

void ChBody::Update()
{
    //TrySleeping();            // See if the body can fall asleep; if so, put it to sleeping 
    ClampSpeed();           // Apply limits (if in speed clamping mode) to speeds.
    ComputeGyro ();         // Set the gyroscopic momentum.

        // Also update the children "markers" and
        // "forces" depending on the body current state.
    UpdateMarkers(ChTime);

    UpdateForces(ChTime);
}



                            // As before, but keeps the current state.
                            // Mostly used for world reference body.
void ChBody::Update (double mytime)
{
                // For the body:
    UpdateTime (mytime);

    Update();
}



void ChBody::UpdateExternalGeometry ()
{
    if (this->GetExternalObject())
        this->GetExternalObject()->onChronoChanged();

    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP)
    {
        MARKpointer->UpdateExternalGeometry();
        HIER_MARKER_NEXT
    }

    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP)
    {
        FORCEpointer->UpdateExternalGeometry();
        HIER_FORCE_NEXT
    }
}

void ChBody::SetBodyFixed (bool mev)
{
    variables.SetDisabled(mev);
    if (mev == BFlagGet(BF_FIXED)) 
            return;
    BFlagSet(BF_FIXED, mev);
    //RecomputeCollisionModel(); // because one may use different model types for static or dynamic coll.shapes
}
 
// collision stuff
void ChBody::SetCollide (bool mcoll)
{
    if (mcoll == BFlagGet(BF_COLLIDE)) 
        return;

    if (mcoll)
    {
        SyncCollisionModels();
        BFlagSetON(BF_COLLIDE);
        if (GetSystem())
            GetSystem()->GetCollisionSystem()->Add(this->GetCollisionModel());
    }
    else 
    {
        BFlagSetOFF(BF_COLLIDE);
        if (GetSystem())
            GetSystem()->GetCollisionSystem()->Remove(this->GetCollisionModel());
    }
}

// forward reference
int coll_model_from_r3d(ChCollisionModel* chmodel, ChBody* mbody, int lod, Vector* mt, ChMatrix33<>* mr);

int ChBody::RecomputeCollisionModel()
{
    if (!GetCollide()) return FALSE; // do nothing unless collision enabled

    collision_model->ClearModel(); // ++++ start geometry definition
  
    if (this->GetExternalObject())
        this->GetExternalObject()->onAddCollisionGeometries(
                    this->collision_model,
                    this,
                    1,
                    &coord.pos,
                    &Amatrix);

    collision_model->BuildModel(); // ++++ complete geometry definition


    return TRUE;
}

void ChBody::SyncCollisionModels()
{
    this->GetCollisionModel()->SyncPosition();
}

void ChBody::AddCollisionModelsToSystem() 
{
    assert(this->GetSystem());
    SyncCollisionModels();
    this->GetSystem()->GetCollisionSystem()->Add(this->GetCollisionModel());
}

void ChBody::RemoveCollisionModelsFromSystem() 
{
    assert(this->GetSystem());
    this->GetSystem()->GetCollisionSystem()->Remove(this->GetCollisionModel());
}



void ChBody::GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax)
{
    if (this->GetCollisionModel())
        this->GetCollisionModel()->GetAABB(bbmin, bbmax);
    else 
        ChPhysicsItem::GetTotalAABB(bbmin, bbmax); // default: infinite aabb
}


//////// FILE I/O

void ChBody::StreamOUT(ChStreamOutBinary& mstream)
{
            // class version number
    mstream.VersionWrite(7);

        // serialize parent class too
    ChPhysicsItem::StreamOUT(mstream);

        // serialize parent class too
    ChFrameMoving<double>::StreamOUT(mstream);

        // stream out all member data
        ChVector<> vfoo=VNULL;
        double dfoo=0;
    mstream << variables.GetBodyMass();
    vfoo = GetInertiaXX();  mstream << vfoo;
    vfoo = GetInertiaXY();  mstream << vfoo;
    
    mstream << dfoo;
    mstream << bflag;
    dfoo=(double)density;   mstream << dfoo;

    mstream << max_speed;
    mstream << max_wvel;
    mstream << sleep_time;
    dfoo=(double)sleep_starttime;   mstream << dfoo;
    mstream << sleep_minspeed;
    mstream << sleep_minwvel;

    this->collision_model->StreamOUT(mstream); // also  mstream << (*this->collision_model);

    this->matsurface->StreamOUT(mstream); 
    dfoo=(double)conductivity;  mstream << dfoo;
}

void ChBody::StreamIN(ChStreamInBinary& mstream)
{
        // class version number
    int version = mstream.VersionRead();

        // deserialize parent class too
    if (version <4)
        ChObj::StreamIN(mstream);
    if (version >= 4)
        ChPhysicsItem::StreamIN(mstream);

    collision_model->ClearModel();


    if (version==1)
    {
            // stream in all member data
        int mlock; double dfoo;
        mstream >> mlock;
        mstream >> coord;       SetCoord(coord);
        mstream >> coord_dt;    SetCoord_dt(coord_dt);
        mstream >> coord_dtdt;  SetCoord_dtdt(coord_dtdt);
        mstream >> dfoo;        SetMass(dfoo);
        ChVector<> vfoo;
        mstream >> vfoo;        SetInertiaXX(vfoo);
        mstream >> vfoo;        SetInertiaXY(vfoo);
        mstream >> dfoo; //bdamper;
        if (version <7)
        {
            mstream >> dfoo;        matsurface->SetRestitution((float)dfoo);
            mstream >> dfoo;        //matsurface->SetRestitutionT((float)dfoo);
            mstream >> dfoo;        matsurface->SetKfriction((float)dfoo);
            mstream >> dfoo;        matsurface->SetSfriction((float)dfoo);
        }
        mstream >> bflag;
        mstream >> dfoo;        density = (float)dfoo;
        SetBodyFixed(mlock != 0);
    }
    if (version >=2)
    {
            // deserialize parent class too
        ChFrameMoving<double>::StreamIN(mstream);

            // stream in all member data
        double dfoo;
        mstream >> dfoo;        SetMass(dfoo);
        ChVector<> vfoo;
        mstream >> vfoo;        SetInertiaXX(vfoo);
        mstream >> vfoo;        SetInertiaXY(vfoo);
        mstream >> dfoo; //bdamper;
        if (version <7)
        {
            mstream >> dfoo;        matsurface->SetRestitution((float)dfoo);
            mstream >> dfoo;        //matsurface->SetRestitutionT((float)dfoo);
            mstream >> dfoo;        matsurface->SetKfriction((float)dfoo);
            mstream >> dfoo;        matsurface->SetSfriction((float)dfoo);
        }
        mstream >> bflag;
        mstream >> dfoo;        density = (float)dfoo;
        if(this->GetBodyFixed())
            SetBodyFixed(true);
        else SetBodyFixed(false);
    }
    if (version <3) SetUseSleeping(true);
    if (version >=3)
    {
        double dfoo;
        mstream >> max_speed;
        mstream >> max_wvel;
        mstream >> sleep_time;
        mstream >> dfoo;        sleep_starttime= (float)dfoo;
        mstream >> sleep_minspeed;
        mstream >> sleep_minwvel;
    }
    if ((version >=5) && (version < 7))
    {
        double dfoo;
        mstream >> dfoo;        matsurface->SetRollingFriction((float)dfoo);
        mstream >> dfoo;        matsurface->SetSpinningFriction((float)dfoo);
    }
    if (version >=6)
    {
        this->collision_model->StreamIN(mstream); // also   mstream >> (*collision_model);
        this->collision_model->BuildModel(); // because previously removed from ChSystem, if any.
    }
    if (version >=7)
    {
        double dfoo;
        this->matsurface->StreamIN(mstream);
        mstream >> dfoo;        conductivity = (float)dfoo;
    }
}




void ChBody::StreamOUTstate(ChStreamOutBinary& mstream)
{
    // Do not serialize parent classes and do not
    // implement versioning, because this must be efficient 
    // and will be used just for domain decomposition.
    this->GetCoord().StreamOUT(mstream);
    this->GetCoord_dt().StreamOUT(mstream);
}

void ChBody::StreamINstate(ChStreamInBinary& mstream)
{
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

int ChBody::StreamINall  (ChStreamInBinary& m_file)
{
    int mchunk = 0;
    ChMarker* newmarker= NULL;
    //ChForce*  newforce= NULL;

    // 0) reset body child lists
    RemoveAllMarkers();
    RemoveAllForces();


    // 1) read body class data...
 
    m_file >> *this; 

    m_file >> mchunk; 

    // 2) read child markers
    while (mchunk == CHCLASS_MARKER)
    {
        ChSharedPtr<ChMarker> newmarker(new ChMarker);
        this->AddMarker(newmarker);

        m_file >> *newmarker;

        newmarker->Impose_Abs_Coord (newmarker->GetAbsCoord());

        m_file >> mchunk;
    }

    // 3) read child links
    while (mchunk == CHCLASS_FORCE)
    {
        ChSharedPtr<ChForce> newforce(new ChForce);
        this->AddForce(newforce);

        m_file >> *newforce;

        m_file >> mchunk;
    }


    if (mchunk != CH_CHUNK_END) return 0;


    return 1;
}


int ChBody::StreamOUTall  (ChStreamOutBinary& m_file)
{

    // 1) read body class data...
    m_file << *this;

    // 2) read child bodies
    HIER_MARKER_INIT
    while HIER_MARKER_NOSTOP
    {
        m_file << (int)CHCLASS_MARKER;
        m_file << *MARKpointer;
        HIER_MARKER_NEXT
    }

    // 3) read child links
    HIER_FORCE_INIT
    while HIER_FORCE_NOSTOP
    {
        m_file << (int)CHCLASS_FORCE;
        m_file << *FORCEpointer;
        HIER_FORCE_NEXT
    }

    m_file << (int)CH_CHUNK_END;

    return 1;
}

void ChBody::StreamOUT(ChStreamOutAscii& mstream)
{
    mstream << "BODY   " << GetName() <<"\n";

    ChFrameMoving<double>::StreamOUT(mstream);

    //***TO DO***
}


int ChBody::StreamOUTall  (ChStreamOutAscii& mstream) // dump rigidbody and childrens (markers.forces)
{
    StreamOUT (mstream);             // 1) dump the body attrs

    HIER_MARKER_INIT
    while (HIER_MARKER_NOSTOP)   // 2) dump the markers
    {
        MARKpointer->StreamOUT (mstream);
        HIER_MARKER_NEXT
    }

    HIER_FORCE_INIT
    while (HIER_FORCE_NOSTOP) // 3) dump the forces
    {
        FORCEpointer->StreamOUT (mstream);
        HIER_FORCE_NEXT
    }

    return 1;
}










} // END_OF_NAMESPACE____


/////////////////////
