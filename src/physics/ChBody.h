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

#ifndef CHBODY_H
#define CHBODY_H


#include <math.h>

#include "physics/ChBodyFrame.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChForce.h"
#include "physics/ChMarker.h"
#include "physics/ChMaterialSurface.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpConstraint.h"



namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


typedef ChSharedPtr<ChForce>  ChSharedForcePtr;
typedef ChSharedPtr<ChMarker> ChSharedMarkerPtr;


/////////////////////////////////////
// Define body specific flags

#define BF_COLLIDE          (1L << 0)  // detects collisions
#define BF_CDINVISIBLE      (1L << 1)  // collision detection invisible
#define BF_EVAL_CONTACT_CN  (1L << 2)  // evaluate CONTACT_CN channel (normal restitution)
#define BF_EVAL_CONTACT_CT  (1L << 3)  // evaluate CONTACT_CT channel (tangential rest.)
#define BF_EVAL_CONTACT_KF  (1L << 4)  // evaluate CONTACT_KF channel (kinetic friction coeff)
#define BF_EVAL_CONTACT_SF  (1L << 5)  // evaluate CONTACT_SF channel (static friction coeff)
#define BF_SHOW_COLLMESH    (1L << 6)  // show collision mesh - obsolete
#define BF_FIXED            (1L << 7)  // body is fixed to ground
#define BF_LIMITSPEED       (1L << 8)  // body angular and linar speed is limited (clamped)
#define BF_SLEEPING         (1L << 9)  // body is sleeping [internal]
#define BF_USESLEEPING      (1L <<10)  // if body remains in same place for too long time, it will be frozen
#define BF_NOGYROTORQUE     (1L <<11)  // body do not get the gyroscopic (quadratic) term, for low-fi but stable RT simulations.




///
/// Class for rigid bodies. A rigid body is an entity which
/// can move in 3D space, and can be constrained to other rigid
/// bodies using ChLink objects. Rigid bodies can contain auxiliary
/// references (the ChMarker objects) and forces (the ChForce objects).
/// These objects have mass and inertia properties. A shape can also
/// be associated to the body, for collision detection.
///

class ChApi ChBody : 
						public ChPhysicsItem , 
						public ChBodyFrame
						//public ChFrameMoving<double> 
{

                        // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChBody,ChPhysicsItem);

protected:
    
                        // Pointer to the collision model, including the
                        // colliding geometry .
    ChCollisionModel* collision_model;

protected:
            //
            // DATA
            //

    int bflag;          // body-specific flags.
    unsigned int body_id; // HM - body specific identifier, used for indexing

                        // list of child markers
    std::vector<ChMarker*> marklist;

                        // list of child forces
    std::vector<ChForce*>  forcelist;
    


    ChVector<> gyro;        // The Qm gyroscopic torque, i.e. Qm= Wvel x (XInertia*Wvel)

    ChVector<> Xforce;      // The force  acting on body, applied to COG -in abs. coords-
    ChVector<> Xtorque;     // The torque acting on body  -in body rel. coords-


    ChVector<> Force_acc;   // force accumulator; (in abs space, applied to COG)
    ChVector<> Torque_acc;  // torque accumulator;(in abs space)

    ChVector<> Scr_force;   // script force accumulator; (in abs space, applied to COG)
    ChVector<> Scr_torque;  // script torque accumulator;(in abs space)

                        // data for surface contact and impact (can be shared):
    ChSharedPtr<ChMaterialSurface> matsurface;

                        // Auxiliary, stores position/rotation once a while
                        // when collision detection routines require to know
                        // the last time that coll.detect. was satisfied.
    ChCoordsys<> last_coll_pos;
                        

    float density;      // used when doing the 'recompute mass' operation.

                        // used to store mass matrix and coordinates, as
                        // an interface to the LCP solver.
    ChLcpVariablesBodyOwnMass   variables;

    float max_speed;    // limit on linear speed (useful for VR & videagames)
    float max_wvel;     // limit on angular vel. (useful for VR & videagames)

    float  sleep_time;
    float  sleep_minspeed;
    float  sleep_minwvel;
    float  sleep_starttime;

public:

            //
            // CONSTRUCTORS
            //

                /// Build a rigid body.
    ChBody ();
                /// Build a rigid body with a different collision model.
    ChBody (ChCollisionModel* new_collision_model);
                /// Destructor
    ~ChBody ();

                /// Copy from another ChBody. 
                /// NOTE: all settings of the body are copied, but the
                /// child hierarchy of ChForces and ChMarkers (if any) are NOT copied.
    void Copy(ChBody* source);


            //
            // FLAGS
            //


                /// Sets the 'fixed' state of the body. If true, it does not move
                /// respect to the absolute world, despite constraints, forces, etc.
    void SetBodyFixed (bool mev);
    bool GetBodyFixed()    {return BFlagGet(BF_FIXED);}

                /// If true, the normal restitution coefficient is evaluated
                /// from painted material channel.
    void SetEvalContactCn (bool mev) { BFlagSet(BF_EVAL_CONTACT_CN, mev);}
    bool GetEvalContactCn() {return BFlagGet(BF_EVAL_CONTACT_CN);}

                /// If true, the tangential restitution coefficient is evaluated
                /// from painted material channel.
    void SetEvalContactCt (bool mev) { BFlagSet(BF_EVAL_CONTACT_CT, mev);}
    bool GetEvalContactCt() {return BFlagGet(BF_EVAL_CONTACT_CT);}

                /// If true, the kinetic friction coefficient is evaluated
                /// from painted material channel.
    void SetEvalContactKf (bool mev) { BFlagSet(BF_EVAL_CONTACT_KF, mev);}
    bool GetEvalContactKf() {return BFlagGet(BF_EVAL_CONTACT_KF);}

                /// If true, the static friction coefficient is evaluated
                /// from painted material channel.
    void SetEvalContactSf (bool mev) { BFlagSet(BF_EVAL_CONTACT_SF, mev);}
    bool GetEvalContactSf() {return BFlagGet(BF_EVAL_CONTACT_SF);}

                /// Enable/disable the collision for this rigid body.
                /// (After setting ON, you may need RecomputeCollisionModel()
                /// before anim starts, if you added an external object 
                /// that implements onAddCollisionGeometries(), ex. in a plugin for a CAD)
    void  SetCollide (bool mcoll);
    bool  GetCollide() {return BFlagGet(BF_COLLIDE);}

                /// Show collision mesh in 3D views.
    void SetShowCollisionMesh    (bool mcoll) { BFlagSet(BF_SHOW_COLLMESH, mcoll);};
    bool GetShowCollisionMesh () {return BFlagGet(BF_SHOW_COLLMESH);};

                /// Trick. Set the maximum linear speed (beyond this limit it will
                /// be clamped). This is useful in virtual reality and real-time
                /// simulations, because it reduces the risk of bad collision detection.
                /// The realism is limited, but the simulation is more stable.
    void SetLimitSpeed    (bool mlimit) { BFlagSet(BF_LIMITSPEED, mlimit);};
    bool GetLimitSpeed()  {return BFlagGet(BF_LIMITSPEED);};

                /// Trick. Deactivate the gyroscopic torque (quadratic term).
                /// This is useful in virtual reality and real-time
                /// simulations, where objects that spin too fast with non-uniform inertia
                /// tensors (ex thin cylinders) might cause the integration to diverge quickly.
                /// The realism is limited, but the simulation is more stable.
    void SetNoGyroTorque    (bool mnogyro) { BFlagSet(BF_NOGYROTORQUE, mnogyro);};
    bool GetNoGyroTorque()  {return BFlagGet(BF_NOGYROTORQUE);};

                /// Trick. If use sleeping= true, bodies which stay in same place
                /// for too long time will be deactivated, for optimization.
                /// The realism is limited, but the simulation is faster.
    void SetUseSleeping    (bool ms) { BFlagSet(BF_USESLEEPING, ms);};
    bool GetUseSleeping()  {return BFlagGet(BF_USESLEEPING);};

                /// Force the body in sleeping mode or not (usually this state change is not
                /// handled by users, anyway, because it is mostly automatic).
    void SetSleeping    (bool ms) { BFlagSet(BF_SLEEPING, ms);};
                /// Tell if the body is actually in sleeping state.
    bool GetSleeping()  {return BFlagGet(BF_SLEEPING);};

                /// Put the body in sleeping state if requirements are satisfied.
    bool TrySleeping();

                /// Tell if the body is active, i.e. it is neither fixed to ground nor
                /// it is in sleep mode.
    bool IsActive() {return !BFlagGet(BF_SLEEPING | BF_FIXED);}
                /// Set the body identifier - HM
    void SetId(int identifier) { body_id = identifier; }
                /// Set the body identifier - HM
    unsigned int GetId() { return body_id;}

            //
            // FUNCTIONS
            //

                /// Number of coordinates of the rigid body =6 (internally, 3+4=7 coords are used 
                /// since quaternions are used for large rotations, but local coords -ex. w&v velocity- are 6)
    virtual int GetDOF  ()   {return 6;}

                /// Returns reference to the encapsulated ChLcpVariablesBody,
                /// representing body variables (pos, speed or accel.- see VariablesLoad...() )
                /// and forces.
                /// The ChLcpVariablesBodyOwnMass is the interface to the LCP system solver.
    ChLcpVariablesBodyOwnMass& VariablesBody() {return variables;}
    ChLcpVariables&            Variables()     {return variables;}


             // Override/implement LCP system functions of ChPhysicsItem
             // (to assembly/manage data for LCP system solver)

                /// Sets the 'fb' part of the encapsulated ChLcpVariablesBodyOwnMass to zero.
    void VariablesFbReset();

                /// Adds the current forces applied to body (including gyroscopic torque) in
                /// encapsulated ChLcpVariablesBody, in the 'fb' part: qf+=forces*factor
    void VariablesFbLoadForces(double factor=1.);

                /// Initialize the 'qb' part of the ChLcpVariablesBody with the 
                /// current value of body speeds. Note: since 'qb' is the unknown of the LCP, this
                /// function seems unuseful, unless used before VariablesFbIncrementMq()
    void VariablesQbLoadSpeed();

                /// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
                /// with v_old using VariablesQbLoadSpeed, this method can be used in 
                /// timestepping schemes that do: M*v_new = M*v_old + forces*dt
    void VariablesFbIncrementMq();

                /// Fetches the body speed (both linear and angular) from the
                /// 'qb' part of the ChLcpVariablesBody (does not updates the full body&markers state)
                /// and sets it as the current body speed.
                /// If 'step' is not 0, also computes the approximate acceleration of
                /// the body using backward differences, that is  accel=(new_speed-old_speed)/step.
                /// Mostly used after the LCP provided the solution in ChLcpVariablesBody .
    void VariablesQbSetSpeed(double step=0.);

                /// Increment body position by the 'qb' part of the ChLcpVariablesBody,
                /// multiplied by a 'step' factor.
                ///     pos+=qb*step
                /// If qb is a speed, this behaves like a single step of 1-st order
                /// numerical integration (Eulero integration).
                /// Does not automatically update markers & forces.
    void VariablesQbIncrementPosition(double step);


                /// Tell to a system descriptor that there are variables of type
                /// ChLcpVariables in this object (for further passing it to a LCP solver)
    virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);


                /// Instantiate the collision model
    virtual ChCollisionModel* InstanceCollisionModel();

               // Other functions

                /// Set no speed and no accelerations (but does not change the position)
    void SetNoSpeedNoAcceleration();


                /// Acess the collision model for the collision engine.
                /// To get a non-null pointer, remember to SetCollide(true), before.
    ChCollisionModel* GetCollisionModel() {return collision_model;}

                /// Synchronize coll.model coordinate and bounding box to the position of the body.
    virtual void SyncCollisionModels();
    virtual void AddCollisionModelsToSystem();
    virtual void RemoveCollisionModelsFromSystem();

                /// Update the optimization structures (OOBB, ABB, etc.)
                /// of the collision model, from the associated geometry in some external object (es.CAD).
    int RecomputeCollisionModel();

                /// Gets the last position when the collision detection was
                /// performed last time (i.e. last time SynchronizeLastCollPos() was used)
    ChCoordsys<> GetLastCollPos () { return last_coll_pos; }
                /// Stores the current position in the last-collision-position buffer.
    void SynchronizeLastCollPos() {last_coll_pos = this->coord;}

                /// Get the rigid body coordinate system that represents
                /// the GOG (Center of Gravity). The mass and inertia tensor
                /// are defined respect to this coordinate system, that is also
                /// assumed the default main coordinates of the body. 
                /// By default, doing mybody.GetPos() etc. is like mybody.GetFrame_COG_abs().GetPos() etc.
    virtual ChFrame<>& GetFrame_COG_to_abs() {return *this;}

                /// Get the rigid body coordinate system that is used for
                /// defining the collision shapes and the ChMarker objects.
                /// For the base ChBody, this is always the same reference of the COG.
    virtual ChFrame<>& GetFrame_REF_to_abs() {return *this;}


                /// Get the master coordinate system for the assets (this will return the 
                /// main coordinate system of the rigid body)
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone=0) { return(GetFrame_REF_to_abs());}


                /// Get the entire AABB axis-aligned bounding box of the object,
                /// as defined by the collision model (if any).
    virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax);

                /// Method to deserialize only the state (position, speed)
    virtual void StreamINstate(ChStreamInBinary& mstream);
                /// Method to serialize only the state (position, speed)
    virtual void StreamOUTstate(ChStreamOutBinary& mstream);    

                /// Access the material surface properties, referenced by this 
                /// rigid body. The material surface contains properties such as friction, etc. 
                /// The ChMaterialSurface can be a shared object! (by default, each body creates its
                /// own as soon as instanced, but later the material object can be replaced).
    ChSharedPtr<ChMaterialSurface>& GetMaterialSurface() {return this->matsurface;}
                /// Set the material surface properties by passing a ChMaterialSurface object.
                /// Thank to smart pointers, the one that was previously used is replaced and,
                /// if needed, it is automatically dereferenced and deleted.
                /// The ChMaterialSurface can be a shared object! (by default, each body creates its
                /// own as soon as instanced, but later the material object can be replaced).
    void SetMaterialSurface(ChSharedPtr<ChMaterialSurface>& mnewsurf) {this->matsurface = mnewsurf;}


                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    float  GetImpactC() {return this->matsurface->GetRestitution();}
    void   SetImpactC(float mval) {this->matsurface->SetRestitution(mval);}
                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    float  GetImpactCt() {return 0;}
    void   SetImpactCt(float mval) {}
                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    float  GetKfriction() {return this->matsurface->GetKfriction();}
    void   SetKfriction(float mval) {this->matsurface->SetKfriction(mval);}
                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    float  GetSfriction() {return this->matsurface->GetSfriction();}
    void   SetSfriction(float mval) {this->matsurface->SetSfriction(mval);}
                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    void   SetFriction(float mval) {this->matsurface->SetFriction(mval);}
                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    float  GetRollingFriction() {return this->matsurface->GetRollingFriction();}
    void   SetRollingFriction(float mval) {this->matsurface->SetRollingFriction(mval);}
                /// FOR BACKWARD COMPATIBILITY ONLY. Better use: GetMaterialSurface()->Get...  etc.etc.
    float  GetSpinningFriction() {return this->matsurface->GetSpinningFriction();}
    void   SetSpinningFriction(float mval) {this->matsurface->SetSpinningFriction(mval);}


                /// The density of the rigid body, as [mass]/[unit volume]. Used just if
                /// the inertia tensor and mass are automatically recomputed from the
                /// geometry (in case a CAD plugin for example provides the surfaces.)
    float  GetDensity() {return density;}
    void   SetDensity(float mdensity) {density = mdensity;}



            //
            // DATABASE HANDLING.
            //
            // To attach/remove items (rigid bodies, links, etc.) you must use 
            // shared pointer, so that you don't need to care about item deletion, 
            // which will be automatic when needed.
            // Please don't add the same item multiple times; also, don't remove
            // items which haven't ever been added.
            // NOTE! After adding/removing items to the system, you should call Update() !

                /// Attach a marker to this body. 
    void AddMarker (ChSharedPtr<ChMarker> amarker);
                /// Attach a force to this body. 
    void AddForce (ChSharedPtr<ChForce> aforce);

                /// Remove a specific marker from this body. Warning: linear time search.
    void RemoveMarker (ChSharedPtr<ChMarker> amarker);
                /// Remove a specific force from this body. Warning: linear time search.
    void RemoveForce  (ChSharedPtr<ChForce> aforce);

                /// Remove all markers at once. Faster than doing multiple RemoveForce()
                /// Don't care about deletion: it is automatic, only when needed.
    void RemoveAllForces();
                /// Remove all markers at once. Faster than doing multiple RemoveForce()
                /// Don't care about deletion: it is automatic, only when needed.
    void RemoveAllMarkers();

                /// Finds a marker from its ChObject name
    ChSharedPtr<ChMarker> SearchMarker (char* m_name);
                /// Finds a force from its ChObject name
    ChSharedPtr<ChForce>  SearchForce (char* m_name);

                /// Gets the list of children markers.
                /// NOTE! use this list only to enumerate etc., but NOT to
                /// remove or add items (use the appropriate Remove.. and Add.. 
                /// functions instead!)
    std::vector<ChMarker*>* GetMarkerList() {return &marklist;} 

                /// Gets the list of children forces.
                /// NOTE! use this list only to enumerate etc., but NOT to
                /// remove or add items (use the appropriate Remove.. and Add.. 
                /// functions instead!)
    std::vector<ChForce*>* GetForceList() {return &forcelist;}

    



            //
            // Point/vector transf.(NOTE! you may also use operators of ChMovingFrame)
            //

    ChVector<> Point_World2Body(const ChVector<>& mpoint);
    ChVector<> Point_Body2World(const ChVector<>& mpoint);
    ChVector<> Dir_World2Body(const ChVector<>& mpoint);
    ChVector<> Dir_Body2World(const ChVector<>& mpoint);
    ChVector<> RelPoint_AbsSpeed(const ChVector<>& mrelpoint);
    ChVector<> RelPoint_AbsAcc(const ChVector<>& mrelpoint);

                /// Mass of the rigid body. Must be positive.
                /// Try not to mix bodies with too high/too low values of mass, for numerical stability.
    void   SetMass (double newmass) { if (newmass>0.) variables.SetBodyMass(newmass);}
    double GetMass() {return variables.GetBodyMass();}

                /// Set the inertia tensor of the body
    void SetInertia (const ChMatrix33<>& newXInertia);
                /// Set the diagonal part of the inertia tensor (Ixx, Iyy, Izz values)
    void SetInertiaXX(const ChVector<>& iner);
                /// Get the diagonal part of the inertia tensor (Ixx, Iyy, Izz values)
    ChVector<> GetInertiaXX();
                /// Set the extradiagonal part of the inertia tensor
                /// (Ixy, Iyz, Izx values, the rest is symmetric)
                /// Warning about sign: in some books they write the inertia tensor as 
                /// I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; etc.]
    void SetInertiaXY(const ChVector<>& iner);
                /// Get the extradiagonal part of the inertia tensor
                /// (Ixy, Iyz, Izx values, the rest is symmetric)
                /// Warning about sign: in some books they write the inertia tensor as 
                /// I=[Ixx, -Ixy, -Ixz; etc.] but here is I=[Ixx, Ixy, Ixz; etc.]
    ChVector<> GetInertiaXY();


                /// Trick. Set the maximum linear speed (beyond this limit it will
                /// be clamped). This is useful in virtual reality and real-time
                /// simulations, because it reduces the risk of bad collision detection.
                /// This speed limit is active only if you set  SetLimitSpeed(true);
    void   SetMaxSpeed(float m_max_speed) {max_speed = m_max_speed;}
    float  GetMaxSpeed () {return max_speed;}

                /// Trick. Set the maximum angualar speed (beyond this limit it will
                /// be clamped). This is useful in virtual reality and real-time
                /// simulations, because it reduces the risk of bad collision detection.
                /// This speed limit is active only if you set  SetLimitSpeed(true);
    void   SetMaxWvel(float m_max_wvel) {max_wvel = m_max_wvel;}
    float  GetMaxWvel () {return max_wvel;}

                /// When this function is called, the speed of the body is clamped
                /// into limits posed by max_speed and max_wvel  - but remember to
                /// put the body in the SetLimitSpeed(true) mode.
    void ClampSpeed();

                /// Set the amount of time which must pass before going automatically in
                /// sleep mode when the body has very small movements.
    void   SetSleepTime(float m_t) {sleep_time = m_t;}
    float GetSleepTime () {return sleep_time;}

                /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void   SetSleepMinSpeed(float m_t) {sleep_minspeed = m_t;}
    float GetSleepMinSpeed () {return sleep_minspeed;}

                /// Set the max linear speed to be kept for 'sleep_time' before freezing.
    void   SetSleepMinWvel(float m_t) {sleep_minwvel = m_t;}
    float GetSleepMinWvel () {return sleep_minwvel;}



                /// Computes the 4x4 inertia tensor in quaternion space, if needed
    void ComputeQInertia(ChMatrixNM<double,4,4>* mQInertia);

                /// Computes the gyroscopic torque. In fact, in sake of highest
                /// speed, the gyroscopic torque isn't automatically updated each time a
                /// SetCoord() or SetCoord_dt() etc. is called, but only if necessary,
                /// for each UpdateState().
    void ComputeGyro ();


                /// Transform and adds a cartesian force to a generic 7x1 vector of body lagrangian forces mQf .
                /// The carthesian force must be passed as vector and application point, and vcan be either in local
                /// (local = TRUE) or absolute reference (local = FALSE)
    void Add_as_lagrangian_force(const ChVector<>& force, const ChVector<>& appl_point, int local, ChMatrixNM<double,7,1>* mQf);
    void Add_as_lagrangian_torque(const ChVector<>& torque, int local, ChMatrixNM<double,7,1>* mQf);

                /// Given a lagrangian force (in a 7x1 matrix), computes the fore and torque as vectors.
    void From_lagrangian_to_forcetorque(const ChMatrixNM<double,7,1>& mQf, ChVector<>& mforce, ChVector<>& mtorque);
                /// Given force and torque as vectors, computes the lagrangian force (in a 7x1 matrix)
    void From_forcetorque_to_lagrangian(const ChVector<>& mforce, const ChVector<>& mtorque, ChMatrixNM<double,7,1>& mQf);


            //
            // UTILITIES FOR FORCES/TORQUES:
            //


                /// Add forces and torques into the "accumulators", as increment.
                /// Forces and torques currently in accumulators will affect the body.
                /// It's up to the user to remember to empty them and/or set again at each 
                /// integration step. Useful to apply forces to bodies without needing to
                /// add ChForce() objects. If local=true, force,appl.point or torque are considered
                /// expressed in body coordinates, otherwise are considered in absolute coordinates.
    void Accumulate_force(const ChVector<>& force, const ChVector<>& appl_point, int local);
    void Accumulate_torque(const ChVector<>& torque, int local);
    ChVector<> Get_accumulated_force  () {return Force_acc;};
    ChVector<> Get_accumulated_torque () {return Torque_acc;};
    void Empty_forces_accumulators () {Force_acc = VNULL; Torque_acc = VNULL;};

                /// To get & set the 'script' force buffers(only accessed by external scripts, so
                /// It's up to the script to remember to set& reset them -link class just add them to
                /// all other forces. Script forces&torques are considered applied to COG, in abs csys.
    ChVector<>* Get_Scr_force() {return &Scr_force;};
    ChVector<>* Get_Scr_torque() {return &Scr_torque;};
    void Set_Scr_force(ChVector<> mf) {Scr_force = mf;};
    void Set_Scr_torque(ChVector<> mf) {Scr_torque = mf;};
    void Accumulate_script_force(const ChVector<>& force, const ChVector<>& appl_point, int local);
    void Accumulate_script_torque(const ChVector<>& torque, int local);

                /// Return the gyroscopic torque.
    ChVector<>  Get_gyro() {return gyro;}

                /// Get the total force applied to the rigid body (applied at center of mass.
                /// expressed in absolute coordinates).
    ChVector<> Get_Xforce () {return Xforce;}
                /// Get the total torque applied to the rigid body (expressed in body coordinates).
                /// This does not include the gyroscopic torque.
    ChVector<> Get_Xtorque() {return Xtorque;}

                /// Get the address of the inertia tensor, as a 3x3 matrix,
                /// expressed in local coordinate system.
    const ChMatrix33<>& GetXInertia() {return variables.GetBodyInertia();}


            // Body-specific flag handling

    void BFlagsSetAllOFF () {bflag = 0;}
    void BFlagsSetAllON () {bflag = 0; bflag = ~ bflag;}
    void BFlagSetON  (int mask) {bflag |= mask ;}
    void BFlagSetOFF (int mask) {bflag &= ~ mask;}
    bool BFlagGet    (int mask) {return (bflag & mask)!=0;};
    void BFlagSet    (int mask, bool state) {if (state) bflag |= mask; else bflag &= ~ mask;};

            //
            // UPDATE FUNCTIONS
            //

                /// Update all children markers of the rigid body, at current body state
    void UpdateMarkers (double mytime);
                /// Update all children forces of the rigid body, at current body state.
    void UpdateForces (double mytime);
                /// Update local time of rigid body, and time-dependant data
    void UpdateTime (double mytime);
                /// Update all auxiliary data of the rigid body, at given time
    void UpdateState(const ChCoordsys<>& mypos, const ChCoordsys<>& mypos_dt);
                /// Update all auxiliary data of the rigid body, at given time and state
    void UpdateStateTime(const ChCoordsys<>& mypos, const ChCoordsys<>& mypos_dt, double mytime);
                /// Update all auxiliary data of the rigid body and of
                /// its children (markers, forces..), at given time and state
    void Update(const ChCoordsys<>& mypos, const ChCoordsys<>& mypos_dt, double mytime);


                /// Update all auxiliary data of the rigid body and of
                /// its children (markers, forces..), at given time
    virtual void Update(double mytime);
                /// Update all auxiliary data of the rigid body and of
                /// its children (markers, forces..)
    virtual void Update();


                /// Tells to the associated external object ChExternalObject() ,if any,
                /// that its 3D shape must be updated in order to syncronize to ChBody
                /// coordinates
    void UpdateExternalGeometry ();


            //
            // STREAMING
            //


                /// Method to allow deserializing a persistent binary archive (ex: a file)
                /// into transient data.
    void StreamIN(ChStreamInBinary& mstream);

                /// Method to allow serializing transient data into a persistent
                /// binary archive (ex: a file).
    void StreamOUT(ChStreamOutBinary& mstream);

                /// Save data, including child markers and child forces
    int StreamOUTall(ChStreamOutBinary& m_file);
                /// Read data, including child markers and child forces
    int StreamINall(ChStreamInBinary&  m_file);


                /// Method to allow serialization of transient data in ascii,
                /// as a readable item, for example   "chrono::GetLog() << myobject;"
    void StreamOUT(ChStreamOutAscii& mstream);

    int  StreamOUTall(ChStreamOutAscii& mstream);

};




const int BODY_DOF = 6;  ///< degrees of freedom of body in 3d space
const int BODY_QDOF = 7; ///< degrees of freedom with quaternion rotation state
const int BODY_ROT = 3;  ///< rotational dof in Newton dynamics


typedef ChSharedPtr<ChBody> ChSharedBodyPtr; 



} // END_OF_NAMESPACE____


#endif
