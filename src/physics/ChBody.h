#ifndef CHBODY_H
#define CHBODY_H

//////////////////////////////////////////////////
//
//   ChBody.h
//
//   Class for rigid bodies, that is rigid moving
//   parts with mass and maybe collision geometry.
//   A rigid body encloses markers and forces too.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "core/ChFrameMoving.h"
#include "core/ChShared.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChForce.h"
#include "physics/ChMarker.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpConstraint.h"



namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


typedef ChSharedPtr<ChForce>  ChSharedForcePtr;
typedef ChSharedPtr<ChMarker> ChSharedMarkerPtr;
typedef ChSharedPtr<ChMarker> ChSharedMarkerPtr;


/////////////////////////////////////
// Define body specific flags

#define BF_COLLIDE			(1L << 0)  // detects collisions
#define BF_CDINVISIBLE		(1L << 1)  // collision detection invisible
#define BF_EVAL_CONTACT_CN	(1L << 2)  // evaluate CONTACT_CN channel (normal restitution)
#define BF_EVAL_CONTACT_CT	(1L << 3)  // evaluate CONTACT_CT channel (tangential rest.)
#define BF_EVAL_CONTACT_KF	(1L << 4)  // evaluate CONTACT_KF channel (kinetic friction coeff)
#define BF_EVAL_CONTACT_SF	(1L << 5)  // evaluate CONTACT_SF channel (static friction coeff)
#define BF_SHOW_COLLMESH	(1L << 6)  // show collision mesh
#define BF_FIXED			(1L << 7)  // body is fixed to ground
#define BF_LIMITSPEED		(1L << 8)  // body angular and linar speed is limited (clamped)
#define BF_SLEEPING			(1L << 9)  // body is sleeping [internal]
#define BF_USESLEEPING		(1L <<10)  // if body remains in same place for too long time, it will be frozen


///
/// Class for rigid bodies. A rigid body is an entity which
/// can move in 3D space, and can be constrained to other rigid
/// bodies using ChLink objects. Rigid bodies can contain auxiliary
/// references (the ChMarker objects) and forces (the ChForce objects).
/// These objects have mass and inertia properties. A shape can also
/// be associated to the body, for collision detection.
///

class ChBody : public ChPhysicsItem , public ChFrameMoving<double> {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBody,ChPhysicsItem);

private:
			//
	  		// DATA
			//

	int bflag;			// body-specific flags.

						// list of child markers
	std::vector<ChMarker*> marklist;

						// list of child forces
	std::vector<ChForce*>  forcelist;
	


	ChVector<> gyro;		// The Qm gyroscopic torque, i.e. Qm= Wvel x (XInertia*Wvel)

	ChVector<> Xforce;		// The force  acting on body, applied to COG -in abs. coords-
	ChVector<> Xtorque;		// The torque acting on body  -in body rel. coords-


	ChVector<> Force_acc;	// force accumulator; (in abs space, applied to COG)
	ChVector<> Torque_acc;	// torque accumulator;(in abs space)

	ChVector<> Scr_force;	// script force accumulator; (in abs space, applied to COG)
	ChVector<> Scr_torque;	// script torque accumulator;(in abs space)

				// Data for surface contact and impact:

	float impactC;		// impact restitution coefficient
	float impactCt;		// tangential impact restitution coefficient
	float k_friction;	// kinematic friction coefficient for surface contact
	float s_friction;	// static friction for rest-contact (sticking)
	float rolling_friction; // rolling friction 
	float spinning_friction; // rolling friction 

						// Pointer to the collision model, including the
						// colliding geometry .
	ChCollisionModel* collision_model;
	 
						// Auxiliary, stores position/rotation once a while
						// when collision detection routines require to know
						// the last time that coll.detect. was satisfied.
	ChCoordsys<> last_coll_pos;
						// (same as above, for speeds)
	ChCoordsys<> last_coll_pos_dt;

	float density;		// used when doing the 'recompute mass' operation.

						// used to store mass matrix and coordinates, as
						// an interface to the LCP solver.
	ChLcpVariablesBodyOwnMass	variables;

	float max_speed;	// limit on linear speed (useful for VR & videagames)
	float max_wvel;		// limit on angular vel. (useful for VR & videagames)

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
	bool GetBodyFixed()	   {return BFlagGet(BF_FIXED);}

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
				/// After setting ON, remember RecomputeCollisionModel()
				/// before anim starts (it is not automatically
				/// recomputed here because of performance issues.)
	void  SetCollide (int mcoll);
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


			//
	  		// FUNCTIONS
			//

				/// Number of coordinates of the rigid body =6 (internally, 3+4=7 coords are used 
				/// since quaternions are used for large rotations, but local coords -ex. w&v velocity- are 6)
	virtual int GetDOF  ()   {return 6;}

				/// Returns reference to the encapsulated ChLcpVariablesBody,
				/// representing body variables (pos, speed or accel.- see VariablesLoad...() )
				/// and forces.
				/// The ChLcpVariablesBodyOwnMass is the interface ta the LCP system solver.
	ChLcpVariablesBodyOwnMass& Variables() {return variables;}


			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

				/// Sets the 'fb' part of the encapsulated ChLcpVariablesBodyOwnMass to zero.
	void VariablesFbReset();

				/// Adds the current forces applied to body (including gyroscopic torque) in
				/// encapsulated ChLcpVariablesBody, in the 'fb' part: qf+=forces*factor
	void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariablesBody with the 
				/// current value of body speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function sems unuseful, however the LCP solver has an option 'add_Mq_to_f', that
				/// takes [M]*qb and add to the 'fb' term before starting (this is often needed in
				/// the Anitescu time stepping method, for instance); this explains the need of this method..
	void VariablesQbLoadSpeed();


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
	Coordsys GetLastCollPos () { return last_coll_pos; }
				/// Stores the current position in the last-collision-position buffer.
	void SynchronizeLastCollPos() {last_coll_pos = this->coord;}

				/// Gets the last speed when the collision detection was
				/// performed last time (i.e. last time SynchronizeLastCollPos_dt() was used)
	Coordsys GetLastCollPos_dt () { return last_coll_pos_dt; }
				/// Stores the current speed in the last-collision-position buffer.
	void SynchronizeLastCollPos_dt() {last_coll_pos_dt = this->coord_dt;}


				/// The normal restitution coefficient, for collisions.
				/// Should be in 0..1 range.
	float  GetImpactC() {return impactC;}
	void   SetImpactC(float mval) {impactC = mval;}

				/// The tangential restitution coefficient, for collisions (--not used--)
	float  GetImpactCt() {return impactCt;}
	void   SetImpactCt(float mval) {impactCt = mval;}

				/// The kinetic friction coefficient. 
				/// Usually in 0..1 range, rarely above. Default 0.6 
				/// Note: currently the static friction will be used instead, anyway, because of an issue in the solver.
	float  GetKfriction() {return k_friction;}
	void   SetKfriction(float mval) {k_friction = mval;}

				/// The static friction coefficient. 
				/// Usually a bit higher than kinetic coeff. Default 0.6
	float  GetSfriction() {return s_friction;}
	void   SetSfriction(float mval) {s_friction = mval;}

				/// Set both static friction and kinetic friction at once, with same value.
	void   SetFriction(float mval) {SetSfriction(mval); SetKfriction(mval);}

				/// The rolling friction coefficient. Usually a very low coefficient.
				/// Note! a non-zero value will make the simulation 2x slower! Also, the
				/// GPU solver currently does not support rolling friction. Default: 0.
	float  GetRollingFriction() {return rolling_friction;}
	void   SetRollingFriction(float mval) {rolling_friction = mval;}

				/// The spinning friction coefficient. Usually a very low coefficient.
				/// Note! a non-zero value will make the simulation 2x slower! Also, the
				/// GPU solver currently does not support spinning friction. Default: 0.
	float  GetSpinningFriction() {return spinning_friction;}
	void   SetSpinningFriction(float mval) {spinning_friction = mval;}

				/// The density of the rigid body, as [mass]/[unit volume]. Used just if
				/// the inertia tensor and mass are automatically recomputed from the
				/// geometry (in case the Realsoft3D plugin for example provides the surfaces.)
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
	void AddMarker (ChSharedMarkerPtr amarker);
				/// Attach a force to this body. 
	void AddForce (ChSharedForcePtr aforce);

				/// Remove a specific marker from this body. Warning: linear time search.
	void RemoveMarker (ChSharedMarkerPtr amarker);
				/// Remove a specific force from this body. Warning: linear time search.
	void RemoveForce  (ChSharedForcePtr aforce);

				/// Remove all markers at once. Faster than doing multiple RemoveForce()
				/// Don't care about deletion: it is automatic, only when needed.
	void RemoveAllForces();
				/// Remove all markers at once. Faster than doing multiple RemoveForce()
				/// Don't care about deletion: it is automatic, only when needed.
	void RemoveAllMarkers();

				/// Finds a marker from its ChObject name
	ChMarker* SearchMarker (char* m_name);
				/// Finds a force from its ChObject name
	ChForce*  SearchForce (char* m_name);

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

	Vector Point_World2Body (Vector* mpoint);
	Vector Point_Body2World (Vector* mpoint);
	Vector Dir_World2Body (Vector* mpoint);
	Vector Dir_Body2World (Vector* mpoint);
	Vector RelPoint_AbsSpeed(Vector* mrelpoint);
	Vector RelPoint_AbsAcc(Vector* mrelpoint);

				/// Mass of the rigid body. Must be positive.
				/// Try not to mix bodies with too high/too low values of mass, for numerical stability.
	void   SetMass (double newmass) { if (newmass>0.) variables.SetBodyMass(newmass);}
	double GetMass() {return variables.GetBodyMass();}

				/// Set the inertia tensor of the body
	void SetInertia (ChMatrix33<>* newXInertia);
				/// Set the diagonal part of the inertia tensor.
	void SetInertiaXX (Vector iner);
				/// Get the diagonal part of the inertia tensor.
	Vector GetInertiaXX();
				/// Set the extradiagonal part of the inertia tensor
				/// (xy, yz, zx values, the rest is symmetric)
	void SetInertiaXY (Vector iner);
				/// Get the extradiagonal part of the inertia tensor
				/// (xy, yz, zx values, the rest is symmetric)
	Vector GetInertiaXY();


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
	void Add_as_lagrangian_force(Vector force, Vector appl_point, int local, ChMatrixNM<double,7,1>* mQf);
	void Add_as_lagrangian_torque(Vector torque, int local, ChMatrixNM<double,7,1>* mQf);

				/// Given a lagrangian force (in a 7x1 matrix), computes the fore and torque as vectors.
	void From_lagrangian_to_forcetorque(ChMatrixNM<double,7,1>* mQf, Vector* mforce, Vector* mtorque);
				/// Given force and torque as vectors, computes the lagrangian force (in a 7x1 matrix)
	void From_forcetorque_to_lagrangian(Vector* mforce, Vector* mtorque, ChMatrixNM<double,7,1>* mQf);


			//
			// UTILITIES FOR FORCES/TORQUES:
			//

				/// Trasform generic cartesian force into absolute force+torque applied to body COG.
				/// If local=1, force & application point are intended as expressed in local
				/// coordinates, if =0, in absolute.
	void To_abs_forcetorque  (Vector force, Vector appl_point, int local, Vector& resultforce, Vector& resulttorque);

				/// Trasform generic cartesian torque into absolute torque applied to body COG.
				/// If local=1, torque is intended as expressed in local coordinates, if =0, in absolute.
	void To_abs_torque (Vector torque, int local, Vector& resulttorque);

				/// As before, but puts the result into the "accumulators", as increment.
				/// Forces and torques currently in accumulators will affect the body.
				/// It's up to the user to remember to empty them and/or set again at each 
				/// integration step. Useful to apply forces to bodies without needing to
				/// add ChForce() objects. If local=true, force,appl.point or torque are considered
				/// expressed in body coordinates, otherwise are considered in absolute coordinates.
	void Accumulate_force  (Vector force, Vector appl_point, int local);
	void Accumulate_torque (Vector torque, int local);
	Vector Get_accumulated_force  () {return Force_acc;};
	Vector Get_accumulated_torque () {return Torque_acc;};
	void Empty_forces_accumulators () {Force_acc = VNULL; Torque_acc = VNULL;};

				/// To get & set the 'script' force buffers(only accessed by external scripts, so
				/// It's up to the script to remember to set& reset them -link class just add them to
				/// all other forces. Script forces&torques are considered applied to COG, in abs csys.
	Vector* Get_Scr_force() {return &Scr_force;};
	Vector* Get_Scr_torque() {return &Scr_torque;};
	void Set_Scr_force(Vector mf) {Scr_force = mf;};
	void Set_Scr_torque(Vector mf) {Scr_torque = mf;};
	void Accumulate_script_force (Vector force, Vector appl_point, int local);
	void Accumulate_script_torque (Vector torque, int local);

				/// Return the gyroscopic torque.
	Vector  Get_gyro() {return gyro;}

				/// Get the total force applied to the rigid body (applied at center of mass.
				/// expressed in absolute coordinates).
	Vector Get_Xforce () {return Xforce;}
				/// Get the total torque applied to the rigid body (expressed in body coordinates).
				/// This does not include the gyroscopic torque.
	Vector Get_Xtorque() {return Xtorque;}

				/// Get the address of the inertia tensor, as a 3x3 matrix,
				/// expressed in local coordinate system.
	ChMatrix33<>* GetXInertia () { return &variables.GetBodyInertia();}


			// Body-specific flag handling

	void BFlagsSetAllOFF () {bflag = 0;}
	void BFlagsSetAllON () {bflag = 0; bflag = ~ bflag;}
	void BFlagSetON  (int mask) {bflag |= mask ;}
	void BFlagSetOFF (int mask) {bflag &= ~ mask;}
	bool BFlagGet	 (int mask) {return (bflag & mask)!=0;};
	void BFlagSet	 (int mask, bool state) {if (state) bflag |= mask; else bflag &= ~ mask;};

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
	void UpdateState (Coordsys mypos, Coordsys mypos_dt);
				/// Update all auxiliary data of the rigid body, at given time and state
	void UpdateStateTime (Coordsys mypos, Coordsys mypos_dt, double mytime);
				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..), at given time and state
	void Update (Coordsys mypos, Coordsys mypos_dt, double mytime);


				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..), at given time
	virtual void Update (double mytime);
				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..)
	virtual void Update ();


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
	int StreamOUTall (ChStreamOutBinary& m_file);
				/// Read data, including child markers and child forces
	int StreamINall  (ChStreamInBinary&  m_file);


				/// Method to allow serialization of transient data in ascii,
				/// as a readable item, for example   "chrono::GetLog() << myobject;"
	void StreamOUT(ChStreamOutAscii& mstream);

	int  StreamOUTall  (ChStreamOutAscii& mstream);

};




const int BODY_DOF = 6;  ///< degrees of freedom of body in 3d space
const int BODY_QDOF = 7; ///< degrees of freedom with quaternion rotation state
const int BODY_ROT = 3;  ///< rotational dof in Newton dynamics


typedef ChSharedPtr<ChBody> ChSharedBodyPtr;



} // END_OF_NAMESPACE____


#endif
