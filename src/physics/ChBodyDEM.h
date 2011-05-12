#ifndef CHBODYDEM_H
#define CHBODYDEM_H

//////////////////////////////////////////////////
//
//   ChBodyDEM.h
//
//   Class for simplified rigid bodies, that is rigid moving
//   parts with mass and collision geometry 
//   which interact through DEM approach.
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
/// bodies using ChLink objects.
/// These objects have mass and inertia properties. A shape can also
/// be associated to the body, for collision detection.
///

class ChApi ChBodyDEM : public ChPhysicsItem , public ChFrameMoving<double> {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBodyDEM,ChPhysicsItem);

protected:
	
						// Pointer to the collision model, including the
						// colliding geometry .
	ChCollisionModel* collision_model;

private:
			//
	  		// DATA
			//

	int bflag;			// body-specific flags.

	ChVector<> gyro;		// The Qm gyroscopic torque, i.e. Qm= Wvel x (XInertia*Wvel)

	ChVector<> Xforce;		// The force  acting on body, applied to COG -in abs. coords-
	ChVector<> Xtorque;		// The torque acting on body  -in body rel. coords-

	float kn;	// normal spring constant for DEM contacts
	float gn;   // normal damping constant for DEM contacts
	 
						// Auxiliary, stores position/rotation once a while
						// when collision detection routines require to know
						// the last time that coll.detect. was satisfied.
	ChCoordsys<> last_coll_pos;
						

						// used to store mass matrix and coordinates, as
						// an interface to the LCP solver.
	ChLcpVariablesBodyOwnMass	variables;

	float max_speed;	// limit on linear speed (useful for VR & videagames)
	float max_wvel;		// limit on angular vel. (useful for VR & videagames)


public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a rigid body.
	ChBodyDEM ();
				/// Destructor
	~ChBodyDEM ();

				/// Copy from another ChBodyDEM. 
				/// NOTE: all settings of the body are copied, but the
				/// child hierarchy of ChForces and ChMarkers (if any) are NOT copied.
	void Copy(ChBodyDEM* source);


			//
	  		// FLAGS
			//


				/// Sets the 'fixed' state of the body. If true, it does not move
				/// respect to the absolute world, despite constraints, forces, etc.
	void SetBodyFixed (bool mev);
	bool GetBodyFixed()	   {return BFlagGet(BF_FIXED);}

				/// Enable/disable the collision for this rigid body.
				/// (After setting ON, you may need RecomputeCollisionModel()
				/// before anim starts, if you added an external object 
				/// that implements onAddCollisionGeometries(), ex. in a plugin for a CAD)
	void  SetCollide (bool mcoll);
	bool  GetCollide() {return BFlagGet(BF_COLLIDE);}

				/// Trick. Set the maximum linear speed (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations, because it reduces the risk of bad collision detection.
				/// The realism is limited, but the simulation is more stable.
	void SetLimitSpeed    (bool mlimit) { BFlagSet(BF_LIMITSPEED, mlimit);};
	bool GetLimitSpeed()  {return BFlagGet(BF_LIMITSPEED);};


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
	Coordsys GetLastCollPos () { return last_coll_pos; }
				/// Stores the current position in the last-collision-position buffer.
	void SynchronizeLastCollPos() {last_coll_pos = this->coord;}

				/// Get the entire AABB axis-aligned bounding box of the object,
				/// as defined by the collision model (if any).
	virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax);

				/// Method to deserialize only the state (position, speed)
	virtual void StreamINstate(ChStreamInBinary& mstream);
				/// Method to serialize only the state (position, speed)
	virtual void StreamOUTstate(ChStreamOutBinary& mstream);	

				/// The spring coefficient for DEM contacts.
				/// Default 392400.0
	float  GetSpringCoefficient() {return kn;}
	void   SetSpringCoefficient(float mval) {kn = mval;}
				
				/// The damping coefficient for DEM contacts.
				/// Default 420.0
	float  GetDampingCoefficient() {return gn;}
	void   SetDampingCoefficient(float mval) {gn = mval;}

				/// accumulate force on this body, or reset the force to zero
	void AccumulateForce(ChVector<> ff) {Xforce+=ff;}
	void ResetBodyForce() {Xforce = VNULL;}
	void AccumulateTorque(ChVector<> tt) {Xtorque+=tt;}
	void ResetBodyTorque() {Xtorque = VNULL;}


			//
			// DATABASE HANDLING.
			//
			// To attach/remove items (rigid bodies, links, etc.) you must use 
			// shared pointer, so that you don't need to care about item deletion, 
			// which will be automatic when needed.
			// Please don't add the same item multiple times; also, don't remove
			// items which haven't ever been added.
			// NOTE! After adding/removing items to the system, you should call Update() !


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



typedef ChSharedPtr<ChBodyDEM> ChSharedBodyDEMPtr;



} // END_OF_NAMESPACE____


#endif
