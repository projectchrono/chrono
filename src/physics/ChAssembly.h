#ifndef CHASSEMBLY_H
#define CHASSEMBLY_H

//////////////////////////////////////////////////
//
//   ChAssembly.h
//
//   Class for a sub-assembly of rigid bodies (with constraints).
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
#include "physics/ChLinksAll.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpConstraint.h"



namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


///
/// Class for a collection of rigid bodies with some constraints, that should always be cosidered together.
/// A rigid body is an entity which can move in 3D space, and the constraints are ChLink objects.
/// The objects have mass and inertia properties. A shape can also
/// be associated to the bodies, for collision detection.
///

class ChApi ChAssembly : public ChPhysicsItem , public ChFrameMoving<double> {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChAssembly,ChPhysicsItem);

protected:
	

						// list of joints (links)
	std::list<ChLink*>   linklist; 

						// list of rigid bodies
	std::vector<ChBody*> bodylist;

private:
			//
	  		// DATA
			//

	bool do_collide;
	bool do_limit_speed;

	float max_speed;	// limit on linear speed (useful for VR & videagames)
	float max_wvel;		// limit on angular vel. (useful for VR & videagames)


public:

			//
	  		// CONSTRUCTORS
			//

				/// Build an assembly.
	ChAssembly ();
				/// Destructor
	~ChAssembly ();

				/// Copy from another ChAssembly. 
				/// NOTE: all settings of the body are copied, but the
				/// child hierarchy of ChForces and ChMarkers (if any) are NOT copied.
	void Copy(ChAssembly* source);


			//
	  		// FLAGS
			//


				/// Enable/disable the collision for this cluster of particles.
				/// After setting ON, remember RecomputeCollisionModel()
				/// before anim starts (it is not automatically
				/// recomputed here because of performance issues.)
	void  SetCollide (bool mcoll);
	bool  GetCollide() {return do_collide;}

				/// Trick. Set the maximum linear speed (beyond this limit it will
				/// be clamped). This is useful in virtual reality and real-time
				/// simulations, because it reduces the risk of bad collision detection.
				/// The realism is limited, but the simulation is more stable.
	void SetLimitSpeed    (bool mlimit) { do_limit_speed = mlimit;};
	bool GetLimitSpeed()  {return do_limit_speed;};
		

			//
	  		// FUNCTIONS
			//

				/// Removes all bodies/marker/forces/links/contacts,
				/// also resets timers and events.
	void Clear();


				/// Searches a marker from its unique ID -OBSOLETE
	ChMarker* SearchMarker(int markID);

				/// Given inserted markers and links, restores the
				/// pointers of links to markers given the information
				/// about the marker IDs.
	void Reference_LM_byID();

				/// Number of coordinates of the rigid body =6 (internally, 3+4=7 coords are used 
				/// since quaternions are used for large rotations, but local coords -ex. w&v velocity- are 6)
	virtual int GetDOF  ();
				/// Get the number of scalar constraints, if any, in this item 
	virtual int GetDOC  ()   {return GetDOC_c()+GetDOC_d();}
				/// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
	virtual int GetDOC_c  ();
				/// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
	virtual int GetDOC_d  ();


			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

				/// Sets the 'fb' part of the encapsulated ChLcpVariablesBodyOwnMass to zero.
	virtual void VariablesFbReset();

				/// Adds the current forces applied to body (including gyroscopic torque) in
				/// encapsulated ChLcpVariablesBody, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariablesBody with the 
				/// current value of body speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function sems unuseful, however the LCP solver has an option 'add_Mq_to_f', that
				/// takes [M]*qb and add to the 'fb' term before starting (this is often needed in
				/// the Anitescu time stepping method, for instance); this explains the need of this method..
	virtual void VariablesQbLoadSpeed();


				/// Fetches the body speed (both linear and angular) from the
				/// 'qb' part of the ChLcpVariablesBody (does not updates the full body&markers state)
				/// and sets it as the current body speed.
				/// If 'step' is not 0, also computes the approximate acceleration of
				/// the body using backward differences, that is  accel=(new_speed-old_speed)/step.
				/// Mostly used after the LCP provided the solution in ChLcpVariablesBody .
	virtual void VariablesQbSetSpeed(double step=0.);

				/// Increment body position by the 'qb' part of the ChLcpVariablesBody,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
				/// If qb is a speed, this behaves like a single step of 1-st order
				/// numerical integration (Eulero integration).
				/// Does not automatically update markers & forces.
	virtual void VariablesQbIncrementPosition(double step);


				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver)
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);

				/// Tell to a system descriptor that there are contraints of type
				/// ChLcpConstraint in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);

				/// Sets to zero the known term (b_i) of encapsulated ChLcpConstraints
	virtual void ConstraintsBiReset();

				/// Adds the current C (constraint violation) to the known term (b_i) of
				/// encapsulated ChLcpConstraints
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);

				/// Adds the current Ct (partial t-derivative, as in C_dt=0-> [Cq]*q_dt=-Ct)
				/// to the known term (b_i) of encapsulated ChLcpConstraints
	virtual void ConstraintsBiLoad_Ct(double factor=1.);

				/// Adds the current Qc (the vector of C_dtdt=0 -> [Cq]*q_dtdt=Qc )
				/// to the known term (b_i) of encapsulated ChLcpConstraints
	virtual void ConstraintsBiLoad_Qc(double factor=1.);

				/// Adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
				/// of the ChLcpVariables referenced by encapsulated ChLcpConstraints
	virtual void ConstraintsFbLoadForces(double factor=1.);

				/// Adds the current jacobians in encapsulated ChLcpConstraints
	virtual void ConstraintsLoadJacobians();

				/// Fills the solution of the constraint (the lagrangian multiplier l_i)
				/// with an initial guess, if any. This can be used for warm-starting the 
				/// LCP solver before starting the solution of the SPEED problem, if some 
				/// approximate solution of constraint impulese l_i already exist (ex. cached
				/// from a previous LCP execution)
				/// When implementing this in sub classes, if no guess is available, set l_i as 0.
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();

				/// As ConstraintsLiLoadSuggestedSpeedSolution(), but for the POSITION problem.
	virtual void ConstraintsLiLoadSuggestedPositionSolution();

				/// After the LCP solver has found the l_i lagangian multipliers for the
				/// SPEED problem, this function will be called to store the solutions in a
				/// cache (to be implemented in ChLink sub classes) so that it can be later retrieved with
				/// ConstraintsLiLoadSuggestedSpeedSolution(). If you do not plan to implement a l_i cache,
				/// just do not override this function in child classes and do nothing.
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();

				/// As ConstraintsLiFetchSuggestedSpeedSolution(), but for the POSITION problem.
	virtual void ConstraintsLiFetchSuggestedPositionSolution();

				/// Fetches the reactions from the lagrangian multiplier (l_i)
				/// of encapsulated ChLcpConstraints. 
				/// Mostly used after the LCP provided the solution in ChLcpConstraints.
				/// Also, should convert the reactions obtained from dynamical simulation,
				/// from link space to intuitive react_force and react_torque.
	virtual void ConstraintsFetch_react(double factor=1.);
				

			   // Other functions

				/// Set no speed and no accelerations (but does not change the position)
	void SetNoSpeedNoAcceleration();


				/// Synchronize coll.model coordinate and bounding box to the position of the body.
	virtual void SyncCollisionModels();
	virtual void AddCollisionModelsToSystem();
	virtual void RemoveCollisionModelsFromSystem();


				/// Get the entire AABB axis-aligned bounding box of the object,
				/// as defined by the collision model (if any).
	virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax);

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

				/// When this function is called, the speed of particles is clamped
				/// into limits posed by max_speed and max_wvel  - but remember to
				/// put the body in the SetLimitSpeed(true) mode.
	void ClampSpeed();


			//
			// DATABASE HANDLING.
			//
			// To attach/remove items (rigid bodies, links, etc.) you must use 
			// shared pointer, so that you don't need to care about item deletion, 
			// which will be automatic when needed.
			// Please don't add the same item multiple times; also, don't remove
			// items which haven't ever been added.
			// NOTE! After adding/removing items to the system, you should call Update() !

				/// Get the number of particles
	unsigned int GetNbodies() {return bodylist.size();}

				/// Attach a body to this system. Must be an object of exactly ChBody class.
	void AddBody (ChSharedPtr<ChBody> newbody);
				/// Attach a link to this system. Must be an object of ChLink or derived classes.
	void AddLink (ChSharedPtr<ChLink> newlink);
	void AddLink (ChLink* newlink);  // _internal use

				/// Remove a body from this system.
	void RemoveBody (ChSharedPtr<ChBody> mbody); 
				/// Remove a link from this system (faster version, mostly internal use)
	std::list<ChLink*>::iterator RemoveLinkIter(std::list<ChLink*>::iterator& mlinkiter); 
				/// Remove a link from this system.
	void RemoveLink (ChSharedPtr<ChLink> mlink); 

				/// Remove all bodies from this system.
	void RemoveAllBodies();
				/// Remove all links from this system.
	void RemoveAllLinks();


			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..), at given time
	virtual void Update (double mytime);
				/// Update all auxiliary data of the rigid body and of
				/// its children (markers, forces..)
	virtual void Update ();


			//
			// STREAMING
			//
				
				/// Method to deserialize only the state (position, speed)
	virtual void StreamINstate(ChStreamInBinary& mstream);
				/// Method to serialize only the state (position, speed)
	virtual void StreamOUTstate(ChStreamOutBinary& mstream);	

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



typedef ChSharedPtr<ChAssembly> ChAssemblyPtr;



} // END_OF_NAMESPACE____


#endif
