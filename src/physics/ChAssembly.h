//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "core/ChShared.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChForce.h"
#include "physics/ChMarker.h"
#include "physics/ChLinksAll.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpConstraint.h"



namespace chrono
{


// Forward references (for parent hierarchy pointer)

class ChSystem;


///
/// Class for a collection of rigid bodies with some constraints, that should always be cosidered together.
/// A rigid body is an entity which can move in 3D space, and the constraints are ChLink objects.
/// The objects have mass and inertia properties. A shape can also
/// be associated to the bodies, for collision detection.
///

class ChApi ChAssembly : public ChPhysicsItem {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChAssembly,ChPhysicsItem);

protected:
	

						// list of joints (links)
	std::vector<ChLink*>   linklist; 

						// list of rigid bodies
	std::vector<ChBody*> bodylist;

	int nbodies;		// number of bodies (currently active)
	int nlinks;			// number of links
	int ncoords;		// number of scalar coordinates (including 4th dimension of quaternions) for all active bodies
	int ndoc;			// number of scalar costraints (including constr. on quaternions)
	int nsysvars;		// number of variables (coords+lagrangian mult.), i.e. = ncoords+ndoc  for all active bodies
	int ncoords_w;		// number of scalar coordinates when using 3 rot. dof. per body;  for all active bodies
	int ndoc_w;			// number of scalar costraints  when using 3 rot. dof. per body;  for all active bodies
	int nsysvars_w;		// number of variables when using 3 rot. dof. per body; i.e. = ncoords_w+ndoc_w
	int ndof;			// number of degrees of freedom, = ncoords-ndoc =  ncoords_w-ndoc_w ,
	int ndoc_w_C;		// number of scalar costraints C, when using 3 rot. dof. per body (excluding unilaterals)
	int ndoc_w_D;		// number of scalar costraints D, when using 3 rot. dof. per body (only unilaterals)
	int nbodies_sleep;  // number of bodies that are sleeping
	int nbodies_fixed;  // number of bodies that are fixed

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

				/// Set the pointer to the parent ChSystem()
	void SetSystem (ChSystem* m_system);


				/// Removes all bodies/marker/forces/links/contacts,
				/// also resets timers and events.
	void Clear();


				/// Searches a marker from its unique ID -OBSOLETE
	ChSharedPtr<ChMarker> SearchMarker(int markID);

				/// Given inserted markers and links, restores the
				/// pointers of links to markers given the information
				/// about the marker IDs.
	void Reference_LM_byID();

					/// Gets the number of links .
	int GetNlinks() {return nlinks;}
				/// Gets the number of coordinates (considering 7 coords for rigid bodies because of the 4 dof of quaternions)
	int GetNcoords() {return ncoords;}
				/// Gets the number of degrees of freedom of the system.
	int GetNdof() {return ndof;}
				/// Gets the number of scalar constraints added to the system, including constraints on quaternion norms
	int GetNdoc() {return ndoc;}
				/// Gets the number of system variables (coordinates plus the constraint multipliers, in case of quaternions)
	int GetNsysvars() {return nsysvars;}
				/// Gets the number of coordinates (considering 6 coords for rigid bodies, 3 transl.+3rot.)
	int GetNcoords_w() {return ncoords_w;}
				/// Gets the number of scalar constraints added to the system		
	int GetNdoc_w() {return ndoc_w;}
				/// Gets the number of scalar constraints added to the system (only bilaterals)
	int GetNdoc_w_C() {return ndoc_w_C;}
				/// Gets the number of scalar constraints added to the system (only unilaterals)
	int GetNdoc_w_D() {return ndoc_w_D;}
				/// Gets the number of system variables (coordinates plus the constraint multipliers)
	int GetNsysvars_w() {return nsysvars_w;}


				/// Number of coordinates of the rigid body =6 (internally, 3+4=7 coords are used 
				/// since quaternions are used for large rotations, but local coords -ex. w&v velocity- are 6)
	virtual int GetDOF  () {return GetNcoords();}
				/// Number of coordinates of the particle cluster, x6 because derivatives es. angular vel.
	virtual int GetDOF_w() {return GetNcoords_w();}
				/// Get the number of scalar constraints, if any, in this item 
	virtual int GetDOC  ()   {return GetNdoc_w();}
				/// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
	virtual int GetDOC_c  () {return GetNdoc_w_C();};
				/// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
	virtual int GetDOC_d  () {return GetNdoc_w_D();};


			//
			// STATE FUNCTIONS
			//

				// (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
	virtual void IntStateGather(const unsigned int off_x,	ChState& x,	const unsigned int off_v, ChStateDelta& v,	double& T);	
	virtual void IntStateScatter(const unsigned int off_x,	const ChState& x, const unsigned int off_v,	const ChStateDelta& v,	const double T);
	virtual void IntStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x,	const unsigned int off_v, const ChStateDelta& Dv); 
	virtual void IntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c );
	virtual void IntLoadResidual_Mv(const unsigned int off,	ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c);
	virtual void IntLoadResidual_CqL(const unsigned int off_L, ChVectorDynamic<>& R, const ChVectorDynamic<>& L, const double c);
	virtual void IntLoadConstraint_C(const unsigned int off, ChVectorDynamic<>& Qc,	const double c, bool do_clamp,	double recovery_clamp);
	virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c);
	virtual void IntToLCP(const unsigned int off_v,	const ChStateDelta& v, const ChVectorDynamic<>& R, const unsigned int off_L, const ChVectorDynamic<>& L, const ChVectorDynamic<>& Qc);
	virtual void IntFromLCP(const unsigned int off_v, ChStateDelta& v, const unsigned int off_L, ChVectorDynamic<>& L);


			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

				/// Sets the 'fb' part of the encapsulated ChLcpVariablesBodyOwnMass to zero.
	virtual void VariablesFbReset();

				/// Adds the current forces applied to body (including gyroscopic torque) in
				/// encapsulated ChLcpVariablesBody, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.);

				/// Initialize the 'qb' part of the ChLcpVariablesBody with the 
				/// current value of body speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function seems unuseful, unless used before VariablesFbIncrementMq()
	virtual void VariablesQbLoadSpeed();

				/// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
	virtual void VariablesFbIncrementMq();

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
	size_t GetNbodies() {return bodylist.size();}

				/// Attach a body to this system. Must be an object of exactly ChBody class.
	void AddBody (ChSharedPtr<ChBody> newbody);
				/// Attach a link to this system. Must be an object of ChLink or derived classes.
	void AddLink (ChSharedPtr<ChLink> newlink);
	void AddLink (ChLink* newlink);  // _internal use

				/// Remove a body from this system.
	void RemoveBody (ChSharedPtr<ChBody> mbody); 
				/// Remove a link from this system (faster version, mostly internal use)
	std::vector<ChLink*>::iterator RemoveLinkIter(std::vector<ChLink*>::iterator& mlinkiter); 
				/// Remove a link from this system.
	void RemoveLink (ChSharedPtr<ChLink> mlink); 

				/// Remove all bodies from this system.
	void RemoveAllBodies();
				/// Remove all links from this system.
	void RemoveAllLinks();

        /// Gets the list of children bodies -low level function-.
        /// NOTE: to modify this list, use the appropriate Remove..
        /// and Add.. functions.
	const std::vector<ChBody*>& Get_bodylist() const {return bodylist;}
        /// Gets the list of children links -low level function-.
        /// NOTE: to modify this list, use the appropriate Remove..
        /// and Add.. functions.
	const std::vector<ChLink*>& Get_linklist() const {return linklist;}


			//
			// UPDATE FUNCTIONS
			//

				/// Counts the number of bodies and links. 
				/// Computes the offsets of object states in the global state.
	virtual void Setup();


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
