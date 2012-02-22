#ifndef CHPHYSICSITEM_H
#define CHPHYSICSITEM_H

///////////////////////////////////////////////////
//
//   ChPhysicsItem.h
//
//
//   Base class for items that can contain objects
//   of ChLcpVariables or ChLcpConstraints, such as 
//   rigid bodies, mechanical joints, etc.
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChObject.h"
#include "assets/ChAsset.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionModel.h"


namespace chrono
{

// Forward references
class ChSystem;


/// Base class for items that can contain objects
/// of ChLcpVariables or ChLcpConstraints, such as 
/// rigid bodies, mechanical joints, etc.


class ChApi ChPhysicsItem :  public ChObj {

	CH_RTTI(ChPhysicsItem,ChObj);

protected:

				//
	  			// DATA
				//

	ChSystem *system;	  // parent system

	std::vector< ChSharedPtr<ChAsset> > assets;

public:
				//
	  			// CONSTRUCTORS
				//
	ChPhysicsItem () { system = 0;};
	virtual ~ChPhysicsItem () {}; 
	virtual void Copy(ChPhysicsItem* source);

public:
		//
	  	// FUNCTIONS
		//



				/// Get the pointer to the parent ChSystem()
	ChSystem* GetSystem () { return system;}

				/// Set the pointer to the parent ChSystem()
	virtual void SetSystem (ChSystem* m_system) {system= m_system;}

				/// Access to the list of optional assets.
	std::vector< ChSharedPtr<ChAsset> >& GetAssets () { return this->assets;}
	ChSharedPtr<ChAsset> GetAssetN (unsigned int num) { if (num<assets.size()) return assets[num]; else {ChSharedPtr<ChAsset> none; return none;};}

		// --- INTERFACES --- 
		// inherited classes might/should implement 
		// some of the following functions.


			// Collisions - override these in child classes if needed
			// 

				/// Tell if the object is subject to collision.
				/// Only for interface; child classes may override this, using internal flags.
	virtual bool GetCollide() {return false;};

				/// If this physical item contains one or more collision models,	
				/// sinchronize their coordinates and bounding boxes to the state of the item.
	virtual void SyncCollisionModels() {};

				/// If this physical item contains one or more collision models,	
				/// add them to the system's collision engine.
	virtual void AddCollisionModelsToSystem() {};

				/// If this physical item contains one or more collision models,	
				/// remove them from the system's collision engine.
	virtual void RemoveCollisionModelsFromSystem() {};

			// Functions used by domain decomposition

				/// Get the entire AABB axis-aligned bounding box of the object.
				/// The AABB must enclose the collision models, if any.
				/// By default is infinite AABB.
				/// Should be overridden by child classes.
	virtual void GetTotalAABB(ChVector<>& bbmin, ChVector<>& bbmax);

				/// Get a symbolic 'center' of the object. By default this 
				/// function returns the center of the AABB.
				/// It could be overridden by child classes, anyway it must 
				/// always get a point that must be inside AABB.
	virtual void GetCenter(ChVector<>& mcenter);

				/// Method to deserialize only the state (position, speed)
				/// Must be implemented by child classes. 
	virtual void StreamINstate(ChStreamInBinary& mstream) {};
				/// Method to serialize only the state (position, speed)
				/// Must be implemented by child classes. 
	virtual void StreamOUTstate(ChStreamOutBinary& mstream) {};				


			// UPDATING  - child classes may implement these functions
			//

				/// This is an important function, which is called by the 
				/// owner ChSystem at least once per integration step.
				/// It may update all auxiliary data of the item, such as
				/// matrices if any, etc., depending on the current coordinates.
				/// The inherited classes, for example the ChLinkMask, often
				/// implement specialized versions of this Update(time) function,
				/// because they might need to update inner states, forces, springs, etc.
				/// This base version, by default, simply updates the item's time.
	virtual void Update (double mytime) { this->ChTime = mytime; };

				/// As above, but does not require updating of time-dependent 
				/// data. By default, calls Update(mytime) using item's current time. 
	virtual void Update() {Update(this->ChTime);}


				/// Set zero speed (and zero accelerations) in state, without changing the position.
				/// Child classes should impement this function if GetDOF() > 0.
				/// It is used by owner ChSystem for some static analysis.
	void SetNoSpeedNoAcceleration() {};


			// STATISTICS  - override these in child classes if needed
			// 

				/// Get the number of scalar coordinates (variables), if any, in this item 
	virtual int GetDOF  ()   {return 0;}
				/// Get the number of scalar constraints, if any, in this item 
	virtual int GetDOC  ()   {return GetDOC_c()+GetDOC_d();}
				/// Get the number of scalar constraints, if any, in this item (only bilateral constr.)
	virtual int GetDOC_c  () {return 0;}
				/// Get the number of scalar constraints, if any, in this item (only unilateral constr.)
	virtual int GetDOC_d  () {return 0;}


			// LCP SYSTEM FUNCTIONS   
			//
			// These are the functions that are used to manage ChLcpConstraint and/or ChLcpVariable 
			// objects that are sent to the system solver.
			// The children classes, inherited from ChPhysicsItem, can implement them (by default,
			// the base ChPhysicsItem does not introduce any variable nor any constraint).

				/// Sets the 'fb' part (the known term) of the encapsulated ChLcpVariables to zero.
	virtual void VariablesFbReset() {};

				/// Adds the current forces (applied to item) into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.) {};

				/// Initialize the 'qb' part of the ChLcpVariables with the 
				/// current value of speeds. Note: since 'qb' is the unknown of the LCP, this
				/// function sems unuseful, however the LCP solver has an option 'add_Mq_to_f', that
				/// takes [M]*qb and add to the 'fb' term before starting (this is often needed in
				/// the Anitescu time stepping method, for instance); this explains the need of this method..
	virtual void VariablesQbLoadSpeed() {};

				/// Fetches the item speed (ex. linear and angular vel.in rigid bodies) from the
				/// 'qb' part of the ChLcpVariables and sets it as the current item speed.
				/// If 'step' is not 0, also should compute the approximate acceleration of
				/// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
				/// Mostly used after the LCP provided the solution in ChLcpVariables.
	virtual void VariablesQbSetSpeed(double step=0.) {};

				/// Increment item positions by the 'qb' part of the ChLcpVariables,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
				/// If qb is a speed, this behaves like a single step of 1-st order
				/// numerical integration (Eulero integration).
	virtual void VariablesQbIncrementPosition(double step) {};

				/// Tell to a system descriptor that there are variables of type
				/// ChLcpVariables in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor) {};



				/// Tell to a system descriptor that there are contraints of type
				/// ChLcpConstraint in this object (for further passing it to a LCP solver)
				/// Basically does nothing, but maybe that inherited classes may specialize this.
	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {};

				/// Sets to zero the known term (b_i) of encapsulated ChLcpConstraints
	virtual void ConstraintsBiReset() {};

				/// Adds the current C (constraint violation) to the known term (b_i) of
				/// encapsulated ChLcpConstraints
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false) {};

				/// Adds the current Ct (partial t-derivative, as in C_dt=0-> [Cq]*q_dt=-Ct)
				/// to the known term (b_i) of encapsulated ChLcpConstraints
	virtual void ConstraintsBiLoad_Ct(double factor=1.) {};

				/// Adds the current Qc (the vector of C_dtdt=0 -> [Cq]*q_dtdt=Qc )
				/// to the known term (b_i) of encapsulated ChLcpConstraints
	virtual void ConstraintsBiLoad_Qc(double factor=1.) {};

				/// Adds the current link-forces, if any, (caused by springs, etc.) to the 'fb' vectors
				/// of the ChLcpVariables referenced by encapsulated ChLcpConstraints
	virtual void ConstraintsFbLoadForces(double factor=1.) {};

				/// Adds the current jacobians in encapsulated ChLcpConstraints
	virtual void ConstraintsLoadJacobians() {};

				/// Fills the solution of the constraint (the lagrangian multiplier l_i)
				/// with an initial guess, if any. This can be used for warm-starting the 
				/// LCP solver before starting the solution of the SPEED problem, if some 
				/// approximate solution of constraint impulese l_i already exist (ex. cached
				/// from a previous LCP execution)
				/// When implementing this in sub classes, if no guess is available, set l_i as 0.
	virtual void ConstraintsLiLoadSuggestedSpeedSolution() {};

				/// As ConstraintsLiLoadSuggestedSpeedSolution(), but for the POSITION problem.
	virtual void ConstraintsLiLoadSuggestedPositionSolution() {};

				/// After the LCP solver has found the l_i lagangian multipliers for the
				/// SPEED problem, this function will be called to store the solutions in a
				/// cache (to be implemented in ChLink sub classes) so that it can be later retrieved with
				/// ConstraintsLiLoadSuggestedSpeedSolution(). If you do not plan to implement a l_i cache,
				/// just do not override this function in child classes and do nothing.
	virtual void ConstraintsLiFetchSuggestedSpeedSolution() {};

				/// As ConstraintsLiFetchSuggestedSpeedSolution(), but for the POSITION problem.
	virtual void ConstraintsLiFetchSuggestedPositionSolution() {};

				/// Fetches the reactions from the lagrangian multiplier (l_i)
				/// of encapsulated ChLcpConstraints. 
				/// Mostly used after the LCP provided the solution in ChLcpConstraints.
				/// Also, should convert the reactions obtained from dynamical simulation,
				/// from link space to intuitive react_force and react_torque.
	virtual void ConstraintsFetch_react(double factor=1.) {};


			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);


};







} // END_OF_NAMESPACE____

#endif
