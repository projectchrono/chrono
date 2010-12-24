#ifndef CHSHAFTSBODY_H
#define CHSHAFTSBODY_H

//////////////////////////////////////////////////
//
//   ChShaftsBody.h
//
//   Class for creating a constraint between a 3D
//   ChBody object and a 1D ChShaft object. A rotation
//   axis must be specified (to tell along which direction
//   she shaft inertia and rotation affects the body).
//   This constraint is useful, for example, when you have modeled a
//   3D car using ChBody items and a 1D powertrain (gears,
//   differential, etc.) using ChShaft objects: you can connect
//   the former (at least, the wheels) to the latter using
//   this constraint.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChBody.h"
#include "physics/ChShaft.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"



namespace chrono
{

// Forward references (for parent hierarchy pointer)

class ChShaft;
class ChBody;

/// Class for creating a constraint between a 3D
/// ChBody object and a 1D ChShaft object. A rotation
/// axis must be specified (to tell along which direction
/// she shaft inertia and rotation affects the body).
/// This constraint is useful, for example, when you have modeled a
/// 3D car using ChBody items and a 1D powertrain (gears,
/// differential, etc.) using ChShaft objects: you can connect
/// the former (at least, the wheels) to the latter using
/// this constraint.

class ChApi ChShaftsBody : public ChPhysicsItem {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChShaftsBody,ChPhysicsItem);

private:
			//
	  		// DATA
			//

	double torque_react;					
	
						// used as an interface to the LCP solver.
	ChLcpConstraintTwoGeneric constraint;

	float cache_li_speed;	// used to cache the last computed value of multiplier (solver warm starting)
	float cache_li_pos;		// used to cache the last computed value of multiplier (solver warm starting)	

	ChShaft* shaft;
	ChBody*  body;

	ChVector<> shaft_dir;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a shaft.
	ChShaftsBody ();
				/// Destructor
	~ChShaftsBody ();

				/// Copy from another ChShaftsPlanetary. 
	void Copy(ChShaftsBody* source);


			//
	  		// FLAGS
			//

			//
	  		// FUNCTIONS
			//

				/// Number of scalar costraints 
	virtual int GetDOC_c  () {return 1;}


			// Override/implement LCP system functions of ChPhysicsItem
			// (to assembly/manage data for LCP system solver

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);


			   // Other functions

				/// Use this function after object creation, to initialize it, given  
				/// the 1D shaft and 3D body to join. 
				/// Each item must belong to the same ChSystem. 
	virtual int Initialize(ChSharedShaftPtr& mshaft, ///< shaft to join 
						   ChSharedBodyPtr&  mbody,  ///< body to join 
						   ChVector<>& mdir			 ///< the direction of the shaft on 3D body (applied on COG: pure torque)
						   );

				/// Get the shaft
	ChShaft* GetShaft() {return shaft;}
				/// Get the body
	ChBody*  GetBody() {return body;}

				/// Set the direction of the shaft respect to 3D body, as a 
				/// normalized vector expressed in the coordinates of the body.
				/// The shaft applies only torque, about this axis.
	void  SetShaftDirection(ChVector<> md) {shaft_dir = Vnorm(md);}

				/// Get the direction of the shaft respect to 3D body, as a 
				/// normalized vector expressed in the coordinates of the body. 
	ChVector<>  GetShaftDirection() {return shaft_dir;}


				/// Get the reaction torque considered as applied to ChShaft.
	double GetTorqueReactionOnShaft() {return -(torque_react);}

				/// Get the reaction torque considered as applied to ChBody.
	ChVector<> GetTorqueReactionOnBody() {return (shaft_dir*torque_react);}

	
			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the gear transmission at given time
	virtual void Update (double mytime);
	


			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

};



typedef ChSharedPtr<ChShaftsBody> ChSharedShaftsBodyPtr;



} // END_OF_NAMESPACE____


#endif
