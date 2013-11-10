//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONVEYOR_H
#define CHCONVEYOR_H

//////////////////////////////////////////////////
//
//   ChConveyor.h
//
//   Class for conveyor belt (approximated by 
//   parallelepiped with moving surface)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "physics/ChBody.h"
#include "physics/ChLinkLock.h"



namespace chrono
{




///
/// Class for conveyor belt. 
/// A conveyor belt is approximated by a box collision
/// shape, where the upper surface has continuous motion
/// in X direction. No cylindrical rounding is used at the ends.
///

class ChApi ChConveyor : public ChBody 
{

	CH_RTTI(ChConveyor,ChBody);

private:
			//
	  		// DATA
			//

	double conveyor_speed;	// speed of conveyor, along the X direction of the box.
		
						// link between this body and conveyor plate 
	ChLinkLockLock* internal_link;

						// used for the conveyor plate
	ChBody* 		conveyor_plate;

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a conveyor belt, with motion along x axis
	ChConveyor (double xlength = 1, double ythick = 0.1, double zwidth =0.5);
				/// Destructor
	~ChConveyor ();

				/// Copy from another ChChConveyor. 
	void Copy(ChConveyor* source);



			//
	  		// FUNCTIONS
			//

				/// Set the speed of the conveyor belt (upper part, X direction)
	void   SetConveyorSpeed(double mspeed) {conveyor_speed = mspeed;}
				/// Get the speed of the conveyor belt (upper part, X direction)
	double GetConveyorSpeed() {return conveyor_speed;}


				/// Number of coordinates of the rigid body =6 (but other 6 variables are
				/// used internally for the motion of the belt surface, as a rigid 3d plate)
	virtual int GetDOF  ()   {return 6;}


			 // Override/implement LCP system functions of ChPhysicsItem
			 // (to assembly/manage data for LCP system solver)

	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor);
	virtual void VariablesFbReset();
	virtual void VariablesFbLoadForces(double factor=1.);
	virtual void VariablesQbLoadSpeed();
	virtual void VariablesFbIncrementMq();
	virtual void VariablesQbSetSpeed(double step=0.);
	virtual void VariablesQbIncrementPosition(double step);

	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsBiLoad_Qc(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsLiLoadSuggestedSpeedSolution();
	virtual void ConstraintsLiLoadSuggestedPositionSolution();
	virtual void ConstraintsLiFetchSuggestedSpeedSolution();
	virtual void ConstraintsLiFetchSuggestedPositionSolution();
	virtual void ConstraintsFetch_react(double factor=1.);



			   // Other functions

	virtual bool GetCollide() {return true;};
	virtual void SyncCollisionModels();
	virtual void AddCollisionModelsToSystem();
	virtual void RemoveCollisionModelsFromSystem();

	

			//
			// UPDATE FUNCTIONS
			//

				/// Update all auxiliary data of the conveyor at given time
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




} // END_OF_NAMESPACE____


#endif
