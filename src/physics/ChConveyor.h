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

				/// Access the internal body used as the moving belt (a plate with const.vel.)
	ChBody* GetPlate() {return conveyor_plate;} 
	

				/// Number of coordinates: this contains an auxiliary body, so it is 14 (with quaternions for rotations) 
	virtual int GetDOF  () {return 7+7;}
				/// Number of coordinates of the particle cluster, 2x6 because derivatives es. angular vel.
	virtual int GetDOF_w() {return 6+6;}
				/// Get the number of scalar constraints. In this case, a lock constraint is embedded, so:
	virtual int GetDOC_c  () {return 6;}


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
