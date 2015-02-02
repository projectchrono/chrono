//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCONTACTROLLING_H
#define CHCONTACTROLLING_H

///////////////////////////////////////////////////
//
//   ChContactRolling.h
//
//   Class for enforcing constraints (contacts) with
//   friction and rolling resistance.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChContact.h"
#include "lcp/ChLcpConstraintTwoRollingN.h"

namespace chrono
{


///
/// Class representing an unilateral contact constraint, used by
/// ChContactContainer, that has also rolling friction. 
///

class ChApi ChContactRolling : public ChContact {

protected:
				//
	  			// DATA
				//
	ChLcpConstraintTwoRollingN   Rx;
	ChLcpConstraintTwoRollingT   Ru;
	ChLcpConstraintTwoRollingT   Rv; 

	ChVector<> react_torque;

	float complianceRoll;
	float complianceSpin;

public:
				//
	  			// CONSTRUCTORS
				//

	ChContactRolling ();

	ChContactRolling (	collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		///< pass A frame
						const ChFrame<>* frameB,		///< pass B frame
						const ChVector<>& vpA,			///< pass coll.point on A
						const ChVector<>& vpB,			///< pass coll.point on B
						const ChVector<>& vN, 			///< pass coll.normal, respect to A
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						ChMaterialCouple& mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				);

	virtual ~ChContactRolling ();


				//
	  			// FUNCTIONS
				//

					/// Initialize again this constraint.
	virtual void Reset(	collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		///< pass A frame
						const ChFrame<>* frameB,		///< pass B frame
						const ChVector<>& vpA,			///< pass coll.point on A
						const ChVector<>& vpB,			///< pass coll.point on B
						const ChVector<>& vN, 			///< pass coll.normal, respect to A
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						ChMaterialCouple& mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				);

	
					/// Get the contact force, if computed, in contact coordinate system
	virtual ChVector<> GetContactTorque() {return react_torque; };

					/// Get the contact rolling friction coefficient
	virtual float GetRollingFriction() {return Rx.GetRollingFrictionCoefficient(); };
					/// Set the contact rolling friction coefficient
	virtual void SetRollingFriction(float mf) { Rx.SetRollingFrictionCoefficient(mf); };

					/// Get the contact spinning friction coefficient
	virtual float GetSpinningFriction() {return Rx.GetSpinningFrictionCoefficient(); };
					/// Set the contact spinning friction coefficient
	virtual void SetSpinningFriction(float mf) { Rx.SetSpinningFrictionCoefficient(mf); };


				//
				// UPDATING FUNCTIONS
				//

	// warning, following are not virtual, for optimization

	void ContIntLoadResidual_CqL(
					const unsigned int off_L,	 ///< offset in L multipliers
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
					const ChVectorDynamic<>& L,  ///< the L vector 
					const double c				 ///< a scaling factor
					);
	void ContIntLoadConstraint_C(
					const unsigned int off_L,	 ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
					);
	void ContIntToLCP(
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
					const ChVectorDynamic<>& Qc
					) ;
	void ContIntFromLCP(
					const unsigned int off_L,			///< offset in L
					ChVectorDynamic<>& L
					);

	void  InjectConstraints(ChLcpSystemDescriptor& mdescriptor);

	void  ConstraintsBiReset();

	void  ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);

	void  ConstraintsFetch_react(double factor);

	void  ConstraintsLiLoadSuggestedSpeedSolution();

	void  ConstraintsLiLoadSuggestedPositionSolution();

	void  ConstraintsLiFetchSuggestedSpeedSolution();

	void  ConstraintsLiFetchSuggestedPositionSolution();

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
