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

#ifndef CHCONTACT_H
#define CHCONTACT_H

///////////////////////////////////////////////////
//
//   ChContact.h
//
//   Classes for enforcing constraints (contacts)
//   created by collision detection.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChFrame.h"
#include "core/ChVectorDynamic.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionModel.h"
#include "physics/ChMaterialCouple.h"


namespace chrono
{


///
/// Class representing an unilateral contact constraint
/// between two 6DOF rigid bodies.
///

class ChApi ChContact {

protected:
				//
	  			// DATA
				//
	collision::ChCollisionModel* modA;	///< model A
	collision::ChCollisionModel* modB;  ///< model B

	ChVector<> p1;			///< max penetration point on geo1, after refining, in abs space
	ChVector<> p2;			///< max penetration point on geo2, after refining, in abs space
	ChVector<double> normal;	///< normal, on surface of master reference (geo1)

							///< the plane of contact (X is normal direction)
	ChMatrix33<double> contact_plane;
	
	double norm_dist;	    ///< penetration distance (negative if going inside) after refining

	float* reactions_cache; ///< points to an array[3] of N,U,V reactions which might be stored in a persistent contact manifold in the collision engine

							// The three scalar constraints, to be feed into the 
							// system solver. They contain jacobians data and special functions.
	ChLcpConstraintTwoContactN  Nx;
	ChLcpConstraintTwoFrictionT Tu;
	ChLcpConstraintTwoFrictionT Tv; 

	ChVector<> react_force;

	double compliance;
	double complianceT;
	double restitution;
	double dampingf;

public:
				//
	  			// CONSTRUCTORS
				//

	ChContact ();

	ChContact (			collision::ChCollisionModel* mmodA,	///< model A
						collision::ChCollisionModel* mmodB,	///< model B
						const ChLcpVariablesBody* varA, ///< pass A vars
						const ChLcpVariablesBody* varB, ///< pass B vars
						const ChFrame<>* frameA,		///< pass A frame
						const ChFrame<>* frameB,		///< pass B frame
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						ChMaterialCouple& mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				);

	virtual ~ChContact ();


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
						const ChVector<>& vpA,			///< pass coll.point on A, in absolute coordinates
						const ChVector<>& vpB,			///< pass coll.point on B, in absolute coordinates
						const ChVector<>& vN, 			///< pass coll.normal, respect to A, in absolute coordinates
						double mdistance,				///< pass the distance (negative for penetration)
						float* mreaction_cache,			///< pass the pointer to array of N,U,V reactions: a cache in contact manifold. If not available=0.
						ChMaterialCouple& mmaterial		///< pass the reference to the material with friction, stiffness, etc.
				);

					/// Get the contact coordinate system, expressed in absolute frame.
					/// This represents the 'main' reference of the link: reaction forces 
					/// are expressed in this coordinate system. Its origin is point P2.
					/// (It is the coordinate system of the contact plane and normal)
	virtual ChCoordsys<> GetContactCoords();

					/// Returns the pointer to a contained 3x3 matrix representing the UV and normal
					/// directions of the contact. In detail, the X versor (the 1s column of the 
					/// matrix) represents the direction of the contact normal.
	ChMatrix33<double>* GetContactPlane() {return &contact_plane;};


					/// Get the contact point 1, in absolute coordinates
	virtual ChVector<> GetContactP1() {return p1; };

					/// Get the contact point 2, in absolute coordinates
	virtual ChVector<> GetContactP2() {return p2; };

					/// Get the contact normal, in absolute coordinates
	virtual ChVector<double> GetContactNormal()  {return normal; };

					/// Get the contact distance
	virtual double	   GetContactDistance()  {return norm_dist; };
	
					/// Get the contact force, if computed, in contact coordinate system
	virtual ChVector<> GetContactForce() {return react_force; };

					/// Get the contact friction coefficient
	virtual double GetFriction() {return Nx.GetFrictionCoefficient(); };
					/// Set the contact friction coefficient
	virtual void SetFriction(double mf) { Nx.SetFrictionCoefficient(mf); };

					/// Get the collision model A, with point P1
	virtual collision::ChCollisionModel* GetModelA() {return this->modA;}
					/// Get the collision model B, with point P2
	virtual collision::ChCollisionModel* GetModelB() {return this->modB;}

				//
				// UPDATING FUNCTIONS
				//


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
