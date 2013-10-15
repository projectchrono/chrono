//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_AABBCOLLIDER_H
#define CHC_AABBCOLLIDER_H

//////////////////////////////////////////////////
//  
//   ChCAABBcollider.h
//
//   Detects collisions between a pair of collision 
//   models based on AABBs
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChCCompile.h"   
#include "collision/ChCCollisionPair.h"                           
#include "ChCAABBTree.h"
#include "ChCNarrowPhaseCollider.h"


namespace chrono 
{
namespace collision 
{


/// 
/// This class specializes the ChNarrowPhaseCollider base class, in order
/// to compute the case of collisions between two collision models
/// based on AABB trees.
///

class CHAABBcollider : public ChNarrowPhaseCollider
{
public:

  CHAABBcollider();
  ~CHAABBcollider();

	// PERFORM THE COLLISION DETECTION

			/// Perform collision detection between two AABB-based
			/// collision models (each belonging to a different rigid
			/// body, for example), given the absolute orientations and
			/// translations of the two models.
			/// The identified contacts will be inserted in the Contacts vector.
			///
  eCollSuccess ComputeCollisions(
            ChMatrix33<>& R1, Vector T1, ChCollisionTree *oc1,
            ChMatrix33<>& R2, Vector T2, ChCollisionTree *oc2,
			eCollMode flag);

private:
		// as this->>R, but absolute values plus some epsilon. Precomputed and used for AABB intersection
	ChMatrix33<> Rabs;

	void CollideRecurse(
               CHAABBTree *o1, int b1, 
               CHAABBTree *o2, int b2, 
			   eCollMode flag);
};








} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif






