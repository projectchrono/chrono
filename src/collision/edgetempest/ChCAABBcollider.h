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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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

class ChAABBcollider : public ChNarrowPhaseCollider
{
public:

  ChAABBcollider();
  ~ChAABBcollider();

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
               ChAABBTree *o1, int b1, 
               ChAABBTree *o2, int b2, 
			   eCollMode flag);
};








} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif






