#ifndef CHC_OBBCOLLIDER
#define CHC_OBBCOLLIDER

//////////////////////////////////////////////////
//  
//   ChCOBBcollider.h
//
//   Detects collisions between a pair of models 
//   based on OBBs
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
#include "ChCOBBTree.h"
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

class ChOBBcollider : public ChNarrowPhaseCollider
{
public:

  ChOBBcollider();
  ~ChOBBcollider();

			/// Perform collision detection between two OBB-based
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
	
	void CollideRecurse(
               ChMatrix33<>& R, Vector& T, 
               ChOBBTree *o1, int b1, 
               ChOBBTree *o2, int b2, 
			   eCollMode flag);
};








} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






