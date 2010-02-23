//////////////////////////////////////////////////
//  
//   ChCAABBcollider.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
#include <stdio.h>
#include <string.h>


#include "ChCMatVec.h"
#include "ChCGetTime.h"
#include "ChCAABBcollider.h"

#include "ChCGeometryCollider.h"



namespace chrono 
{
namespace collision 
{



ChAABBcollider::ChAABBcollider()
{
	this->Rabs.Set33Identity();
}

ChAABBcollider::~ChAABBcollider()
{

}





void ChAABBcollider::CollideRecurse(
               ChAABBTree *o1, int b1, 
               ChAABBTree *o2, int b2, 
			   eCollMode flag)
{
 
  ChAABB* box1 = o1->child(b1);
  ChAABB* box2 = o2->child(b2);

  this->num_bv_tests++;

  // first thing, see if we're overlapping

  static Vector Translation;
  Translation = Vsub (this->T, box1->To);
  Translation = Vadd (Translation, this->R.Matr_x_Vect(box2->To));

  if (!ChAABB::AABB_Overlap(this->R, this->Rabs, Translation, box1, box2)) 
	  return;
  
  // if we are, see if we test triangles next

  int l1 = box1->IsLeaf();
  int l2 = box2->IsLeaf();

  //
  // CASE TWO LEAVES
  //

  if (l1 && l2) 
  {
    this->num_geo_tests++;

    // transform the points in b2 into space of b1, then compare

	ChGeometry* mgeo1 = o1->geometries[box1->GetGeometryIndex()];
	ChGeometry* mgeo2 = o2->geometries[box2->GetGeometryIndex()];

	bool just_intersect = false;
	if (flag==ChNarrowPhaseCollider::ChC_FIRST_CONTACT)
		just_intersect = true;

	ChGeometryCollider::ComputeCollisions(*mgeo1, &this->R1, &this->T1, 
										  *mgeo2, &this->R2, &this->T2, 
										  *this, 
										  &this->R, &this->T, 
										  just_intersect);
	return;
  }

  // we dont, so decide whose children to visit next

  double sz1 = box1->GetSize();
  double sz2 = box2->GetSize();

    
  if (l2 || (!l1 && (sz1 > sz2)))
  {
    int c1 = box1->GetFirstChildIndex();
    int c2 = box1->GetSecondChildIndex();

    CollideRecurse(o1,c1,o2,b2,flag);

     if ((flag == ChC_FIRST_CONTACT) && (this->GetNumPairs() > 0)) return;

    CollideRecurse(o1,c2,o2,b2,flag);
  }
  else 
  {
    int c1 = box2->GetFirstChildIndex();
    int c2 = box2->GetSecondChildIndex();

    CollideRecurse(o1,b1,o2,c1,flag);

     if ((flag == ChC_FIRST_CONTACT) && (this->GetNumPairs() > 0)) return;

    CollideRecurse(o1,b1,o2,c2,flag);
  }
}


//
// PERFORM COLLISION DETECTION
//

ChNarrowPhaseCollider::eCollSuccess ChAABBcollider::ComputeCollisions(
            ChMatrix33<>& R1, Vector T1, ChCollisionTree *oc1,
            ChMatrix33<>& R2, Vector T2, ChCollisionTree *oc2,
            eCollMode flag)
{
  double t1 = GetTime();

  // INHERIT parent class behaviour

  if (ChNarrowPhaseCollider::ComputeCollisions(
			R1, T1, oc1,
            R2, T2, oc2,
            flag) != ChC_RESULT_OK)
				return ChC_RESULT_GENERICERROR;
 
  // Downcasting 
  ChAABBTree* o1 = (ChAABBTree*)oc1;
  ChAABBTree* o2 = (ChAABBTree*)oc2;

  // clear the stats

  this->num_bv_tests = 0;
  this->num_geo_tests = 0;
  

  // Precompute the Rabs matrix, to be used many times in AABB collisions

  const double reps = (double)1e-6;
  // Rabs = fabs(R)+eps
  Rabs(0,0) = myfabs(R.Get33Element(0,0));  Rabs(0,0) += reps;
  Rabs(0,1) = myfabs(R.Get33Element(0,1));  Rabs(0,1) += reps;
  Rabs(0,2) = myfabs(R.Get33Element(0,2));  Rabs(0,2) += reps;
  Rabs(1,0) = myfabs(R.Get33Element(1,0));  Rabs(1,0) += reps;
  Rabs(1,1) = myfabs(R.Get33Element(1,1));  Rabs(1,1) += reps;
  Rabs(1,2) = myfabs(R.Get33Element(1,2));  Rabs(1,2) += reps;
  Rabs(2,0) = myfabs(R.Get33Element(2,0));  Rabs(2,0) += reps;
  Rabs(2,1) = myfabs(R.Get33Element(2,1));  Rabs(2,1) += reps;
  Rabs(2,2) = myfabs(R.Get33Element(2,2));  Rabs(2,2) += reps;

  // Now start with both top level BVs and recurse... 

  CollideRecurse(o1,0,o2,0,flag);
  

  
  double t2 = GetTime();
  this->query_time_secs = t2 - t1;

  return ChC_RESULT_OK; 
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

