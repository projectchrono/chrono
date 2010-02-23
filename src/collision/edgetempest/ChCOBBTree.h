#ifndef CHC_OBBTREE
#define CHC_OBBTREE

//////////////////////////////////////////////////
//  
//   ChCOBBTree.h
//
//   Class for the binary space partitioning tree 
//   (aka BSP) based on OBB bounding volumes
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChCCollisionTree.h"
#include "ChCOBB.h"

 
namespace chrono 
{
namespace collision 
{


///
/// Class containing a tree of Object oriented boxes OOB, 
/// ready for collision detection.
///


class ChOBBTree : public ChCollisionTree
{

public:

  ChOBBTree();
  ~ChOBBTree();

		/// Deletes all inserted geometries and resets bounding box hierarchies
  int ResetModel();

		/// Rebuilds the OBB BV hierarchy
		/// (boxes may be inflated by an 'envelope' amount).
  int BuildModel(double envelope=0.);

		/// This function can be used if you want to easily scan the hierarchy 
		/// of bounding boxes. You must provide a callback to a function which 
		/// will be automatically called per each bounding box in this tree.
		/// Note that the callback will get info about the rotation Rot of the 
		/// box, its position Pos, its size d, its level. Also you can pass generic
		/// user data via a pointer.  \return the number of scanned boxes.  
  int TraverseBoundingBoxes( void callback(ChMatrix33<>& Rot,Vector& Pos,Vector& d,int level, void* userdata),
									  void* userdata);
	
		/// Returns the n-th child of the tree.
	ChOBB* child(int n) { return &b[n]; }

public:

		/// Vector of AABB bounding boxes.
	std::vector<ChOBB> b;

		/// Used by internal algorithms
	int current_box;

		/// From geometric objects, builds the hierarchy of BV bounding volumes
	int build_model(double envelope);
};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
