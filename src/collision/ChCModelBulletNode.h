#ifndef CHC_MODELBULLETNODE_H
#define CHC_MODELBULLETNODE_H
 
//////////////////////////////////////////////////
//  
//   ChCModelBulletNode.h
//
//   A wrapper to use the Bullet collision detection
//   library
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "collision/ChCModelBullet.h"


namespace chrono 
{

// forward references
class ChIndexedNodes;

namespace collision 
{


/// Class for the collision model to be used in ChIndexedNodes shapes,
/// that are collections of point-like items (each with 3 DOFs). 
/// Uses features of the Bullet library.

class ChApi ChModelBulletNode : public ChModelBullet
{

public:

  ChModelBulletNode();
  virtual ~ChModelBulletNode();

		/// Sets the pointer to the client owner (the ChIndexedNodes container) and the node number.
  void SetNode(ChIndexedNodes* mpa, unsigned int id);

    	/// Gets the pointer to the client owner, ChIndexedNodes cluster. 
  ChIndexedNodes* GetNodes() {return nodes;};
    	/// Gets the number of the node in the  cluster. 
  unsigned int GetNodeId() {return node_id;};

		/// If the collision shape is a sphere, resize it and return true (if no 
		/// sphere is found in this collision shape, return false). 
		/// It can also change the outward envelope; the inward margin is automatically the radius of the sphere. 
  bool SetSphereRadius(double coll_radius, double out_envelope);


	// Overrides and implementations of base members:

		/// Sets the position and orientation of the collision
		/// model as the current position of the corresponding node
  virtual void SyncPosition();

  		/// Gets the pointer to the client owner ChPhysicsItem. 
  virtual ChPhysicsItem* GetPhysicsItem() {return (ChPhysicsItem*)GetNodes();};

private:
	unsigned int node_id;
	ChIndexedNodes* nodes;
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
