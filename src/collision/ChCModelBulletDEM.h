#ifndef CHC_MODELBULLETDEM_H
#define CHC_MODELBULLETDEM_H
 
//////////////////////////////////////////////////
//  
//   ChCModelBulletDEM.h
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
class ChBodyDEM;

namespace collision 
{


/// Class for the collision model to be used in ChIndexedParticles shapes,
/// that are collections of point-like nodes (each with 3 DOFs)
/// using features of the Bullet library.

class ChApi ChModelBulletDEM : public ChModelBullet
{

public:

  ChModelBulletDEM();
  virtual ~ChModelBulletDEM();
  

    	/// Gets the pointer to the client owner rigid body. 
  ChBodyDEM* GetBody() const {return mbody;};
		/// Sets the pointer to the client owner rigid body
  void SetBody(ChBodyDEM* mbo) {mbody = mbo;};

  
	// Overrides and implementations of base members:

		/// Sets the position and orientation of the collision
		/// model as the current position of the corresponding ChBody
  virtual void SyncPosition();

  		/// Gets the pointer to the client owner ChPhysicsItem. 
  virtual ChPhysicsItem* GetPhysicsItem() {return (ChPhysicsItem*)GetBody();};

private:
	ChBodyDEM* mbody;
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
