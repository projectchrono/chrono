#ifndef CHC_MODELBULLETPARTICLE_H
#define CHC_MODELBULLETPARTICLE_H
 
//////////////////////////////////////////////////
//  
//   ChCModelBulletParticle.h
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
class ChIndexedParticles;

namespace collision 
{


/// Class for the collision model to be used in ChIndexedParticles shapes,
/// that are collections of frame-like items (each with 6 DOFs). 
/// Uses features of the Bullet library.

class ChApi ChModelBulletParticle : public ChModelBullet
{

public:

  ChModelBulletParticle();
  virtual ~ChModelBulletParticle();

		/// Sets the pointer to the client owner (the ChIndexedParticles cluster) and the particle number.
  void SetParticle(ChIndexedParticles* mpa, unsigned int id);

    	/// Gets the pointer to the client owner, ChIndexedParticles cluster. 
  ChIndexedParticles* GetParticles() {return particles;};
    	/// Gets the number of the particle in the particle cluster. 
  unsigned int GetParticleId() {return particle_id;};


	// Overrides and implementations of base members:

		/// Sets the position and orientation of the collision
		/// model as the current position of the corresponding item in ChParticles
  virtual void SyncPosition();

  		/// Gets the pointer to the client owner ChPhysicsItem. 
  virtual ChPhysicsItem* GetPhysicsItem() {return (ChPhysicsItem*)GetParticles();};

private:
	unsigned int particle_id;
	ChIndexedParticles* particles;
};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
