//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_MODELBULLETBODY_H
#define CHC_MODELBULLETBODY_H
 
//////////////////////////////////////////////////
//  
//   ChCModelBulletBody.h
//
//   A wrapper to use the Bullet collision detection
//   library
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "collision/ChCModelBullet.h"


namespace chrono 
{

// forward references
class ChBody;

namespace collision 
{


/// Class for the collision model to be used in ChIndexedParticles shapes,
/// that are collections of point-like nodes (each with 3 DOFs)
/// using features of the Bullet library.

class ChApi ChModelBulletBody : public ChModelBullet
{

public:

  ChModelBulletBody();
  virtual ~ChModelBulletBody();
  

    	/// Gets the pointer to the client owner rigid body. 
  ChBody* GetBody() const {return mbody;};
		/// Sets the pointer to the client owner rigid body

  
	// Overrides and implementations of base members:

		/// Sets the position and orientation of the collision
		/// model as the current position of the corresponding ChBody
  virtual void SyncPosition();

  		/// Gets the pointer to the client owner ChPhysicsItem. 
  virtual ChPhysicsItem* GetPhysicsItem() {return (ChPhysicsItem*)GetBody();};

private:

};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
