//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHBODYDEM_H
#define CHBODYDEM_H

//////////////////////////////////////////////////
//
//   ChBodyDEM.h
//
//   Class for rigid bodies, that is rigid moving
//   parts with mass and collision geometry 
//   which interact through DEM approach.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChBody.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChMaterialSurfaceDEM.h"



namespace chrono
{

using namespace collision;



///
/// Class for rigid bodies. A rigid body is an entity which
/// can move in 3D space, and can be constrained to other rigid
/// bodies using ChLink objects.
/// These objects have mass and inertia properties. A shape can also
/// be associated to the body, for collision detection.
///

class ChApi ChBodyDEM: public ChBody {

	// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBodyDEM, ChBody);


private:
			//
			// DATA
			//

	// data for surface contact and impact (can be shared):
	ChSharedPtr<ChMaterialSurfaceDEM> matsurfaceDEM;

public:

			//
			// CONSTRUCTORS
			//

	ChBodyDEM();
	ChBodyDEM(ChCollisionModel* new_collision_model);
	~ChBodyDEM() {}

	/// Copy from another body
	void Copy(ChBodyDEM* source);

			//
			// FUNCTIONS
			//

	/// Instantiate the collision model
	virtual ChCollisionModel* InstanceCollisionModel();

	const ChSharedPtr<ChMaterialSurfaceDEM>& GetMaterialSurfaceDEM() const {return matsurfaceDEM;}
	ChSharedPtr<ChMaterialSurfaceDEM>& GetMaterialSurfaceDEM() {return matsurfaceDEM;}
	void SetMaterialSurfaceDEM(ChSharedPtr<ChMaterialSurfaceDEM>& mnewsurf) {matsurfaceDEM = mnewsurf;}

	/// accumulate force on this body, or reset the force to zero
	//void AccumulateForce(ChVector<> ff) {Xforce+=ff;}
	//void ResetBodyForce() {Xforce = VNULL;}
	//void AccumulateTorque(ChVector<> tt) {Xtorque+=tt;}
	//void ResetBodyTorque() {Xtorque = VNULL;}

			//
			// STREAMING
			//

	/// Method to allow deserializing a persistent binary archive (ex: a file)
	/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

	/// Method to allow serializing transient data into a persistent
	/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

	/// Save data, including child markers and child forces
	int StreamOUTall(ChStreamOutBinary& m_file);

	/// Read data, including child markers and child forces
	int StreamINall(ChStreamInBinary&  m_file);

};



typedef ChSharedPtr<ChBodyDEM> ChSharedBodyDEMPtr;



} // END_OF_NAMESPACE____


#endif
