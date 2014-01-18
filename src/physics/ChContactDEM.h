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

#ifndef CHCONTACTDEM_H
#define CHCONTACTDEM_H

///////////////////////////////////////////////////
//
//   ChContactDEM.h
//
//   Classes for enforcing constraints between DEM bodies
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChFrame.h"
#include "lcp/ChLcpConstraintTwoContactN.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "collision/ChCCollisionInfo.h"
#include "collision/ChCModelBulletDEM.h"

namespace chrono
{


///
/// Class representing a contact between DEM bodies
///

class ChApi ChContactDEM {

protected:
			//
			// DATA
			//

	collision::ChModelBulletDEM*  m_mod1;  ///< first contact model
	collision::ChModelBulletDEM*  m_mod2;  ///< second contact model

	ChVector<>  m_p1;           ///< max penetration point on geo1, after refining, in abs space
	ChVector<>  m_p2;           ///< max penetration point on geo2, after refining, in abs space
	ChVector<float> m_normal;   ///< normal, on surface of master reference (geo1)

	ChMatrix33<float> m_contact_plane;   ///< the plane of contact (X is normal direction)

	double m_delta;            ///< penetration distance (negative if going inside) after refining

	ChVector<> m_force;        ///< contact force on body1

public:
			//
			// CONSTRUCTORS
			//

	ChContactDEM() {}

	ChContactDEM(collision::ChModelBulletDEM*      mod1,
	             collision::ChModelBulletDEM*      mod2,
	             const collision::ChCollisionInfo& cinfo);

	~ChContactDEM() {}

			//
			// FUNCTIONS
			//

	// Reuse an existing contact
	void Reset(collision::ChModelBulletDEM*      mod1,
	           collision::ChModelBulletDEM*      mod2,
	           const collision::ChCollisionInfo& cinfo);

	/// Get the contact coordinate system, expressed in absolute frame.
	/// This represents the 'main' reference of the link: reaction forces 
	/// are expressed in this coordinate system. Its origin is point P2.
	/// (It is the coordinate system of the contact plane and normal)
	ChCoordsys<> GetContactCoords();

	/// Returns the pointer to a contained 3x3 matrix representing the UV and normal
	/// directions of the contact. In detail, the X versor (the 1s column of the 
	/// matrix) represents the direction of the contact normal.
	ChMatrix33<float>* GetContactPlane() {return &m_contact_plane;}

	/// Get the contact point 1, in absolute coordinates
	const ChVector<>& GetContactP1() const {return m_p1;}

	/// Get the contact point 2, in absolute coordinates
	const ChVector<>& GetContactP2() const {return m_p2;}

	/// Get the contact normal, in absolute coordinates
	const ChVector<float>& GetContactNormal() const {return m_normal;}

	/// Get the contact distance
	double GetContactDistance() const {return m_delta;}
	
	/// Get the contact force, if computed, in absolute coordinates
	const ChVector<>& GetContactForce() const {return m_force;}

	/// Get the collision model 1, with point P1
	collision::ChCollisionModel* GetModel1() {return (collision::ChCollisionModel*) m_mod1;}

	/// Get the collision model 2, with point P2
	collision::ChCollisionModel* GetModel2() {return (collision::ChCollisionModel*) m_mod2;}

	/// Apply contact forces to bodies.
	void ConstraintsFbLoadForces(double factor);

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
