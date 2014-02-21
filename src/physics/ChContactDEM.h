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
//   Class for DEM-based contact between bodies
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChFrame.h"
#include "collision/ChCCollisionInfo.h"
#include "collision/ChCModelBulletBody.h"

namespace chrono
{

///
/// Class representing a contact between DEM bodies
///
class ChApi ChContactDEM
{
public:

	enum NormalForceModel {
		HuntCrossley
	};

	enum TangentialForceModel {
		SimpleCoulombSliding,
		LinearSpring,
		LinearDampedSpring
	};

	/// Constructors/destructor
	ChContactDEM() {}
	ChContactDEM(collision::ChModelBulletBody*     mod1,
	             collision::ChModelBulletBody*     mod2,
	             const collision::ChCollisionInfo& cinfo);

	~ChContactDEM() {}

	/// This is the worked function for calculating and recording a new
	/// contact. It calculates and stores kinematic information and the
	/// resulting contact force and is used to construct a new contact
	//// or to reset an existing one for reuse.
	void Reset(collision::ChModelBulletBody*     mod1,
	           collision::ChModelBulletBody*     mod2,
	           const collision::ChCollisionInfo& cinfo);

	/// Get the contact coordinate system, expressed in absolute frame.
	/// This is the coordinate system of the contact plane and normal.
	/// Its origin is at point P2.
	ChCoordsys<> GetContactCoords() const;

	/// Returns the pointer to a 3x3 matrix representing the normal and
	/// tangential (UV) directions of the contact. In particular, the X
	/// unit vector (first column of the matrix) represents the direction
	/// of the contact normal.
	const ChMatrix33<float>& GetContactPlane() const {return m_contact_plane;}

	/// Get the contact point 1, expressed in absolute coordinates.
	const ChVector<>& GetContactP1() const {return m_p1;}

	/// Get the contact point 2, expressed in absolute coordinates.
	const ChVector<>& GetContactP2() const {return m_p2;}

	/// Get the contact normal, expressed in absolute coordinates.
	/// The contact normal points from P2 to P1.
	const ChVector<>& GetContactNormal() const {return m_normal;}

	/// Get the contact penetration (positive if there is overlap).
	double GetContactPenetration() const {return m_delta;}
	
	/// Get the contact force, expressed in absolute coordinates. This is
	/// the force applied to body 2 (for body 1 it is inverted).
	const ChVector<>& GetContactForce() const {return m_force;}

	/// Get the collision model 1, with point P1.
	collision::ChCollisionModel* GetModel1() {return (collision::ChCollisionModel*) m_mod1;}

	/// Get the collision model 2, with point P2.
	collision::ChCollisionModel* GetModel2() {return (collision::ChCollisionModel*) m_mod2;}

	/// Calculate contact force, expressed in absolute coordinates.
	void CalculateForce();

	/// Apply contact forces to bodies.
	void ConstraintsFbLoadForces(double factor);

	/// Contact force models
	static NormalForceModel     m_normalForceModel;
	static TangentialForceModel m_tangentialForceModel;

	static void                 SetNormalContactModel(NormalForceModel model) {m_normalForceModel = model;}
	static NormalForceModel     GetNormalContactModel()                       {return m_normalForceModel;}

	static void                 SetTangentialForceModel(TangentialForceModel model) {m_tangentialForceModel = model;}
	static TangentialForceModel GetTangentialForceModel()                           {return m_tangentialForceModel;}

	/// Slip velocity threshold. No tangential contact forces are generated
	/// if the magnitude of the tangential relative velocity is below this.
	static double m_minSlipVelocity;

	static void SetSlipVelocitythreshold(double vel) {m_minSlipVelocity = vel;}

private:

	collision::ChModelBulletBody*  m_mod1;          ///< first contact model
	collision::ChModelBulletBody*  m_mod2;          ///< second contact model

	double                         m_delta;         ///< penetration distance (positive if going inside)
	ChVector<>                     m_p1;            ///< max penetration point on surf1, in abs frame
	ChVector<>                     m_p2;            ///< max penetration point on surf2, in abs frame
	ChVector<>                     m_normal;        ///< normal, on surface of master reference (surf1)
	ChMatrix33<float>              m_contact_plane; ///< the plane of contact (X is normal direction)
	ChVector<>                     m_p1_loc;        ///< max. penetration point on surf1, in local frame
	ChVector<>                     m_p2_loc;        ///< max. penetration point on surf2, in local frame

	ChVector<>                     m_force;         ///< contact force on body2
};



} // END_OF_NAMESPACE____


#endif
