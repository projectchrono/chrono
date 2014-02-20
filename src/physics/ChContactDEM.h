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
#include "collision/ChCModelBulletBody.h"

namespace chrono
{

///
/// Structure with kinematic contact data
///
struct ChContactKinematicsDEM
{
	double            delta;           ///< penetration distance (negative if going inside) after refining
	ChVector<>        p1;              ///< max penetration point on surf1, after refining, in abs frame
	ChVector<>        p2;              ///< max penetration point on surf2, after refining, in abs frame
	ChVector<>        normal;          ///< normal, on surface of master reference (surf1)
	ChMatrix33<float> contact_plane;   ///< the plane of contact (X is normal direction)
	ChVector<>        p1_loc;          ///< max. penetration point on surf1, in local frame
	ChVector<>        p2_loc;          ///< max. penetration point on surf2, in local frame
};

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

	/// Constructors
	ChContactDEM() {}

	ChContactDEM(collision::ChModelBulletBody*     mod1,
	             collision::ChModelBulletBody*     mod2,
	             const collision::ChCollisionInfo& cinfo);

	~ChContactDEM() {}

	/// Reuse an existing contact
	void Reset(collision::ChModelBulletBody*     mod1,
	           collision::ChModelBulletBody*     mod2,
	           const collision::ChCollisionInfo& cinfo);

	/// Get the contact coordinate system, expressed in absolute frame.
	/// This represents the 'main' reference of the link: reaction forces 
	/// are expressed in this coordinate system. Its origin is point P2.
	/// (It is the coordinate system of the contact plane and normal)
	ChCoordsys<> GetContactCoords();

	/// Returns the pointer to a contained 3x3 matrix representing the UV and normal
	/// directions of the contact. In detail, the X versor (the 1s column of the 
	/// matrix) represents the direction of the contact normal.
	ChMatrix33<float>* GetContactPlane() {return &m_kdata.contact_plane;}

	/// Get the contact point 1, in absolute coordinates
	const ChVector<>& GetContactP1() const {return m_kdata.p1;}

	/// Get the contact point 2, in absolute coordinates
	const ChVector<>& GetContactP2() const {return m_kdata.p2;}

	/// Get the contact normal, in absolute coordinates
	const ChVector<>& GetContactNormal() const {return m_kdata.normal;}

	/// Get the contact penetration
	double GetContactPenetration() const {return m_kdata.delta;}
	
	/// Get the contact force, if computed, in absolute coordinates
	const ChVector<>& GetContactForce() const {return m_force;}

	/// Get the collision model 1, with point P1
	collision::ChCollisionModel* GetModel1() {return (collision::ChCollisionModel*) m_mod1;}

	/// Get the collision model 2, with point P2
	collision::ChCollisionModel* GetModel2() {return (collision::ChCollisionModel*) m_mod2;}

	/// Calculate contact force
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

	/// Slip velocity threshold
	static double m_minSlipVelocity;

	static void SetSlipVelocitythreshold(double vel) {m_minSlipVelocity = vel;}

private:

	collision::ChModelBulletBody*  m_mod1;  ///< first contact model
	collision::ChModelBulletBody*  m_mod2;  ///< second contact model

	ChContactKinematicsDEM         m_kdata;  ///< contact kinematics data

	ChVector<>                     m_force;  ///< contact force on body1
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
