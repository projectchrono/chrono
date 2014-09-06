//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKCLEARANCE_H
#define CHLINKCLEARANCE_H

///////////////////////////////////////////////////
//
//   ChLinkClearance.h
//
//
//   Classes for revolute joints with clearance.
//
//   HEADER file for CHRONO, 
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChLinkLock.h"


namespace chrono
{
///
/// A class for the custom fast simulation of revolute
/// joints with clearance.
/// ***OBSOLETE***

class ChApi ChLinkClearance : public ChLinkLock {

	CH_RTTI(ChLinkClearance,ChLinkLock);

protected:
	double clearance;			// distance offset
	double c_friction;			// friction coeff.
	double c_restitution;		// restitution coeff.
	double c_tang_restitution;	// restitution coeff tangential
	double c_viscous;			// viscous friction in contact point

	double diameter;			// radius of shaft (in case of circular shaft)

	Vector contact_F_abs;		// [internal]
	Vector contact_V_abs;		// [internal]

public:
						// builders and destroyers
	ChLinkClearance ();
	virtual ~ChLinkClearance ();
	virtual void Copy(ChLinkClearance* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


							// UPDATING FUNCTIONS - "lin.act. link" custom implementations

							// Updates marker positions, etc.
	virtual void UpdateTime (double mytime);
							// Updates forces
	virtual void UpdateForces (double mytime);

			// data get/set
	double  Get_clearance() {return clearance;};
	void  Set_clearance(double mset) {clearance = mset; limit_X->Set_max(clearance);}
	double  Get_c_friction() {return c_friction;};
	void  Set_c_friction(double mset) {c_friction = mset;}
	double  Get_c_restitution() {return c_restitution;};
	void  Set_c_restitution(double mset) {c_restitution = mset; limit_X->Set_maxElastic(c_restitution);}
	double  Get_c_tang_restitution() {return c_tang_restitution;};
	void  Set_c_tang_restitution(double mset) {c_tang_restitution = mset;}
	double  Get_c_viscous() {return c_viscous;};
	void  Set_c_viscous(double mset) {c_viscous = mset;}
	double  Get_diameter() {return diameter;};
	void  Set_diameter(double mset) {diameter = mset;}

			// easy data getting
	double  Get_axis_eccentricity();// distance between the two shafts
	double  Get_axis_phase();		// phase of center of shaft, respect to hole
	double  Get_rotation_angle();	// rotation of shafti in hole (relative)
	Vector  Get_contact_P_abs();	// absolute contact point
	Vector  Get_contact_N_abs();	// absolute normal to contact
	Vector  Get_contact_F_abs();	// absolute force in contact
	double  Get_contact_F_n();		// normal  part of force
	double  Get_contact_F_t();		// tangent part of force
	double  Get_contact_V_t();		// tangent part of speed

	int		Get_is_in_contact();	// returns: 1= is sliding contact, 0= is flying

							// STREAMING
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};





//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
