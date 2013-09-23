//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKWHEEL_H
#define CHLINKWHEEL_H

///////////////////////////////////////////////////
//
//   ChLink.h
//
//
//   Classes for fast simulation of rigid wheels
//   with zero mass.
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

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_WHEEL		26


#define WCOLLISION_PLANE	0
#define WCOLLISION_YCOLLIDE 1

///
/// Class representing a wheel constraint (for fast simulation
/// of cars, etc.)
///

class ChApi ChLinkWheel : public ChLinkLock {

	CH_RTTI(ChLinkWheel,ChLinkLock);

protected:
	ChFunction* wheel_rotation;		// rotation(time) or rotation_speed(time)
	int	speed_handled;		// if TRUE, wheel_rotation means angular speed, not angle.
	double radius;			//  wheel radius
	double thickness;		//  wheel thickness

								// FRICTION MODEL
	double friction;		// friction coefficient;
	ChFunction* fri_spe;	// friction(slipping_speed)	 modulation (def=1)
	ChFunction* fri_norm;	// friction(normal_contact_force)  modulation (def=1)

								// STICKING
	int allow_sticking;		// if true, tangential contraint will be added when
							// sticking happens (for  slip_speed < slip_treshold)
	double slip_treshold;	// def. 0.01 m/s ??
	double static_friction;	// statical friction coefficient (to guess when breaking tangential sticking contraint);

								// VERTICAL CONSTRAINT
	int wcollision;			// collision detection mode
	int	unilateral;			// if TRUE, unilateral constraint, FALSE= bilateral;
	int pneus_krp;			// if TRUE, uses the nonlinear functions of k and r to simulate pneus radial deformability
	 double rad_k;			// radial stiffness;
	 double rad_r;			// radial damping;
	 double rad_p;			// radial pressure; (const.radial force f, per square surf.unit)
	 double pneus_h;		// radial height of pneus (percentual of radius, 0..1)
	 ChFunction* rad_k_def;	// modulation of k stiffness as a function of deformation

								// INFO DATA   -also as auxiliary temp.data for intermediate calculus-
	double angle;			// current wheel rotation angle (artificial, for simplified model)
	double angle_dt;		// "" speed
	double angle_dtdt;		// "" acceleration
	double slipping;		// absolute slipping speed;
	double f_slip;			// the frontal part of slipping (in direction of the wheel)
	double l_slip;			// the lateral part of slipping (perp. to direction of wheel)
	double derive_angle;	// derive angle
	double tforce;			// tangential contact force (modulus) on wheel
	double f_tforce;		// the frontal part of contact force (in direction of the wheel)
	double l_tforce;		// the lateral part of contact force (perp. to direction of wheel)
	double curr_friction;	// the current value of friction coeff as f=f(slip, norm.pressure, etc)

								// Auxiliary, internal
	ChMatrix33<> spindle_csys;	// direction of spindle (Z axis used)
	Vector spindle_pos;		// position of spindle
	int loc_iters;
	double mu;
	double mv;
	double malpha;

public:
						// builders and destroyers
	ChLinkWheel ();
	virtual ~ChLinkWheel ();
	virtual void Copy(ChLinkWheel* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


							// UPDATING FUNCTIONS - "wheel link" custom implementations

							// Updates the position of marker m2, moving it -tangentially- on the
							// surf/wheel contact point (m1 representing the center of wheel)
							// Updates motion laws depending on surface & wheel curvatures.
	virtual void UpdateTime (double mytime);

							// Apply tangential force/torque to terrain and wheel, depending on
							// relative slipping.  Also, all the INFO DATA datas are computed.
	virtual void UpdateForces (double mytime);


			// data get/set
	ChFunction* Get_wheel_rotation() {return wheel_rotation;};
	void Set_wheel_rotation(ChFunction* m_funct);
	int   Get_speed_handled() {return speed_handled;};
	void  Set_speed_handled(int mset) {speed_handled = mset;}
	int   Get_wcollision() {return wcollision;};
	void  Set_wcollision(int mset) {wcollision = mset;}
	double  Get_radius() {return radius;};
	void  Set_radius(double mset) {radius = mset; this->limit_Z->Set_min(radius-thickness);}
	double  Get_thickness() {return thickness;};
	void  Set_thickness(double mset) {thickness = mset; this->limit_Z->Set_min(radius-thickness);}
	double  Get_friction() {return friction;};
	void  Set_friction(double mset) {friction = mset;}
	ChFunction* Get_fri_spe() {return fri_spe;};
	void Set_fri_spe(ChFunction* m_funct);
	ChFunction* Get_fri_norm() {return fri_norm;};
	void Set_fri_norm(ChFunction* m_funct);
	int   Get_allow_sticking() {return allow_sticking;};
	void  Set_allow_sticking(int mset) {allow_sticking = mset;}
	double  Get_slip_treshold() {return slip_treshold;};
	void  Set_slip_treshold(double mset) {slip_treshold = mset;}
	double  Get_static_friction() {return static_friction;};
	void  Set_static_friction(double mset) {static_friction = mset;}
	int   Get_unilateral() {return unilateral;};
	void  Set_unilateral(int mset); // see cpp,
	int   Get_pneus_krp() {return pneus_krp;};
	void  Set_pneus_krp(int mset) {pneus_krp = mset;}
	double Get_rad_k() {return rad_k;};
	void  Set_rad_k(double mset) {rad_k = mset;}
	double Get_rad_r() {return rad_r;};
	void  Set_rad_r(double mset) {rad_r = mset;}
	double Get_rad_p() {return rad_p;};
	void  Set_rad_p(double mset) {rad_p = mset;}
	double Get_pneus_h() {return pneus_h;};
	void  Set_pneus_h(double mset) {pneus_h = mset; if (pneus_h >1) pneus_h = 1;}
	void Set_rad_k_def(ChFunction* m_funct);
	ChFunction* Get_rad_k_def() {return rad_k_def;};
	double Get_RigidRadius() {return (radius - thickness);};
	ChMatrix33<>* Get_wheel_spindle_csys() {return &spindle_csys;};
	Vector  Get_wheel_spindle_pos() {return spindle_pos;};

	double Get_angle() {return angle;};
	double Get_angle_dt() {return angle_dt;};
	double Get_angle_dtdt() {return angle_dtdt;};
	double Get_slipping() {return slipping;};
	double Get_f_slip() {return f_slip;};
	double Get_l_slip() {return l_slip;};
	double Get_derive_angle() {return derive_angle;};
	double Get_tforce() {return tforce;};
	double Get_f_tforce() {return f_tforce;};
	double Get_l_tforce() {return l_tforce;};
	double Get_curr_friction() {return curr_friction;};


							// STREAMING
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};





//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
