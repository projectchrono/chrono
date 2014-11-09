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

#ifndef CHLINKLOCK_H
#define CHLINKLOCK_H

///////////////////////////////////////////////////
//
//   ChLinkLock.h
//
//
//   Classes for constraints using the 'link lock'
//   formulation. Such formulation allows the 
//   simulation of many joint types with the same
//   formulas (spherical joint, revolute, and many
//   others, etc) 
//
//   This class is inherited from the base ChLink()
//   class, used by all joints in 3D.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChLinkMasked.h"
#include "physics/ChLimit.h"



namespace chrono
{
///
/// ChLinkLock class.
/// This class implements lot of sub types like the revolute
/// joint, the linear guide, the spherical joint, etc. using
/// the 'lock formulation'.
/// Also, it optionally allows the adoption of 'limits' over
/// upper-lower motions on all the 6 degrees of freedom,
/// thank to the ChLinkLimit objects. 
///

class ChApi ChLinkLock : public ChLinkMasked {

	CH_RTTI(ChLinkLock,ChLinkMasked);

protected:
	Coordsys relC;		// relative costraint position: relC = (relM-deltaC)
	Coordsys relC_dt;	// relative costraint speed
	Coordsys relC_dtdt;	// relative costraint acceleration

	Coordsys deltaC;	// user-imposed rel. position
	Coordsys deltaC_dt;	// user-imposed rel. speed
	Coordsys deltaC_dtdt;// user-imposed rel. acceleration

						//(only for intermediate calculus)
	ChMatrix<>* Cq1_temp;	//
	ChMatrix<>* Cq2_temp;   //   the temporary "lock" jacobians,
	ChMatrix<>* Qc_temp;	//   i.e. the full x,y,z,r0,r1,r2,r3 joint
	Coordsys Ct_temp;	//

	Vector PQw;			// for intermediate calculus (here, for speed reasons)
	Vector PQw_dt;		// 
	Vector PQw_dtdt;	//
	Quaternion q_AD;
	Quaternion q_BC;
	Quaternion q_8;
	Vector q_4;

		// imposed motion
	ChFunction* motion_X;	// user imposed motion for X coord, marker relative
	ChFunction* motion_Y;	// user imposed motion for Y coord, marker relative
	ChFunction* motion_Z;	// user imposed motion for Z coord, marker relative
	ChFunction* motion_ang;	// user imposed angle rotation about axis
	ChFunction* motion_ang2;// user imposed angle rotation if three-angles rot.
	ChFunction* motion_ang3;// user imposed angle rotation if three-angles rot.
	Vector		motion_axis;// this is the axis for the user imposed rotation
	int			angleset;	// type of rotation (3 Eul angles, angle/axis, etc.)
		// limits
	ChLinkLimit* limit_X;	// the upper/lower limits for X dof
	ChLinkLimit* limit_Y;	// the upper/lower limits for Y dof
	ChLinkLimit* limit_Z;	// the upper/lower limits for Z dof
	ChLinkLimit* limit_Rx;// the upper/lower limits for Rx dof
	ChLinkLimit* limit_Ry;// the upper/lower limits for Ry dof
	ChLinkLimit* limit_Rz;// the upper/lower limits for Rz dof
	ChLinkLimit* limit_Rp;// the polar (conical) limit for "shoulder"rotation
	ChLinkLimit* limit_D; // the polar (conical) limit for "shoulder"rotation

	int type;

public:
						// builders and destroyers
	ChLinkLock ();
	virtual ~ChLinkLock ();
	virtual void Copy(ChLinkLock* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


	void BuildLinkType (int link_type);

	void ChangeLinkType (int new_link_type);

	void Set2Dmode(int mode);  // mode=1 use only constraints for 2D xy plane, mode=0 switch back to 3D.

	virtual int GetType	() {return this->type;}



			//
			// UPDATING FUNCTIONS
			//
							// Inherits, and also updates motion laws: deltaC, deltaC_dt, deltaC_dtdt
	virtual void UpdateTime (double mytime);

							// Updates coords relM, relM_dt, relM_dtdt;
							// dist, dist_dt et similia, just like in parent class, but
							// overrides parent implementation of ChLinkMarkers because it can save some
							// temporary vectors (q_4, q_8 etc.) which can be useful in UpdateState(),
							// for speed reasons.
	virtual void UpdateRelMarkerCoords();

							// Given current time and body state, computes
							// the constraint differentiation to get the
							// the state matrices     Cq1,  Cq2,  Qc,  Ct , and also
							// C, C_dt, C_dtd.   ie. the JACOBIAN matrices and friends.
							//  NOTE!! this function uses the fast analytical approach
							// of the "lock formulation".
	virtual void UpdateState ();

							// Inherits, and also updates the local F,M forces adding penalties from 
							// the contained link ChLinkLimit objects, if any.
	virtual void UpdateForces (double mytime);



			//
			// OTHER FUNCTIONS
			//


						// constraint violations in pos/rot coordinates
	Coordsys GetRelC () {return relC;}
	Coordsys GetRelC_dt () {return relC_dt;}
	Coordsys GetRelC_dtdt () {return relC_dtdt;}

						// to get the imposed clearances
	Coordsys GetDeltaC () {return deltaC;}
	Coordsys GetDeltaC_dt () {return deltaC_dt;}
	Coordsys GetDeltaC_dtdt () {return deltaC_dtdt;}
						// to set the imposed clearances (best use SetMotion() if you can..)
	void SetDeltaC (Coordsys mc) {deltaC = mc;}
	void SetDeltaC_dt (Coordsys mc) {deltaC_dt = mc;}
	void SetDeltaC_dtdt (Coordsys mc) {deltaC_dtdt = mc;}

						// for the imposed motion functions
	ChFunction* GetMotion_X() {return motion_X;};
	ChFunction* GetMotion_Y() {return motion_Y;};
	ChFunction* GetMotion_Z() {return motion_Z;};
	ChFunction* GetMotion_ang()  {return motion_ang;};
	ChFunction* GetMotion_ang2() {return motion_ang2;};
	ChFunction* GetMotion_ang3() {return motion_ang3;};
	Vector		GetMotion_axis() {return motion_axis;};
	void SetMotion_X	(ChFunction* m_funct);
	void SetMotion_Y	(ChFunction* m_funct);
	void SetMotion_Z	(ChFunction* m_funct);
	void SetMotion_ang	(ChFunction* m_funct);
	void SetMotion_ang2	(ChFunction* m_funct);
	void SetMotion_ang3	(ChFunction* m_funct);
	void SetMotion_axis (Vector m_axis);
	int   Get_angleset() {return angleset;};
	void  Set_angleset(int mset) {angleset = mset;}

						// for the limits on free degrees
	ChLinkLimit* GetLimit_X()  {return limit_X;}
	ChLinkLimit* GetLimit_Y()  {return limit_Y;}
	ChLinkLimit* GetLimit_Z()  {return limit_Z;}
	ChLinkLimit* GetLimit_Rx() {return limit_Rx;}
	ChLinkLimit* GetLimit_Ry() {return limit_Ry;}
	ChLinkLimit* GetLimit_Rz() {return limit_Rz;}
	ChLinkLimit* GetLimit_Rp() {return limit_Rp;}
	ChLinkLimit* GetLimit_D()  {return limit_D;}
	void SetLimit_X  (ChLinkLimit* m_limit_X)  {if (limit_X) delete limit_X; limit_X = m_limit_X;}
	void SetLimit_Y  (ChLinkLimit* m_limit_Y)  {if (limit_Y) delete limit_Y; limit_Y = m_limit_Y;}
	void SetLimit_Z  (ChLinkLimit* m_limit_Z)  {if (limit_Z) delete limit_Z; limit_Z = m_limit_Z;}
	void SetLimit_Rx (ChLinkLimit* m_limit_Rx) {if (limit_Rx)delete limit_Rx; limit_Rx = m_limit_Rx;}
	void SetLimit_Ry (ChLinkLimit* m_limit_Ry) {if (limit_Ry)delete limit_Ry; limit_Ry = m_limit_Ry;}
	void SetLimit_Rz (ChLinkLimit* m_limit_Rz) {if (limit_Rz)delete limit_Rz; limit_Rz = m_limit_Rz;}
	void SetLimit_Rp (ChLinkLimit* m_limit_Rp) {if (limit_Rp)delete limit_Rp; limit_Rp = m_limit_Rp;}
	void SetLimit_D  (ChLinkLimit* m_limit_D)  {if (limit_D) delete limit_D ; limit_D  = m_limit_D;}


			//
			// LCP SYSTEM FUNCTIONS   ( functions to assembly/manage data for system solver)
			//
						// expand parent constraint stuff from ChLinkMasked because here 
						// it may also consider the	constraints caused by 'limits'..
	virtual void InjectConstraints(ChLcpSystemDescriptor& mdescriptor);
	virtual void ConstraintsBiReset();
	virtual void ConstraintsBiLoad_C(double factor=1., double recovery_clamp=0.1, bool do_clamp=false);
	virtual void ConstraintsBiLoad_Ct(double factor=1.);
	virtual void ConstraintsBiLoad_Qc(double factor=1.);
	virtual void ConstraintsLoadJacobians();
	virtual void ConstraintsFetch_react(double factor=1.);


			//
			// STREAMING
			//

	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};



// SOME WRAPPER CLASSES, TO MAKE 'LINK LOCK' CREATION EASIER...

/// Revolute joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockRevolute : public ChLinkLock 
{
	CH_RTTI(ChLinkLockRevolute,ChLinkLock);
public:
	ChLinkLockRevolute() {ChangeLinkType(LNK_REVOLUTE);}
};

/// 6-dof locked joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockLock : public ChLinkLock 
{
	CH_RTTI(ChLinkLockLock,ChLinkLock);
public:
	ChLinkLockLock() {ChangeLinkType(LNK_LOCK);}
};

/// spherical joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockSpherical : public ChLinkLock 
{
	CH_RTTI(ChLinkLockSpherical,ChLinkLock);
public:
	ChLinkLockSpherical() {ChangeLinkType(LNK_SPHERICAL);}
};

/// cylindrical joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockCylindrical : public ChLinkLock 
{
	CH_RTTI(ChLinkLockCylindrical,ChLinkLock);
public:
	ChLinkLockCylindrical() {ChangeLinkType(LNK_CYLINDRICAL);}
};

/// prismatic joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockPrismatic : public ChLinkLock 
{
	CH_RTTI(ChLinkLockPrismatic,ChLinkLock);
public:
	ChLinkLockPrismatic() {ChangeLinkType(LNK_PRISMATIC);}
};

/// Universal joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockUniversal : public ChLinkLock
{
  CH_RTTI(ChLinkLockUniversal, ChLinkLock);
public:
  ChLinkLockUniversal() { ChangeLinkType(LNK_UNIVERSAL); }
};

/// point-plane joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockPointPlane : public ChLinkLock 
{
	CH_RTTI(ChLinkLockPointPlane,ChLinkLock);
public:
	ChLinkLockPointPlane() {ChangeLinkType(LNK_POINTPLANE);}
};

/// point-line joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockPointLine : public ChLinkLock 
{
	CH_RTTI(ChLinkLockPointLine,ChLinkLock);
public:
	ChLinkLockPointLine() {ChangeLinkType(LNK_POINTLINE);}
};

/// plane-plane joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockPlanePlane : public ChLinkLock 
{
	CH_RTTI(ChLinkLockPlanePlane,ChLinkLock);
public:
	ChLinkLockPlanePlane() {ChangeLinkType(LNK_PLANEPLANE);}
};

/// oldham joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockOldham : public ChLinkLock 
{
	CH_RTTI(ChLinkLockOldham,ChLinkLock);
public:
	ChLinkLockOldham() {ChangeLinkType(LNK_OLDHAM);}
};

/// free joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockFree : public ChLinkLock 
{
	CH_RTTI(ChLinkLockFree,ChLinkLock);
public:
	ChLinkLockFree() {ChangeLinkType(LNK_FREE);}
};

/// align joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockAlign : public ChLinkLock 
{
	CH_RTTI(ChLinkLockAlign,ChLinkLock);
public:
	ChLinkLockAlign() {ChangeLinkType(LNK_ALIGN);}
};

/// parallel joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockParallel : public ChLinkLock 
{
	CH_RTTI(ChLinkLockParallel,ChLinkLock);
public:
	ChLinkLockParallel() {ChangeLinkType(LNK_PARALLEL);}
};

/// perpendicularity joint , with the 'ChLinkLock' formulation.
/// (allows a simplier creation of a link as a sub-type of ChLinkLock). 

class ChApi ChLinkLockPerpend : public ChLinkLock 
{
	CH_RTTI(ChLinkLockPerpend,ChLinkLock);
public:
	ChLinkLockPerpend() {ChangeLinkType(LNK_PERPEND);}
};



} // END_OF_NAMESPACE____

#endif
