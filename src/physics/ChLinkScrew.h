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

#ifndef CHLINKSCREW_H
#define CHLINKSCREW_H

///////////////////////////////////////////////////
//
//   ChLinkScrew.h
//
//
//   Classes for screw joint
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
/// Screw joint between two rigid bodies. This 
/// link type is able to couple translation and rotation.
///

class ChApi ChLinkScrew : public ChLinkLock {

	CH_RTTI(ChLinkScrew,ChLinkLock);

protected:
	double tau;			// transmission coeff.

public:
						// builders and destroyers
	ChLinkScrew ();
	virtual ~ChLinkScrew ();
	virtual void Copy(ChLinkScrew* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer

							// UPDATING FUNCTIONS - "screw" custom implementations

							// Inherit the link-lock computations like it were a
							// normal "revolute" joint, but then modifies the Z-lock parts of C,
							// Cdt, Cdtdt, [Cq] etc., in order to have z = tau * alpha.
	virtual void UpdateState ();

			// data get/set
	double  Get_tau() {return tau;};
	void  Set_tau(double mset) {tau = mset;}
	double  Get_thread() {return tau*(2*CH_C_PI);};
	void  Set_thread(double mset) {tau = mset/(2*CH_C_PI);}

							// STREAMING
	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
