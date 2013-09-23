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

///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoContactN.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoContactN.h" 


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoContactN> a_registration_ChLcpConstraintTwoContactN;

  

void ChLcpConstraintTwoContactN::Project()
{
	if (!constraint_U)
		return;

	if (!constraint_V)
		return;

	// METHOD 
	// Anitescu-Tasora projection on cone generator and polar cone 
	// (contractive, but performs correction on three components: normal,u,v)

	double f_n = this->l_i + this->cohesion;
	double f_u = constraint_U->Get_l_i();
	double f_v = constraint_V->Get_l_i();;
	double f_tang = sqrt (f_v*f_v + f_u*f_u );

		// shortcut
	if (!friction)
	{
		constraint_U->Set_l_i(0);
		constraint_V->Set_l_i(0);
		if (f_n < 0)
			this->Set_l_i( 0 );
		return;
	}

		// inside upper cone? keep untouched!
	if (f_tang < friction * f_n)
		return;

		// inside lower cone? reset  normal,u,v to zero!
	if ((f_tang < -(1.0/friction) * f_n)||(fabs(f_n)<10e-15))
	{
		double f_n_proj = 0;
		double f_u_proj = 0;
		double f_v_proj = 0;

		this->Set_l_i( f_n_proj );
		constraint_U->Set_l_i(f_u_proj);
		constraint_V->Set_l_i(f_v_proj);

		return;
	} 
 
		// remaining case: project orthogonally to generator segment of upper cone
	double f_n_proj =  ( f_tang * friction + f_n ) / (friction*friction + 1) ;
	double f_tang_proj = f_n_proj * friction;
	double tproj_div_t = f_tang_proj / f_tang;
	double f_u_proj = tproj_div_t * f_u;
	double f_v_proj = tproj_div_t * f_v; 

	this->Set_l_i( f_n_proj -this->cohesion);
	constraint_U->Set_l_i(f_u_proj);
	constraint_V->Set_l_i(f_v_proj);					  
}


void ChLcpConstraintTwoContactN::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoBodies::StreamOUT(mstream);

		// stream out all member data..
	mstream << friction;

}

void ChLcpConstraintTwoContactN::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoBodies::StreamIN(mstream);

		// stream in all member data..
	mstream >> friction;
}




} // END_OF_NAMESPACE____


