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

///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoRollingN.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoRollingN.h" 


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoRollingN> a_registration_ChLcpConstraintTwoRollingN;

  

void ChLcpConstraintTwoRollingN::Project()
{
	if (!constraint_U)
		return;

	if (!constraint_V)
		return;

	if (!constraint_N)
		return;

	// METHOD 
	// Anitescu-Tasora projection on rolling-friction cone generator and polar cone 
	// (contractive, but performs correction on three components: normal,u,v)

	double f_n = constraint_N->Get_l_i();
	double t_n = this->Get_l_i();
	double t_u = constraint_U->Get_l_i();
	double t_v = constraint_V->Get_l_i();
	double t_tang = sqrt (t_v*t_v + t_u*t_u );
	double t_sptang = fabs(t_n);  // = sqrt(t_n*t_n);

	// A Project the spinning friction (approximate - should do cone 
	//   projection stuff as in B, but spinning friction is usually very low...)

	if (spinningfriction)
	{
			// inside upper cone? keep untouched!
		if (t_sptang < spinningfriction * f_n)
		{
		}
		else
		{
				// inside lower cone? reset  normal,u,v to zero!
			if ((t_sptang < -(1.0/spinningfriction) * f_n)||(fabs(f_n)<10e-15))
			{
				constraint_N->Set_l_i(0 );
				this->Set_l_i        (0 );
			}
			else
			{
					// remaining case: project orthogonally to generator segment of upper cone (CAN BE simplified)
				double f_n_proj =  ( t_sptang * spinningfriction + f_n ) / (spinningfriction*spinningfriction + 1) ;
				double t_tang_proj = f_n_proj * spinningfriction;
				double tproj_div_t = t_tang_proj / t_sptang;
				double t_n_proj = tproj_div_t * t_n; 

				constraint_N->Set_l_i(f_n_proj);
				this->Set_l_i        (t_n_proj);
			}
		}
	}

	// B Project the rolling friction

		// shortcut
	if (!rollingfriction)
	{
		constraint_U->Set_l_i(0);
		constraint_V->Set_l_i(0);
		if (f_n < 0)
			constraint_N->Set_l_i( 0 );
		return;
	}

		// inside upper cone? keep untouched!
	if (t_tang < rollingfriction * f_n)
		return;

		// inside lower cone? reset  normal,u,v to zero!
	if ((t_tang < -(1.0/rollingfriction) * f_n)||(fabs(f_n)<10e-15))
	{
		double f_n_proj = 0;
		double t_u_proj = 0;
		double t_v_proj = 0;

		constraint_N->Set_l_i(f_n_proj );
		constraint_U->Set_l_i(t_u_proj);
		constraint_V->Set_l_i(t_v_proj);

		return;
	} 
 
		// remaining case: project orthogonally to generator segment of upper cone
	double f_n_proj =  ( t_tang * rollingfriction + f_n ) / (rollingfriction*rollingfriction + 1) ;
	double t_tang_proj = f_n_proj * rollingfriction;
	double tproj_div_t = t_tang_proj / t_tang;
	double t_u_proj = tproj_div_t * t_u;
	double t_v_proj = tproj_div_t * t_v; 

	constraint_N->Set_l_i(f_n_proj);
	constraint_U->Set_l_i(t_u_proj);
	constraint_V->Set_l_i(t_v_proj);	
	
}


void ChLcpConstraintTwoRollingN::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoBodies::StreamOUT(mstream);

		// stream out all member data..
	mstream << rollingfriction;
	mstream << spinningfriction;

}

void ChLcpConstraintTwoRollingN::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoBodies::StreamIN(mstream);

		// stream in all member data..
	mstream >> rollingfriction;
	mstream >> spinningfriction;
}




} // END_OF_NAMESPACE____


