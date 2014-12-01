//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoBodies.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
#include "ChLcpConstraintTwoBodies.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoBodies> a_registration_ChLcpConstraintTwoBodies;



ChLcpConstraintTwoBodies& ChLcpConstraintTwoBodies::operator=(const ChLcpConstraintTwoBodies& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpConstraintTwo::operator=(other);

	Cq_a = other.Cq_a;
	Cq_b = other.Cq_b;
	Eq_a = other.Eq_a;
	Eq_b = other.Eq_b;

	this->variables_a = other.variables_a;
	this->variables_b = other.variables_b;

	return *this;
}

 

void ChLcpConstraintTwoBodies::SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b)
{
	assert(dynamic_cast<ChLcpVariablesBody*>(mvariables_a));
	assert(dynamic_cast<ChLcpVariablesBody*>(mvariables_b));

	if (!mvariables_a || !mvariables_b)
	{
		SetValid(false);
		return ;
	}

	SetValid(true);
	variables_a = mvariables_a;
	variables_b = mvariables_b;
	
}

void ChLcpConstraintTwoBodies::Update_auxiliary()
{
	//1- Assuming jacobians are already computed, now compute
	//   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
	if (variables_a->IsActive())
	{
		ChMatrixNM<double,6,1> mtemp1;
		mtemp1.CopyFromMatrixT(Cq_a);
		variables_a->Compute_invMb_v(Eq_a, mtemp1);
	}
	if (variables_b->IsActive())
	{
		ChMatrixNM<double,6,1> mtemp1;
		mtemp1.CopyFromMatrixT(Cq_b);
		variables_b->Compute_invMb_v(Eq_b, mtemp1);
	}

	//2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i 
	ChMatrixNM<double,1,1> res;
	g_i = 0;
	if (variables_a->IsActive())
	{
		res.MatrMultiply(Cq_a, Eq_a);
		g_i = res(0,0);
	}
	if (variables_b->IsActive())
	{
		res.MatrMultiply(Cq_b, Eq_b); 
		g_i += res(0,0);
	}

	//3- adds the constraint force mixing term (usually zero):
	if (cfm_i)
		g_i += cfm_i;
}; 
 






void ChLcpConstraintTwoBodies::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwo::StreamOUT(mstream);

		// stream out all member data
	// NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
	// important to waste disk space.. they may be recomputed run-time,
	// and pointers to variables must be rebound in run-time.)
	// mstream << Cq_a;
	// mstream << Cq_b;
}

void ChLcpConstraintTwoBodies::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwo::StreamIN(mstream);

		// stream in all member data
	// NOTHING INTERESTING TO DESERIALIZE (the Cq jacobians are not so
	// important to waste disk space.. they may be recomputed run-time,
	// and pointers to variables must be rebound in run-time.)
	// mstream >> Cq_a;
	// mstream >> Cq_b;
}




} // END_OF_NAMESPACE____


