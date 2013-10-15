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
//   ChLcpConstraintThreeBBShaft.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
#include "ChLcpConstraintThreeBBShaft.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintThreeBBShaft> a_registration_ChLcpConstraintThreeBBShaft;



ChLcpConstraintThreeBBShaft& ChLcpConstraintThreeBBShaft::operator=(const ChLcpConstraintThreeBBShaft& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpConstraintThree::operator=(other);

	Cq_a = other.Cq_a;
	Cq_b = other.Cq_b;
	Cq_c = other.Cq_c;
	Eq_a = other.Eq_a;
	Eq_b = other.Eq_b;
	Eq_c = other.Eq_c;

	this->variables_a = other.variables_a;
	this->variables_b = other.variables_b;
	this->variables_c = other.variables_c;

	return *this;
}

 

void ChLcpConstraintThreeBBShaft::SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b, ChLcpVariables* mvariables_c)
{
	assert(dynamic_cast<ChLcpVariablesBody*>(mvariables_a));
	assert(dynamic_cast<ChLcpVariablesBody*>(mvariables_b));
	assert(dynamic_cast<ChLcpVariablesBody*>(mvariables_c));

	if (!mvariables_a || !mvariables_b || !mvariables_c)
	{
		SetValid(false);
		return ;
	}

	SetValid(true);
	variables_a = mvariables_a;
	variables_b = mvariables_b;
	variables_c = mvariables_c;
	
}

void ChLcpConstraintThreeBBShaft::Update_auxiliary()
{
	//1- Assuming jacobians are already computed, now compute
	//   the matrices [Eq_a]=[invM_a]*[Cq_a]'  etc
	if (variables_a->IsActive())
	{
		ChMatrixNM<float,6,1> mtemp1;
		mtemp1.CopyFromMatrixT(Cq_a);
		variables_a->Compute_invMb_v(Eq_a, mtemp1);
	}
	if (variables_b->IsActive())
	{
		ChMatrixNM<float,6,1> mtemp1;
		mtemp1.CopyFromMatrixT(Cq_b);
		variables_b->Compute_invMb_v(Eq_b, mtemp1);
	}
	if (variables_c->IsActive())
	{
		ChMatrixNM<float,1,1> mtemp1;
		mtemp1.CopyFromMatrixT(Cq_c);
		variables_c->Compute_invMb_v(Eq_c, mtemp1);
	}

	//2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i 
	ChMatrixNM<float,1,1> res;
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
	if (variables_c->IsActive())
	{
		res.MatrMultiply(Cq_c, Eq_c); 
		g_i += res(0,0);
	}

	//3- adds the constraint force mixing term (usually zero):
	if (cfm_i)
		g_i += cfm_i;
}; 
 






void ChLcpConstraintThreeBBShaft::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintThree::StreamOUT(mstream);

		// stream out all member data
	// NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
	// important to waste disk space.. they may be recomputed run-time,
	// and pointers to variables must be rebound in run-time.)
	// mstream << Cq_a;
	// mstream << Cq_b;
}

void ChLcpConstraintThreeBBShaft::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintThree::StreamIN(mstream);

		// stream in all member data
	// NOTHING INTERESTING TO DESERIALIZE (the Cq jacobians are not so
	// important to waste disk space.. they may be recomputed run-time,
	// and pointers to variables must be rebound in run-time.)
	// mstream >> Cq_a;
	// mstream >> Cq_b;
}




} // END_OF_NAMESPACE____


