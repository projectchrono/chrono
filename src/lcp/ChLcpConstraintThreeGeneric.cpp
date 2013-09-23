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
//   ChLcpConstraintThreeGeneric.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
#include "ChLcpConstraintThreeGeneric.h" 

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintThreeGeneric> a_registration_ChLcpConstraintThreeGeneric;



ChLcpConstraintThreeGeneric& ChLcpConstraintThreeGeneric::operator=(const ChLcpConstraintThreeGeneric& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpConstraintThree::operator=(other);

	if (other.Cq_a)
	{
		if (Cq_a==NULL)
			Cq_a = new ChMatrixDynamic<float>;
		Cq_a->CopyFromMatrix(*other.Cq_a);
	}
	else
	{
		if (Cq_a) delete Cq_a; Cq_a=NULL;
	}

	if (other.Cq_b)
	{
		if (Cq_b==NULL)
			Cq_b = new ChMatrixDynamic<float>;
		Cq_b->CopyFromMatrix(*other.Cq_b);
	}
	else
	{
		if (Cq_b) delete Cq_b; Cq_b=NULL;
	}

	if (other.Cq_c)
	{
		if (Cq_c==NULL)
			Cq_c = new ChMatrixDynamic<float>;
		Cq_c->CopyFromMatrix(*other.Cq_c);
	}
	else
	{
		if (Cq_c) delete Cq_c; Cq_c=NULL;
	}

	if (other.Eq_a)
	{
		if (Eq_a==NULL)
			Eq_a = new ChMatrixDynamic<float>;
		Eq_a->CopyFromMatrix(*other.Eq_a);
	}
	else
	{
		if (Eq_a) delete Eq_a; Eq_a=NULL;
	}

	if (other.Eq_b)
	{
		if (Eq_b==NULL)
			Eq_b = new ChMatrixDynamic<float>;
		Eq_b->CopyFromMatrix(*other.Eq_b);
	}
	else
	{
		if (Eq_b) delete Eq_b; Eq_b=NULL;
	}

	if (other.Eq_c)
	{
		if (Eq_c==NULL)
			Eq_c = new ChMatrixDynamic<float>;
		Eq_c->CopyFromMatrix(*other.Eq_c);
	}
	else
	{
		if (Eq_c) delete Eq_c; Eq_c=NULL;
	}

	return *this;
}




void ChLcpConstraintThreeGeneric::SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b, ChLcpVariables* mvariables_c)
{
	if (!mvariables_a || !mvariables_b ||  !mvariables_c)
	{
		SetValid(false);
		return ;
	}

	SetValid(true);
	variables_a = mvariables_a;
	variables_b = mvariables_b;
	variables_c = mvariables_c;

	if (variables_a->Get_ndof())
	{
		if (!Cq_a)
			Cq_a = new ChMatrixDynamic<float>(1, variables_a->Get_ndof());
		else
			Cq_a->Resize(1, variables_a->Get_ndof());

		if (!Eq_a)
			Eq_a = new ChMatrixDynamic<float>(variables_a->Get_ndof(), 1);
		else
			Eq_a->Resize(variables_a->Get_ndof(), 1);
	}
	else
	{
		if (Cq_a) delete Cq_a; Cq_a = NULL;
		if (Eq_a) delete Eq_a; Eq_a = NULL;
	}

	if (variables_b->Get_ndof())
	{
		if (!Cq_b)
			Cq_b = new ChMatrixDynamic<float>(1, variables_b->Get_ndof());
		else
			Cq_b->Resize(1, variables_b->Get_ndof());

		if (!Eq_b)
			Eq_b = new ChMatrixDynamic<float>(variables_b->Get_ndof(), 1);
		else
			Eq_b->Resize(variables_b->Get_ndof(), 1);
	}
	else
	{
		if (Cq_b) delete Cq_b; Cq_b = NULL;
		if (Eq_b) delete Eq_b; Eq_b = NULL;
	}

	if (variables_c->Get_ndof())
	{
		if (!Cq_c)
			Cq_c = new ChMatrixDynamic<float>(1, variables_c->Get_ndof());
		else
			Cq_b->Resize(1, variables_c->Get_ndof());

		if (!Eq_c)
			Eq_c = new ChMatrixDynamic<float>(variables_c->Get_ndof(), 1);
		else
			Eq_c->Resize(variables_c->Get_ndof(), 1);
	}
	else
	{
		if (Cq_c) delete Cq_c; Cq_c = NULL;
		if (Eq_c) delete Eq_c; Eq_c = NULL;
	}
}




void ChLcpConstraintThreeGeneric::Update_auxiliary()
{
	//1- Assuming jacobians are already computed, now compute
	//   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
	if (variables_a->IsActive())
	if (variables_a->Get_ndof())
	{
		ChMatrixDynamic<float> mtemp1(variables_a->Get_ndof(), 1);
		mtemp1.CopyFromMatrixT(*Cq_a);
		variables_a->Compute_invMb_v(*Eq_a, mtemp1);
	}
	if (variables_b->IsActive())
	if (variables_b->Get_ndof())
	{
		ChMatrixDynamic<float> mtemp1(variables_b->Get_ndof(), 1);
		mtemp1.CopyFromMatrixT(*Cq_b);
		variables_b->Compute_invMb_v(*Eq_b, mtemp1);
	}
	if (variables_c->IsActive())
	if (variables_c->Get_ndof())
	{
		ChMatrixDynamic<float> mtemp1(variables_c->Get_ndof(), 1);
		mtemp1.CopyFromMatrixT(*Cq_c);
		variables_c->Compute_invMb_v(*Eq_c, mtemp1);
	}

	//2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
	ChMatrixDynamic<float> res(1,1);
	g_i = 0;
	if (variables_a->IsActive())
	if (variables_a->Get_ndof())
	{
		res.MatrMultiply(*Cq_a, *Eq_a);
		g_i = res(0,0);
	}
	if (variables_b->IsActive())
	if (variables_b->Get_ndof())
	{
		res.MatrMultiply(*Cq_b, *Eq_b);
		g_i += res(0,0);
	}
	if (variables_c->IsActive())
	if (variables_c->Get_ndof())
	{
		res.MatrMultiply(*Cq_c, *Eq_c);
		g_i += res(0,0);
	}

	//3- adds the constraint force mixing term (usually zero):
	if (cfm_i)
		g_i += cfm_i;
};







void ChLcpConstraintThreeGeneric::StreamOUT(ChStreamOutBinary& mstream)
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
 



void ChLcpConstraintThreeGeneric::StreamIN(ChStreamInBinary& mstream)
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


