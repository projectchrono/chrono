//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChFunction_Mirror.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Mirror.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Mirror> a_registration_mirror;


void ChFunction_Mirror::Copy (ChFunction_Mirror* source)
{
	mirror_axis = source->mirror_axis;
	// fa = source->fa;		//***? shallow copy (now sharing same object)...
	fa = ChSharedPtr<ChFunction>(source->fa->new_Duplicate());	//***? ..or deep copy? make optional with flag?
}

ChFunction* ChFunction_Mirror::new_Duplicate ()
{
	ChFunction_Mirror* m_func;
	m_func = new ChFunction_Mirror;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Mirror::Get_y      (double x)
{
	if (x<= this->mirror_axis)
		return fa->Get_y(x);
	return fa->Get_y(2*this->mirror_axis - x);
}

void ChFunction_Mirror::Estimate_x_range (double& xmin, double& xmax)
{
	fa->Estimate_x_range(xmin,xmax);
}

int ChFunction_Mirror::MakeOptVariableTree(ChList<chjs_propdata>* mtree)
{
	int i=0;

	// inherit parent behaviour
	ChFunction::MakeOptVariableTree(mtree);

	// expand tree for children..

	chjs_propdata* mdataA = new chjs_propdata;
	strcpy(mdataA->propname, "fa");
	strcpy(mdataA->label,    mdataA->propname);
	mdataA->haschildren = TRUE;
	mtree->AddTail(mdataA);

	i += this->fa->MakeOptVariableTree(&mdataA->children);

	return i;
}


void ChFunction_Mirror::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << mirror_axis;

	mstream.AbstractWrite(this->fa.get_ptr()); 
	//***TODO*** better direct management of shared pointers serialization
}

void ChFunction_Mirror::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> mirror_axis;
	
	ChFunction* fooshared;
	mstream.AbstractReadCreate(&fooshared);	 // instance new
	fa = ChSharedPtr<ChFunction>(fooshared); // swap old shared to new shared, may delete old
	//***TODO*** better direct management of shared pointers serialization
}

void ChFunction_Mirror::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_MIRROR  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
