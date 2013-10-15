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
//   ChLcpVariablesGeneric.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesGeneric.h"
#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono 
{


ChLcpVariablesGeneric& ChLcpVariablesGeneric::operator=(const ChLcpVariablesGeneric& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariables::operator=(other);

	// copy class data

	if (other.Mmass)
	{
		if (Mmass==NULL)
			Mmass = new ChMatrixDynamic<>;
		Mmass->CopyFromMatrix(*other.Mmass);
	}
	else
	{
		if (Mmass) delete Mmass;
		Mmass=NULL;
	}

	if (other.inv_Mmass)
	{
		if (inv_Mmass==NULL)
			inv_Mmass = new ChMatrixDynamic<>;
		inv_Mmass->CopyFromMatrix(*other.inv_Mmass);
	}
	else
	{
		if (inv_Mmass) delete inv_Mmass;
		inv_Mmass=NULL;
	}


	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesGeneric> a_registration_ChLcpVariablesGeneric;



} // END_OF_NAMESPACE____


