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
//   ChFunction_Poly345.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Poly345.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Poly345> a_registration_poly345;


void ChFunction_Poly345::Copy (ChFunction_Poly345* source)
{
	h = source->h;
	end = source->end;
}

ChFunction* ChFunction_Poly345::new_Duplicate ()
{
	ChFunction_Poly345* m_func;
	m_func = new ChFunction_Poly345;
	m_func->Copy(this);
	return (m_func);
}


double ChFunction_Poly345::Get_y      (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return h;
	double a = x/end;
	ret = h* ( 10*pow(a,3) - 15*pow(a,4) + 6*pow(a,5) );
	return ret;
}

double ChFunction_Poly345::Get_y_dx   (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = h*(1/end) * ( 30*pow(a,2) - 60*pow(a,3) + 30*pow(a,4) );
	return ret;
}

double ChFunction_Poly345::Get_y_dxdx (double x)
{
	double ret = 0;
	if (x<=0) return 0;
	if (x>=end) return 0;
	double a = x/end;
	ret = h*(1/(end*end)) * ( 60*a - 180*pow(a,2) + 120*pow(a,3) );
	return ret;
}

void ChFunction_Poly345::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << h;
	mstream << end;
}

void ChFunction_Poly345::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> h;
	mstream >> end;
}

void ChFunction_Poly345::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}







} // END_OF_NAMESPACE____


// eof
