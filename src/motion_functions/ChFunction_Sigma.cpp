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
//   ChFunction_Sigma.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Sigma.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Sigma> a_registration_sigma;

void ChFunction_Sigma::Copy (ChFunction_Sigma* source)
{
	Set_start (source->start);
	Set_end (source->end);
	Set_amp (source->amp);
}

ChFunction* ChFunction_Sigma::new_Duplicate ()
{
	ChFunction_Sigma* m_func;
	m_func = new ChFunction_Sigma;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Sigma::Estimate_x_range (double& xmin, double& xmax)
{
	double mdx = end-start;
	xmin = start +mdx*0.1;
	xmax = end -mdx*0.1;
}

double ChFunction_Sigma::Get_y      (double x)
{
	double ret;
	double A = (end - start);
	if (x < start) return 0;
	if (x > end) return amp;
	else
	{
		ret = amp *( (3*(pow(((x-start)/A),2))) - 2*(pow(((x-start)/A),3)) );
	}
	return ret;
}

double ChFunction_Sigma::Get_y_dx   (double x)
{
	double ret;
	double A = (end - start);
	if ((x < start) || (x > end)) ret = 0;
	else
	{
		ret = amp * ( 6*((x-start) / pow(A,2)) - 6*(pow((x-start),2)/pow(A,3))  );
	}
	return ret;
}

double ChFunction_Sigma::Get_y_dxdx (double x)
{
	double ret;
	double A = (end - start);
	if ((x < start) || (x > end)) ret = 0;
	else
	{
		ret = amp * ( 6*(1/pow(A,2)) - 12*((x-start)/pow(A,3))  );
	}
	return ret;
}

void ChFunction_Sigma::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << start;
	mstream << end;
	mstream << amp;
}

void ChFunction_Sigma::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> start;
	mstream >> end;
	mstream >> amp;
}

void ChFunction_Sigma::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}




} // END_OF_NAMESPACE____


// eof
