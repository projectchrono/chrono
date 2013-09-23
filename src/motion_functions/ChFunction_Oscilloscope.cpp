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
//   ChFunction_Oscilloscope.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Oscilloscope.h"


namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Oscilloscope> a_registration_oscilloscope;


void ChFunction_Oscilloscope::Copy (ChFunction_Oscilloscope* source)
{
	this->values = source->values;
	this->dx = source->dx;
	this->end_x = source->end_x;
	this->amount = source->amount;
	this->max_amount = source->max_amount;
}

ChFunction* ChFunction_Oscilloscope::new_Duplicate ()
{
	ChFunction_Oscilloscope* m_func;
	m_func = new ChFunction_Oscilloscope;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Oscilloscope::Extimate_x_range (double& xmin, double& xmax)
{
	xmin = this->end_x - this->dx*(this->amount-1);
	xmax = this->end_x;
	if (xmin >= xmax) xmax = xmin+ 0.5;
}


int ChFunction_Oscilloscope::AddLastPoint (double mx, double my)
{
	if (mx < end_x)
		this->Reset();
	this->end_x=mx;
	this->values.push_back(my);
	if (this->amount < this->max_amount)
		this->amount++;
	else
		this->values.pop_front();

	assert(this->values.size()==this->amount);
	return TRUE;
}



double ChFunction_Oscilloscope::Get_y      (double x)
{
	double y = 0;

	double start_x = this->end_x - this->dx*(this->amount-1);
	if (x > end_x) return 0;
	if (x < start_x) return 0;
	
	int i1 = (int)floor( (x - start_x)/this->dx );
	int i2 = i1+1;
	double p1x = start_x + dx*(double)i1;
	double p2x = start_x + dx*(double)i2;
	double p2y,p1y = 0;
	int count = 0;
	std::list<double>::iterator iter = values.begin();
	while(iter != values.end())
	{
		if (count == i1)
		{
			p2y = *iter;
			iter++;
			p1y = *iter;
			break;
		}
		count++;
		iter++;
	}

	y = ((x - p1x)*p2y + (p2x -x)*p1y)/(p2x - p1x);

	return y;
}




void ChFunction_Oscilloscope::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << dx;
	mstream << end_x;
	mstream << max_amount;
	mstream << amount;
	std::list<double>::iterator iter = values.begin();
	while(iter != values.end())
	{
		mstream << *iter;
		iter++;
	}
}

void ChFunction_Oscilloscope::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> dx;
	mstream >> end_x;
	mstream >> max_amount;
	mstream >> amount;
	values.clear();
	int i = 0;
	while(i < amount)
	{
		double mval;
		mstream >> mval;
		this->values.push_back(mval);
	}
}

void ChFunction_Oscilloscope::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_OSCILLOSCOPE \n";

	//***TO DO***
}




} // END_OF_NAMESPACE____


// eof
