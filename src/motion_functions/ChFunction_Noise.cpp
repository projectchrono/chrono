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
//   ChFunction_Noise.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Noise.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Noise> a_registration_noise;


ChFunction_Noise::ChFunction_Noise ()
{
	this->amp = 1;
	this->octaves = 2;
	this->amp_ratio = 0.5;
	this->freq = 1;
}

void ChFunction_Noise::Copy (ChFunction_Noise* source)
{
	this->amp = source->amp;
	this->freq = source->freq;
	this->amp_ratio = source->amp_ratio;
	this->octaves = source->octaves;
}

ChFunction* ChFunction_Noise::new_Duplicate ()
{
	ChFunction_Noise* m_func;
	m_func = new ChFunction_Noise;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Noise::Get_y      (double x)
{
	return ChNoise(x,amp,freq,octaves,amp_ratio);
}

void ChFunction_Noise::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << amp;
	mstream << freq;
	mstream << amp_ratio;
	mstream << octaves;
}

void ChFunction_Noise::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> amp;
	mstream >> freq;
	mstream >> amp_ratio;
	mstream >> octaves;
}

void ChFunction_Noise::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_NOISE  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
