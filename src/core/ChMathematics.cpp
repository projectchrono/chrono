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
//   ChMathematics.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChMathematics.h"


namespace chrono 
{



// ANGLE UNITS CONVERSIONS


double ChAtan2(double mcos, double msin)
{
	double ret;
	if (fabs(mcos) < 0.707)
	{
		ret = acos(mcos);
		if (msin < 0.0)
			ret = -ret;
	}
	else
	{
		ret = asin(msin);
		if (mcos < 0.0)
			ret = CH_C_PI - ret;
	}
	return ret;
}



// OTHER

// Park-Miller hi-quality random generator 

#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define MASK 123459876

static long CH_PAseed = 123;

void ChSetRandomSeed (long newseed)
{
	if (CH_PAseed) CH_PAseed = newseed;
}

double ChRandom()
{
	long k;
	double ans;
	CH_PAseed ^= MASK;
	k=(CH_PAseed)/IQ;
	CH_PAseed = IA*(CH_PAseed-k*IQ)-IR*k;
	if (CH_PAseed <0) CH_PAseed += IM;
	ans=AM*(CH_PAseed);
	CH_PAseed ^= MASK;
	return ans;
}


void ChPeriodicPar(double& u, int closed)
{
	if (u < 0) { if (closed) u=u+1; else u=0;}
	if (u > 1) { if (closed) u=u-1; else u=1;}
}


double ChNoise(double x, double amp, double freq, int octaves, double amp_ratio)
{
	double ret = 0;
	long oldseed = CH_PAseed;
	double o_freq, o_amp, xA, xB, yA, yB, period;
	int iA, iB;

	o_freq = freq;
	o_amp = amp;

	for (int i=1; i<= octaves; i++)
	{
		
		period = 1.0/o_freq;
		xA = period * floor(x/period);
		xB = xA + period;
		iA = int (floor(x/period));
		iB = iA+1;
		ChSetRandomSeed((long)(iA+12345));
		ChRandom();ChRandom();ChRandom();  // just to puzzle the seed..
		yA = (ChRandom()-0.5) * o_amp;
		ChSetRandomSeed((long)(iB+12345));
		ChRandom();ChRandom();ChRandom();  // just to puzzle the seed..
		yB = (ChRandom()-0.5) * o_amp;
		// cubic noise interpolation from (xA,yA) to (xB,yB), with flat extremal derivatives
		ret += yA + (yB-yA)*( (3*(pow(((x-xA)/(xB-xA)),2))) - 2*(pow(((x-xA)/(xB-xA)),3)) );
		// for following octave, reduce amplitude...
		o_amp *= amp_ratio;
		o_freq *= 2.0;
	}
	// restore previous seed
	CH_PAseed = oldseed;
	return ret;
}


} // END_OF_NAMESPACE____


//////// 

