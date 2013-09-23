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

#ifndef CHFUNCT_SINE_H
#define CHFUNCT_SINE_H

//////////////////////////////////////////////////
//  
//   ChFunction_Sine.h
//
//   Function objects, 
//   as scalar functions of scalar variable y=f(t)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Base.h"


namespace chrono 
{

#define FUNCT_SINE		2



/// SINE FUNCTION:
/// y = sin (phase + w*x )     w=2*PI*freq

class ChApi ChFunction_Sine : public ChFunction
{
	CH_RTTI(ChFunction_Sine, ChFunction);
private:
	double amp;
	double phase;
	double freq;
	double w;
public:
	ChFunction_Sine () {phase =0; freq=1;  w=2*CH_C_PI*freq; amp = 1;};
	ChFunction_Sine (double m_phase, double m_freq, double m_amp)  {phase = m_phase; freq = m_freq; w=2*CH_C_PI*freq; amp = m_amp;};
	~ChFunction_Sine () {};
	void Copy (ChFunction_Sine* source);
	ChFunction* new_Duplicate ();

	void Set_phase (double m_phase) {phase = m_phase;};
	void Set_freq  (double m_freq)  {freq = m_freq; w=2*CH_C_PI*freq;};
	void Set_w	   (double m_w)		{w = m_w;		freq = w/(2*CH_C_PI);};
	void Set_amp   (double m_amp)   {amp = m_amp;}
	double Get_phase () {return phase;};
	double Get_freq ()  {return freq;};
	double Get_w	()  {return w;};
	double Get_amp ()   {return amp;}

	double Get_y      (double x) {return amp * (sin(phase + w*x)) ;};
	double Get_y_dx   (double x) {return amp * w*(cos(phase + w*x))	;};
	double Get_y_dxdx (double x) {return amp * -w*w*(sin(phase + w*x));};

	int Get_Type () {return (FUNCT_SINE);}

	OPT_VARIABLES_START
		"amp",
		"phase",
		"freq",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




} // END_OF_NAMESPACE____


#endif
