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

#ifndef CHFUNCT_SIGMA_H
#define CHFUNCT_SIGMA_H

//////////////////////////////////////////////////
//  
//   ChFunction_Sigma.h
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

#define FUNCT_SIGMA     5



/// SIGMA FUNCTION:           
/// y = polynomial smooth ramp 

class ChApi ChFunction_Sigma : public ChFunction
{
	CH_RTTI(ChFunction_Sigma, ChFunction);
private:
	double amp;
	double start;
	double end;
public:
	ChFunction_Sigma () {amp =1; start=0;  end=1;}
	ChFunction_Sigma (double m_amp, double m_start, double m_end)  {start = m_start; end = m_end; amp = m_amp;};
	~ChFunction_Sigma () {};
	void Copy (ChFunction_Sigma* source);
	ChFunction* new_Duplicate ();

	void Set_start (double m_start) {start = m_start;}
	void Set_end   (double m_end)   {end = m_end;}
	void Set_amp   (double m_amp)   {amp = m_amp;}
	double Get_start ()   {return start;}
	double Get_end ()   {return end;}
	double Get_amp ()   {return amp;}

	double Get_y      (double x) ;
	double Get_y_dx   (double x) ;
	double Get_y_dxdx (double x) ;

	double Get_Ca_pos () {return 6.0;};
	double Get_Ca_neg () {return 6.0;};
	double Get_Cv () {return 1.5;};

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_SIGMA);}

	OPT_VARIABLES_START
		"start",
		"end",
		"amp",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



} // END_OF_NAMESPACE____


#endif
