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

#ifndef CHFUNCT_REPEAT_H
#define CHFUNCT_REPEAT_H

//////////////////////////////////////////////////
//  
//   ChFunction_Repeat.h
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
#include "ChFunction_Const.h"

namespace chrono 
{

#define FUNCT_REPEAT	19


/// REPEAT FUNCTION:
/// y = __/__/__/ 
///
/// Repeats a 'window' of a function, periodically.

class ChApi ChFunction_Repeat : public ChFunction
{
	CH_RTTI(ChFunction_Repeat, ChFunction);
private:
	ChFunction* fa;
	double window_start;			// window begin position
	double window_length;			// window length
public:
	ChFunction_Repeat() {window_start=0;window_length=1;  fa = new ChFunction_Const;}
	~ChFunction_Repeat () {if (fa) delete fa;};
	void Copy (ChFunction_Repeat* source);
	ChFunction* new_Duplicate ();

	void Set_window_start  (double m_v)  {window_start = m_v;}
	double Get_window_start () {return window_start;}
	void Set_window_length  (double m_v)  {window_length = m_v;}
	double Get_window_length () {return window_length;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	int Get_Type () {return (FUNCT_REPEAT);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	OPT_VARIABLES_START
		"window_start",
		"window_length",
	OPT_VARIABLES_END

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};



} // END_OF_NAMESPACE____


#endif
