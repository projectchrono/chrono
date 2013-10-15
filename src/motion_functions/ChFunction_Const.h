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

#ifndef CHFUNCT_CONST_H
#define CHFUNCT_CONST_H

//////////////////////////////////////////////////
//  
//   ChFunction_Const.h
//
//   Function object, 
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

#define FUNCT_CONST		0



/// THE CONSTANT FUNCTION:
/// 
///  y= C
/// 

class ChApi ChFunction_Const : public ChFunction
{
		// Chrono simulation of RTTI, needed for serialization 
		// Btw. should be "CH_RTTI(ChFunction_Const, ..",  but for backward compatibilty with streaming & class factory:
	CH_RTTI(ChFunction, ChFunction);

private:
	
	double C;

public:
	ChFunction_Const () {C = 0;};
	ChFunction_Const (double y_constant) {C = y_constant;};
	virtual ~ChFunction_Const () {};
	virtual void Copy (ChFunction_Const* source);
	virtual ChFunction* new_Duplicate ();

	virtual int Get_Type () {return (FUNCT_CONST);}				
	
				/// Set the constant C for the function, y=C.
	void   Set_yconst (double y_constant) {C = y_constant;};
				/// Get the constant C for the function, y=C.
	double Get_yconst () {return C;};

				// Override the Get_y(), Get_y_dx etc. functions with analytical formulas.

	virtual double Get_y      (double x) {return C;};
	virtual double Get_y_dx   (double x) {return 0;};
	virtual double Get_y_dxdx (double x) {return 0;};
				

				// Expose the parameters which can be used for optimizations, ex. as Javascript vars.

	OPT_VARIABLES_START
		"C",
	OPT_VARIABLES_END

				// Streaming

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);
};




} // END_OF_NAMESPACE____


#endif
