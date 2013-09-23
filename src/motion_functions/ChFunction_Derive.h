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

#ifndef CHFUNCT_DERIVE_H
#define CHFUNCT_DERIVE_H

//////////////////////////////////////////////////
//  
//   ChFunction_Derive.h
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


#define FUNCT_DERIVE	16


/// DERIVATIVE OF A FUNCTION:
///  y = df/dx  
///
/// Uses a numerical differentiation method to compute the derivative
/// of a generic function.

class ChApi ChFunction_Derive : public ChFunction
{
	CH_RTTI(ChFunction_Derive, ChFunction);
private:
	ChFunction* fa;
	int order;			// 1= derive one time, 2= two times, etc.

public:
	ChFunction_Derive() {order = 1; fa = new ChFunction_Const;}
	~ChFunction_Derive () {if (fa) delete fa;};
	void Copy (ChFunction_Derive* source);
	ChFunction* new_Duplicate ();

	void Set_order  (int m_order)  {order = m_order;}
	int Get_order () {return order;}

	void Set_fa  (ChFunction* m_fa)  {fa = m_fa;}
	ChFunction* Get_fa () {return fa;}
	
	double Get_y      (double x) ;

	void Extimate_x_range (double& xmin, double& xmax);
	
	int Get_Type () {return (FUNCT_DERIVE);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




} // END_OF_NAMESPACE____


#endif
