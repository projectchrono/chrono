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

#ifndef CHFUNCT_POLY_H
#define CHFUNCT_POLY_H

//////////////////////////////////////////////////
//  
//   ChFunction_Poly.h
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

#define FUNCT_POLY		4

#define POLY_COEFF_ARRAY  6



/// POLYNOMIAL FUNCTION:
/// y = a + bx + cx^2 + dx^3 + ...


class ChApi ChFunction_Poly : public ChFunction
{
	CH_RTTI(ChFunction_Poly, ChFunction);
private:
	double	coeff[POLY_COEFF_ARRAY]; // vector of coefficients
	int		order;					 // 0= const, 1= linear, etc... 
public:
	ChFunction_Poly ();
	~ChFunction_Poly () {};
	void Copy (ChFunction_Poly* source);
	ChFunction* new_Duplicate ();

	void Set_coeff (double m_coeff, int m_ind) {if (m_ind >= POLY_COEFF_ARRAY) {return;}; coeff[m_ind]= m_coeff;};
	void Set_order (int m_order)  {if (m_order >= POLY_COEFF_ARRAY) {m_order= (POLY_COEFF_ARRAY -1);} order= m_order;};
	double Get_coeff (int m_ind) {if (m_ind >= POLY_COEFF_ARRAY) {return 0;};  return coeff[m_ind];};
	int Get_order () {return order;};

	double Get_y      (double x);
	double Get_y_dx   (double x);
	double Get_y_dxdx (double x);

	int Get_Type () {return (FUNCT_POLY);}

	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);

	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};





} // END_OF_NAMESPACE____


#endif
