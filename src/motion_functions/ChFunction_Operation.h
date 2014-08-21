//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHFUNCT_OPERATION_H
#define CHFUNCT_OPERATION_H

//////////////////////////////////////////////////
//  
//   ChFunction_Operation.h
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

#define FUNCT_OPERATION 12


enum {
	ChOP_ADD = 0,
	ChOP_SUB,
	ChOP_MUL,
	ChOP_DIV,
	ChOP_POW,
	ChOP_MAX,
	ChOP_MIN,
	ChOP_MODULO,
	ChOP_FABS,
	ChOP_FUNCT,
};

/// OPERATION BETWEEN FUNCTIONS
/// (math operation between A and  B operands  
///   - fa = first operand function
///   - fb = second operand function

class ChApi ChFunction_Operation : public ChFunction
{
	CH_RTTI(ChFunction_Operation, ChFunction);
private:
	ChSharedPtr<ChFunction> fa;
	ChSharedPtr<ChFunction> fb;
	int op_type;		// see operation type IDS

public:
	ChFunction_Operation() 
		{
			op_type = ChOP_ADD; 
			fa = ChSharedPtr<ChFunction_Const>(new ChFunction_Const); 
			fb = ChSharedPtr<ChFunction_Const>(new ChFunction_Const); 
		}
	~ChFunction_Operation () {};
	void Copy (ChFunction_Operation* source);
	ChFunction* new_Duplicate ();

	void Set_optype  (int m_op)  {op_type = m_op;}
	int Get_optype () {return op_type;}

	void Set_fa  (ChSharedPtr<ChFunction> m_fa)  {fa = m_fa;}
	ChSharedPtr<ChFunction> Get_fa () {return fa;}
	
	void Set_fb  (ChSharedPtr<ChFunction> m_fb)  {fb = m_fb;}
	ChSharedPtr<ChFunction> Get_fb () {return fb;}

	double Get_y      (double x) ;
	//	double Get_y_dx   (double x) ;
	//	double Get_y_dxdx (double x) ;

	void Estimate_x_range (double& xmin, double& xmax);

	int Get_Type () {return (FUNCT_OPERATION);}
	
	int MakeOptVariableTree(ChList<chjs_propdata>* mtree);
	
	void StreamOUT(ChStreamOutAscii& mstream);
	void StreamIN(ChStreamInBinary& mstream);
	void StreamOUT(ChStreamOutBinary& mstream);

};




} // END_OF_NAMESPACE____


#endif
