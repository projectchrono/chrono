//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPCONSTRAINTTWO_H
#define CHLCPCONSTRAINTTWO_H

//////////////////////////////////////////////////
//
//   ChLcpConstraintTwo.h
//
//    An 'easy' derived class for representing a
//   constraint between two ChLcpVariable items.
//   Used with for building sparse variational problems 
//   (VI/CCP/LCP/linear problems) described by 
//   a ChLcpSystemDescriptor
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChLcpConstraint.h"
#include "ChLcpVariables.h"

#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{


///  This class is inherited by the base ChLcpConstraint(),
/// which does almost nothing. So here this class implements
/// the functionality for a constrint between a COUPLE of TWO
/// objects of type ChLcpVariables(), and defines two constraint
/// matrices, whose column number automatically matches the number
/// of elements in variables vectors.
///  Before starting the LCP solver one must provide the proper
/// values in constraints (and update them if necessary), i.e.
/// must set at least the c_i and b_i values, and jacobians.

class ChApi ChLcpConstraintTwo : public ChLcpConstraint
{
	CH_RTTI(ChLcpConstraintTwo, ChLcpConstraint)

			//
			// DATA
			//

protected:

				/// The first  constrained object
	ChLcpVariables* variables_a;
				/// The second constrained object
	ChLcpVariables* variables_b;


public:

			//
			// CONSTRUCTORS
			//
						/// Default constructor
	ChLcpConstraintTwo()
					{
						variables_a = variables_b = NULL;
					};

						/// Copy constructor
	ChLcpConstraintTwo(const ChLcpConstraintTwo& other) : ChLcpConstraint(other)
					{
						variables_a = other.variables_a;
						variables_b = other.variables_b;
					}

	virtual ~ChLcpConstraintTwo()
					{
					};


					/// Assignment operator: copy from other object
	ChLcpConstraintTwo& operator=(const ChLcpConstraintTwo& other);



			//
			// FUNCTIONS
			//

				/// Access jacobian matrix
	virtual ChMatrix<double>* Get_Cq_a() =0;
				/// Access jacobian matrix
	virtual ChMatrix<double>* Get_Cq_b() =0;

				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<double>* Get_Eq_a() =0;
				/// Access auxiliary matrix (ex: used by iterative solvers)
	virtual ChMatrix<double>* Get_Eq_b() =0;

				/// Access the first variable object
	ChLcpVariables* GetVariables_a() {return variables_a;}
				/// Access the second variable object
	ChLcpVariables* GetVariables_b() {return variables_b;}

				/// Set references to the constrained objects, each of ChLcpVariables type,
				/// automatically creating/resizing jacobians if needed.
	virtual void SetVariables(ChLcpVariables* mvariables_a, ChLcpVariables* mvariables_b) =0;



			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	virtual void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	virtual void StreamOUT(ChStreamOutBinary& mstream);
};




} // END_OF_NAMESPACE____



#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  // END of ChLcpConstraintTwo.h
