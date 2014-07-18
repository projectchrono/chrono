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

#ifndef CHLCPVARIABLESNODE_H
#define CHLCPVARIABLESNODE_H

//////////////////////////////////////////////////
//
//   ChLcpVariablesNode.h
//
//    Specialized class for representing a mass matrix
//   and associate variables (3 element vector, ex.speed)
//   for a 3D point 'node'.
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpVariables.h"


namespace chrono
{

///    Specialized class for representing a 3-DOF item for a
///   LCP system, that is a 3D point node, with mass matrix and
///   associate variables (a 3 element vector, ex.speed)


class ChApi ChLcpVariablesNode :  public ChLcpVariables
{
	CH_RTTI(ChLcpVariablesNode, ChLcpVariables)

private:
			//
			// DATA		//


			void* user_data;

			double mass;		// mass value

public:

			//
			// CONSTRUCTORS
			//

	ChLcpVariablesNode() : ChLcpVariables(3)
				{
					user_data = 0;
					mass = 1.0;
				};

	virtual ~ChLcpVariablesNode()
				{
				};


				/// Assignment operator: copy from other object
	ChLcpVariablesNode& operator=(const ChLcpVariablesNode& other);


			//
			// FUNCTIONS
			//

				/// Get the mass associated with translation of node
	virtual double	GetNodeMass() const {return mass;};

				/// Set the mass associated with translation of node
	void SetNodeMass(const double mmass)
						{
							mass = mmass;
						}


				// IMPLEMENT PARENT CLASS METHODS


				/// The number of scalar variables in the vector qb
				/// (dof=degrees of freedom)
	virtual int Get_ndof() const {return 3;};


	virtual void* GetUserData() {return this->user_data;}
	virtual void SetUserData(void* mdata) {this->user_data = mdata;}



				/// Computes the product of the inverse mass matrix by a
				/// vector, and set in result: result = [invMb]*vect
    void Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const;
    void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const;

				/// Computes the product of the inverse mass matrix by a
				/// vector, and increment result: result += [invMb]*vect
    void Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const;
    void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const;


				/// Computes the product of the mass matrix by a
				/// vector, and set in result: result = [Mb]*vect
    void Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect) const;
    void Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect) const;

				/// Computes the product of the corresponding block in the 
				/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' and 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect.
    void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const;

				/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
				/// NOTE: the 'result' vector must already have the size of system unknowns, ie
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offset (that must be already updated) as index.
    void DiagonalAdd(ChMatrix<double>& result) const;

				/// Build the mass matrix (for these variables) storing
				/// it in 'storage' sparse matrix, at given column/row offset.
				/// Note, most iterative solvers don't need to know mass matrix explicitly.
				/// Optimised: doesn't fill unneeded elements except mass.
    void Build_M(ChSparseMatrix& storage, int insrow, int inscol);

};




} // END_OF_NAMESPACE____




#endif  // END of ChLcpVariablesBody.h
