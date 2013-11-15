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
	virtual int Get_ndof() {return 3;};


	virtual void* GetUserData() {return this->user_data;}
	virtual void SetUserData(void* mdata) {this->user_data = mdata;}



				/// Computes the product of the inverse mass matrix by a
				/// vector, and set in result: result = [invMb]*vect
	virtual void Compute_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect)
					{
						assert (vect.GetRows()   == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						double inv_mass = 1.0/mass;
						result(0)= (float)inv_mass * vect(0);
						result(1)= (float)inv_mass * vect(1);
						result(2)= (float)inv_mass * vect(2);
					};
	virtual void Compute_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect)
					{
						assert (vect.GetRows()   == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						double inv_mass = 1.0/mass;
						result(0)= inv_mass * vect(0);
						result(1)= inv_mass * vect(1);
						result(2)= inv_mass * vect(2);
					};

				/// Computes the product of the inverse mass matrix by a
				/// vector, and increment result: result += [invMb]*vect
	virtual void Compute_inc_invMb_v(ChMatrix<float>& result, const ChMatrix<float>& vect)
					{
						assert (vect.GetRows()   == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						double inv_mass = 1.0/mass;
						result(0)+= (float)inv_mass * vect(0);
						result(1)+= (float)inv_mass * vect(1);
						result(2)+= (float)inv_mass * vect(2);
					};
	virtual void Compute_inc_invMb_v(ChMatrix<double>& result, const ChMatrix<double>& vect)
					{
						assert (vect.GetRows()   == Get_ndof());
						assert (result.GetRows() == Get_ndof());
						// optimized unrolled operations
						double inv_mass = 1.0/mass;
						result(0)+= inv_mass * vect(0);
						result(1)+= inv_mass * vect(1);
						result(2)+= inv_mass * vect(2);
					};


				/// Computes the product of the mass matrix by a
				/// vector, and set in result: result = [Mb]*vect
	virtual void Compute_inc_Mb_v(ChMatrix<float>& result, const ChMatrix<float>& vect)
					{
						assert (result.GetRows() == Get_ndof());
						assert (vect.GetRows()   == Get_ndof());
						// optimized unrolled operations
						result(0)+= (float)mass * vect(0);
						result(1)+= (float)mass * vect(1);
						result(2)+= (float)mass * vect(2);
					};
	virtual void Compute_inc_Mb_v(ChMatrix<double>& result, const ChMatrix<double>& vect)
					{
						assert (result.GetRows() == vect.GetRows());
						assert (vect.GetRows()   == Get_ndof());
						// optimized unrolled operations
						result(0)+= mass * vect(0);
						result(1)+= mass * vect(1);
						result(2)+= mass * vect(2);
					};

				/// Computes the product of the corresponding block in the 
				/// system matrix (ie. the mass matrix) by 'vect', and add to 'result'. 
				/// NOTE: the 'vect' and 'result' vectors must already have
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offsets (that must be already updated) to know the 
				/// indexes in result and vect.
	virtual void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect) const
					{
						assert(result.GetColumns()==1 && vect.GetColumns()==1);
						// optimized unrolled operations
						result(this->offset  )+= mass * vect(this->offset  );
						result(this->offset+1)+= mass * vect(this->offset+1);
						result(this->offset+2)+= mass * vect(this->offset+2);
					}

				/// Add the diagonal of the mass matrix (as a column vector) to 'result'.
				/// NOTE: the 'result' vector must already have the size of system unknowns, ie
				/// the size of the total variables&constraints in the system; the procedure
				/// will use the ChVariable offset (that must be already updated) as index.
	virtual void DiagonalAdd(ChMatrix<double>& result) const 
					{
						assert(result.GetColumns()==1);
						result(this->offset  )+= mass;
						result(this->offset+1)+= mass;
						result(this->offset+2)+= mass;
					}

				/// Build the mass matrix (for these variables) storing
				/// it in 'storage' sparse matrix, at given column/row offset.
				/// Note, most iterative solvers don't need to know mass matrix explicitly.
				/// Optimised: doesn't fill unneeded elements except mass.
	virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol)
					{
						storage.SetElement(insrow+0, inscol+0, mass);
						storage.SetElement(insrow+1, inscol+1, mass);
						storage.SetElement(insrow+2, inscol+2, mass);
					};

};




} // END_OF_NAMESPACE____




#endif  // END of ChLcpVariablesBody.h
