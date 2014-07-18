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

#ifndef CHLCPVARIABLESGENERIC_H
#define CHLCPVARIABLESGENERIC_H

//////////////////////////////////////////////////
//
//   ChLcpVariablesGeneric.h
//
//    Specialized class for representing a N-DOF item for a
//   LCP system, that is an item with mass matrix and
//   associate variables.
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpVariables.h"
#include "core/ChMemory.h" // must be after system's include (memory leak debugger).


namespace chrono
{

///    Specialized class for representing a N-DOF item for a
///   LCP system, that is an item with mass matrix and
///   associate variables.
///    The main difference from the base class ChLcpVariables
///   is that the base class does not create any mass matrix,
///   while this minimal specialization at least creates a
///   NxN mass matrix. Of course a generic (uncompressed) NxN
///   matrix is used. This means that, for example, this
///   class could  be used for 3D rigid bodies if N=6, however
///   it would be better to implement a more optimized class
///   which does not create a full 6x6 matrix (since only few
///   elements on the diagonal would be different from 0 in case
///   of rigid bodies), so use the ChLcpVariablesBody in this case..


class ChApi ChLcpVariablesGeneric :  public ChLcpVariables
{
	CH_RTTI(ChLcpVariablesGeneric, ChLcpVariables)

private:
			//
			// DATA
			//
				/// the data (qb, variables and fb, forces, already defined in base class)

			ChMatrixDynamic<>* Mmass;
			ChMatrixDynamic<>* inv_Mmass;
			int ndof;

public:

			//
			// CONSTRUCTORS
			//
	ChLcpVariablesGeneric(int m_ndof= 1)
							: ChLcpVariables(m_ndof)
				{
					ndof = m_ndof;
					Mmass = new ChMatrixDynamic<>(ndof, ndof);
					Mmass->SetIdentity();
					inv_Mmass = new ChMatrixDynamic<>(ndof, ndof);
					inv_Mmass->SetIdentity();
				};

	virtual ~ChLcpVariablesGeneric()
				{
					if (Mmass) delete Mmass; Mmass= NULL;
					if (inv_Mmass) delete inv_Mmass; inv_Mmass= NULL;
				};


				/// Assignment operator: copy from other object
	ChLcpVariablesGeneric& operator=(const ChLcpVariablesGeneric& other);


			//
			// FUNCTIONS
			//

				/// Access the inertia matrix
	ChMatrix<>& GetMass() {return *Mmass;}

				/// Access the inverted inertia matrix
	ChMatrix<>& GetInvMass() {return *inv_Mmass;}


				// IMPLEMENT PARENT CLASS METHODS


				/// The number of scalar variables in the vector qb
				/// (dof=degrees of freedom)
	int Get_ndof() const {return this->ndof;};

				/// Computes the product of the inverse mass matrix by a
				/// vector, and add to result: result = [invMb]*vect
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
	void Build_M(ChSparseMatrix& storage, int insrow, int inscol)
					{
						storage.PasteMatrix(Mmass, insrow, inscol);
					};

};




} // END_OF_NAMESPACE____


#include "core/ChMemorynomgr.h" // back to default new/delete/malloc/calloc etc. Avoid conflicts with system libs.


#endif  // END of ChLcpVariablesGeneric.h
