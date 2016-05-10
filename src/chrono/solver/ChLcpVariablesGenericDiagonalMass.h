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

#ifndef CHLCPVARIABLESGENERICDIAGONALMASS_H
#define CHLCPVARIABLESGENERICDIAGONALMASS_H

#include "ChLcpVariables.h"
#include "core/ChVectorDynamic.h"

namespace chrono {

///    Specialized class for representing a N-DOF item for a
///   LCP system, that is an item with a diagonal mass matrix and
///   associate variables.

class ChApi ChLcpVariablesGenericDiagonalMass : public ChLcpVariables {
    CH_RTTI(ChLcpVariablesGenericDiagonalMass, ChLcpVariables)

  private:
    //
    // DATA
    //
    /// the data (qb, variables and fb, forces, already defined in base class)

    ChVectorDynamic<>* MmassDiag;
    int ndof;

  public:
    //
    // CONSTRUCTORS
    //
    ChLcpVariablesGenericDiagonalMass(int m_ndof = 1) : ChLcpVariables(m_ndof) {
        ndof = m_ndof;
        MmassDiag = new ChVectorDynamic<>(ndof);
        MmassDiag->FillElem(1.0);
    };

    virtual ~ChLcpVariablesGenericDiagonalMass() {
        if (MmassDiag)
            delete MmassDiag;
        MmassDiag = NULL;
    };

    /// Assignment operator: copy from other object
    ChLcpVariablesGenericDiagonalMass& operator=(const ChLcpVariablesGenericDiagonalMass& other);

    //
    // FUNCTIONS
    //

    /// Access the diagonal mass
    ChVectorDynamic<>& GetMassDiagonal() { return *MmassDiag; }

    // IMPLEMENT PARENT CLASS METHODS

    /// The number of scalar variables in the vector qb
    /// (dof=degrees of freedom)
    int Get_ndof() const { return this->ndof; };

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
    /// system matrix (ie. the mass matrix) by 'vect', scale by c_a, and add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offsets (that must be already updated) to know the
    /// indexes in result and vect.
    void MultiplyAndAdd(ChMatrix<double>& result, const ChMatrix<double>& vect, const double c_a) const;

    /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie
    /// the size of the total variables&constraints in the system; the procedure
    /// will use the ChVariable offset (that must be already updated) as index.
    void DiagonalAdd(ChMatrix<double>& result, const double c_a) const;

    /// Build the mass matrix (for these variables) scaled by c_a, storing
    /// it in 'storage' sparse matrix, at given column/row offset.
    /// Note, most iterative solvers don't need to know mass matrix explicitly.
	void Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) {
        for (int i = 0; i < MmassDiag->GetRows(); ++i) {
            storage.SetElement(insrow + i, inscol + i, c_a * (*MmassDiag)(i));
        }
    };

	

};

}  // END_OF_NAMESPACE____

#endif  // END of .h
