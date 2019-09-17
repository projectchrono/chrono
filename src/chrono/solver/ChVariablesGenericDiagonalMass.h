// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHVARIABLESGENERICDIAGONALMASS_H
#define CHVARIABLESGENERICDIAGONALMASS_H

#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Specialized class for representing a N-DOF item for a system, that is an item with a diagonal mass matrix and
/// associated variables.
class ChApi ChVariablesGenericDiagonalMass : public ChVariables {
  private:
    ChVectorDynamic<double> MmassDiag;
    int ndof;

  public:
    ChVariablesGenericDiagonalMass(int m_ndof = 1);
    virtual ~ChVariablesGenericDiagonalMass() {}

    /// Assignment operator: copy from other object
    ChVariablesGenericDiagonalMass& operator=(const ChVariablesGenericDiagonalMass& other);

    /// Access the diagonal mass
    ChVectorDynamic<>& GetMassDiagonal() { return MmassDiag; }

    /// The number of scalar variables in the vector qb (dof=degrees of freedom)
    virtual int Get_ndof() const override { return this->ndof; }

    /// Computes the product of the inverse mass matrix by a vector, and add to result: result = [invMb]*vect
    virtual void Compute_invMb_v(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Computes the product of the inverse mass matrix by a vector, and increment result: result += [invMb]*vect
    virtual void Compute_inc_invMb_v(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Computes the product of the mass matrix by a vector, and set in result: result = [Mb]*vect
    virtual void Compute_inc_Mb_v(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Computes the product of the corresponding block in the system matrix (ie. the mass matrix) by 'vect', scale by
    /// c_a, and add to 'result'.
    /// NOTE: the 'vect' and 'result' vectors must already have the size of the total variables&constraints in the
    /// system; the procedure will use the ChVariable offsets (that must be already updated) to know the indexes in
    /// result and vect.
    virtual void MultiplyAndAdd(ChVectorRef result, ChVectorConstRef vect, const double c_a) const override;

    /// Add the diagonal of the mass matrix scaled by c_a, to 'result'.
    /// NOTE: the 'result' vector must already have the size of system unknowns, ie the size of the total variables &
    /// constraints in the system; the procedure will use the ChVariable offset (that must be already updated) as index.
    virtual void DiagonalAdd(ChVectorRef result, const double c_a) const override;

    /// Build the mass matrix (for these variables) scaled by c_a, storing
    /// it in 'storage' sparse matrix, at given column/row offset.
    /// Note, most iterative solvers don't need to know mass matrix explicitly.
    virtual void Build_M(ChSparseMatrix& storage, int insrow, int inscol, const double c_a) override;
};

}  // end namespace chrono

#endif
