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

#ifndef CHVARIABLESGENERIC_H
#define CHVARIABLESGENERIC_H

#include "chrono/solver/ChVariables.h"

namespace chrono {

/// Specialized class for representing a N-DOF item for a system, that is an item with mass matrix and associate
/// variables. The main difference from the base class ChVariables is that the base class does not create any mass
/// matrix, while this minimal specialization at least creates a NxN mass matrix. Of course a generic (uncompressed) NxN
/// matrix is used. This means that, for example, this class could  be used for 3D rigid bodies if N=6, however it would
/// be better to implement a more optimized class which does not create a full 6x6 matrix (since only few elements on
/// the diagonal would be different from 0 in case of rigid bodies), so use the ChVariablesBody in this case..

class ChApi ChVariablesGeneric : public ChVariables {
  public:
    ChVariablesGeneric(unsigned int dof = 1);
    virtual ~ChVariablesGeneric() {}

    /// Assignment operator: copy from other object
    ChVariablesGeneric& operator=(const ChVariablesGeneric& other);

    /// Access the inertia matrix
    ChMatrixDynamic<>& GetMass() { return Mmass; }

    /// Access the inverted inertia matrix
    ChMatrixDynamic<>& GetInvMass() { return inv_Mmass; }

    /// Compute the product of the inverse mass matrix by a given vector and store in result.
    /// This function must calculate `result = M^(-1) * vect` for a vector of same size as the variables state.
    virtual void ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Compute the product of the mass matrix by a given vector and increment result.
    /// This function must perform the operation `result += M * vect` for a vector of same size as the variables state.
    virtual void AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const override;

    /// Add the product of the mass submatrix by a given vector, scaled by ca, to result.
    /// Note: 'result' and 'vect' are system-level vectors of appropriate size. This function must index into these
    /// vectors using the offsets of each variable.
    virtual void AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const override;

    /// Add the diagonal of the mass matrix, as a vector scaled by ca, to result.
    /// Note: 'result' is a system-level vector of appropriate size. This function must index into this vector using the
    /// offsets of each variable.
    virtual void AddMassDiagonalInto(ChVectorRef result, const double ca) const override;

    /// Write the mass submatrix for these variables into the specified global matrix at the offsets of each variable.
    /// The masses will be scaled by the given factor 'ca'. The (start_row, start_col) pair specifies the top-left
    /// corner of the system-level mass matrix in the provided matrix. Assembling the system-level sparse matrix
    /// is required only if using a direct sparse solver or for debugging/reporting purposes.
    virtual void PasteMassInto(ChSparseMatrix& mat,
                               unsigned int start_row,
                               unsigned int start_col,
                               const double ca) const override;

  private:
    ChMatrixDynamic<double> Mmass;
    ChMatrixDynamic<double> inv_Mmass;
};

}  // end namespace chrono

#endif
