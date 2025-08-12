// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_SOA_VARIABLES_H
#define CH_SOA_VARIABLES_H

#include "chrono/solver/ChVariables.h"
#include "chrono/utils/ChUtils.h"

namespace chrono {

/// Class for representing an SOA mobilized body with mass matrix and associated variables.
template <int dof>
class ChSoaVariables : public ChVariables {
  public:
    ChSoaVariables() : ChVariables(dof) {}
    ~ChSoaVariables() {}

    /// Assignment operator: copy from other object
    ChSoaVariables& operator=(const ChSoaVariables& other) {
        if (&other == this)
            return *this;
        ChVariables::operator=(other);
        return *this;
    }

    /// Compute the product of the inverse mass matrix by a given vector and store in result.
    /// This function must calculate `result = M^(-1) * vect` for a vector of same size as the variables state.
    virtual void ComputeMassInverseTimesVector(ChVectorRef result, ChVectorConstRef vect) const override {
        //// TODO
        ChAssertAlways(false);
    }

    /// Compute the product of the mass matrix by a given vector and increment result.
    /// This function must perform the operation `result += M * vect` for a vector of same size as the variables state.
    virtual void AddMassTimesVector(ChVectorRef result, ChVectorConstRef vect) const override {
        //// TODO
        ChAssertAlways(false);
    }

    /// Add the product of the mass submatrix by a given vector, scaled by ca, to result.
    /// Note: 'result' and 'vect' are system-level vectors of appropriate size. This function must index into these
    /// vectors using the offsets of each variable.
    virtual void AddMassTimesVectorInto(ChVectorRef result, ChVectorConstRef vect, const double ca) const override {
        //// TODO
        ChAssertAlways(false);
    }

    /// Add the diagonal of the mass matrix, as a vector scaled by ca, to result.
    /// Note: 'result' is a system-level vector of appropriate size. This function must index into this vector using the
    /// offsets of each variable.
    virtual void AddMassDiagonalInto(ChVectorRef result, const double ca) const override {
        //// TODO
        ChAssertAlways(false);
    }

    /// Write the mass submatrix for these variables into the specified global matrix at the offsets of each variable.
    /// The masses will be scaled by the given factor 'ca'. The (start_row, start_col) pair specifies the top-left
    /// corner of the system-level mass matrix in the provided matrix. Assembling the system-level sparse matrix
    /// is required only if using a direct sparse solver or for debugging/reporting purposes.
    virtual void PasteMassInto(ChSparseMatrix& mat,
                               unsigned int start_row,
                               unsigned int start_col,
                               const double ca) const override {
        //// TODO
        ChAssertAlways(false);
    }

  private:
};

}  // end namespace chrono

#endif
