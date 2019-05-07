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

#ifndef CHMATRIXNM_H
#define CHMATRIXNM_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// ChMatrixNM
///
/// Specialized matrix class having a pre-allocated NxM buffer of elements on stack.
///  This means that elements are put the stack when the matrix is created, and this
/// can be more efficient than heap allocation (but, please, do not use too large N or M
/// sizes, this is meant to work up to 10x10, for example; prefer ChMatrixDyamic for larger).
///  The NxM size must be known 'statically', at compile-time; however later at run-time
/// this matrix can be resized anyway (but for larger size than NxM, it falls back to
/// heap allocation). Note that if resizing is often required, it may be better
/// to create a ChMatrixDyamic instead, from the beginning.

template <class Real = double, int preall_rows = 3, int preall_columns = 3>
class ChMatrixNM : public ChMatrix<Real> {
  protected:
#ifdef CHRONO_HAS_AVX
    Real buffer[preall_rows * preall_columns + 3];
#else
    Real buffer[preall_rows * preall_columns];

#endif

  public:
    /// The default constructor builds a NxN matrix
    inline ChMatrixNM() {
        this->rows = preall_rows;
        this->columns = preall_columns;
        this->address = buffer;
    }

    /// Copy constructor
    inline ChMatrixNM(const ChMatrixNM<Real, preall_rows, preall_columns>& msource) {
        this->rows = preall_rows;
        this->columns = preall_columns;
        this->address = buffer;
        std::memcpy(this->address, msource.address, preall_rows * preall_columns * sizeof(Real));
    }

    /// Copy constructor from all types of base matrices (only with same size)
    template <class RealB>
    inline ChMatrixNM(const ChMatrix<RealB>& msource) {
        assert(msource.GetColumns() == preall_columns && msource.GetRows() == preall_rows);
        this->rows = preall_rows;
        this->columns = preall_columns;
        this->address = buffer;
        for (int i = 0; i < preall_rows * preall_columns; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// Destructor
    virtual inline ~ChMatrixNM() {}

    //
    // OPERATORS
    //

    /// Assignment operator (from generic other matrix, acceptable only if other matrix has same size)
    ChMatrixNM<Real, preall_rows, preall_columns>& operator=(const ChMatrix<Real>& matbis) {
        assert(matbis.GetColumns() == preall_columns && matbis.GetRows() == preall_rows);
        ChMatrix<Real>::operator=(matbis);
        return *this;
    }

    /// Negates sign of the matrix.
    /// Performance warning: a new object is created.
    ChMatrixNM<Real, preall_rows, preall_columns> operator-() const {
        ChMatrixNM<Real, preall_rows, preall_columns> result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this matrix and another matrix (of same size)
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrixNM<Real, preall_rows, preall_columns> operator+(const ChMatrix<RealB>& matbis) const {
        ChMatrixNM<Real, preall_rows, preall_columns> result;
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this matrix and another matrix (of same size).
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrixNM<Real, preall_rows, preall_columns> operator-(const ChMatrix<RealB>& matbis) const {
        ChMatrixNM<Real, preall_rows, preall_columns> result;
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this matrix and another ChMatrixNM matrix.
    /// This is optimized: it returns another ChMatrixMN because size of matbis is known statically.
    /// Performance warning: a new object is created.
    template <class RealB, int B_rows, int B_columns>
    ChMatrixNM<Real, preall_rows, B_columns> operator*(
        const ChMatrixNM<RealB, preall_columns, B_columns>& matbis) const {
        ChMatrixNM<Real, preall_rows, B_columns> result;
        result.MatrMultiply(*this, matbis);
        return result;
    }

    /// Multiplies this matrix and another generic matrix.
    /// Returns a ChMatrixDynamic matrix, because size of matbis is not known at compile time.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrixDynamic<Real> operator*(const ChMatrix<RealB>& matbis) const {
        ChMatrixDynamic<Real> result(this->rows, matbis.GetColumns());
        result.MatrMultiply(*this, matbis);
        return result;
    }

    /// Multiplies this matrix by a scalar value
    /// Performance warning: a new object is created.
    ChMatrixNM<Real, preall_rows, preall_columns> operator*(const Real factor) const {
        ChMatrixNM<Real, preall_rows, preall_columns> result(*this);
        result.MatrScale(factor);
        return result;
    }

    //
    // FUNCTIONS
    //

    /// Resize for this matrix is NOT SUPPORTED ! DO NOTHING!
    virtual inline void Resize(int nrows, int ncols) { assert((nrows == this->rows) && (ncols == this->columns)); }
};

}  // end namespace chrono

#endif
