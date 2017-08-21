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

#ifndef CHMATRIXDYNAMIC_H
#define CHMATRIXDYNAMIC_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

//
// FAST MACROS TO SPEEDUP CODE
//

//#define SetZero(els) {for (int i=0; i<els; ++i) this->address[i]=0; }
//#define ElementsCopy(to,from,els) {for (int i=0; i<els; ++i) to[i]=(Real)from[i]; }

///
/// ChMatrixDynamic
///
///  Specialized 'resizeable' matrix class where the elements are allocated on heap.
/// The size of the matrix can be known even at compile-time, and the matrix can
/// be freely resized also after creation. The size is unlimited (until you have memory).
///  Although this is the most generic type of matrix, please do not use it
/// where you know in advance its size because there are more efficient
/// types for those matrices with 'static' size (for example, 3x3 rotation
/// matrices are faster if created as ChMatrix33).

template <class Real>
class ChMatrixDynamic : public ChMatrix<Real> {
  private:
    //
    // DATA
    //

    /// [simply use the  "Real* address" pointer of the base class

  public:
    //
    // CONSTRUCTORS
    //

    /// The default constructor builds a 3x3 matrix.
    ChMatrixDynamic() {
        this->rows = 3;
        this->columns = 3;
        this->address = new Real[9];
        for (int i = 0; i < 9; ++i)
            this->address[i] = 0;
    }

    /// Copy constructor
    ChMatrixDynamic(const ChMatrixDynamic<Real>& msource) {
        this->rows = msource.GetRows();
        this->columns = msource.GetColumns();
        this->address = new Real[this->rows * this->columns];
        // ElementsCopy(this->address, msource.GetAddress(), this->rows*this->columns);
        for (int i = 0; i < this->rows * this->columns; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// Copy constructor from all types of base matrices
    template <class RealB>
    ChMatrixDynamic(const ChMatrix<RealB>& msource) {
        this->rows = msource.GetRows();
        this->columns = msource.GetColumns();
        this->address = new Real[this->rows * this->columns];
        // ElementsCopy(this->address, msource.GetAddress(), this->rows*this->columns);
        for (int i = 0; i < this->rows * this->columns; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// The constructor for a generic nxm matrix.
    /// Rows and columns cannot be zero or negative.
    ChMatrixDynamic(const int row, const int col) {
        assert(row >= 0 && col >= 0);
        this->rows = row;
        this->columns = col;
#ifdef CHRONO_HAS_AVX
        this->address = new Real[row * col + 3];
#else
        this->address = new Real[row * col];

#endif
        // SetZero(row*col);
        for (int i = 0; i < this->rows * this->columns; ++i)
            this->address[i] = 0;
    }

    /// Destructor
    /// Delete allocated heap mem.
    virtual ~ChMatrixDynamic() { delete[] this->address; }

    //
    // OPERATORS
    //

    /// Assignment operator (from generic other matrix, it always work)
    ChMatrixDynamic<Real>& operator=(const ChMatrix<Real>& matbis) {
        ChMatrix<Real>::operator=(matbis);
        return *this;
    }

    /// Negates sign of the matrix.
    /// Performance warning: a new object is created.
    ChMatrixDynamic<Real> operator-() const {
        ChMatrixDynamic<Real> result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrixDynamic<Real> operator+(const ChMatrix<RealB>& matbis) const {
        ChMatrixDynamic<Real> result(this->rows, this->columns);
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrixDynamic<Real> operator-(const ChMatrix<RealB>& matbis) const {
        ChMatrixDynamic<Real> result(this->rows, this->columns);
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChMatrixDynamic<Real> operator*(const ChMatrix<RealB>& matbis) const {
        ChMatrixDynamic<Real> result(this->rows, matbis.GetColumns());
        result.MatrMultiply(*this, matbis);
        return result;
    }

    /// Multiplies this matrix by a scalar value.
    /// Performance warning: a new object is created.
    ChMatrixDynamic<Real> operator*(const Real factor) const {
        ChMatrixDynamic<Real> result(*this);
        result.MatrScale(factor);
        return result;
    }

    //
    // FUNCTIONS
    //

    /// Reallocate memory for a new size.
    virtual void Resize(int nrows, int ncols) {
        assert(nrows >= 0 && ncols >= 0);
        if ((nrows != this->rows) || (ncols != this->columns)) {
            this->rows = nrows;
            this->columns = ncols;
            delete[] this->address;
            this->address = new Real[this->rows * this->columns];
            // SetZero(this->rows*this->columns);
            for (int i = 0; i < this->rows * this->columns; ++i)
                this->address[i] = 0;
        }
    }
};

}  // end namespace chrono

#endif
