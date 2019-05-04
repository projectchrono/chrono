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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHVECTORDYNAMIC_H
#define CHVECTORDYNAMIC_H

#include <cmath>

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChStream.h"

namespace chrono {

/// ChVectorDynamic
///
///  Specialized 'resizeable' vector class where the elements are allocated on heap.
/// This is a 'one column matrix', so one could use ChMatrixDynamic<>(nrows,1) too,
/// but this makes syntax more clear.
///  The size of the vector can be known even at compile-time, and the vector can
/// be freely resized also after creation. The size is unlimited (until you have memory).
///  Although this is a generic type of vector, please do not use it for 3D vectors
/// because there is already the specific ChVector<> class that implements lot of features
/// for 3D vectors.

template <class Real = double>
class ChVectorDynamic : public ChMatrix<Real> {
  public:
    /// The default constructor builds a 1 element vector.
    ChVectorDynamic() {
        this->rows = 1;
        this->columns = 1;
        this->address = new Real[1];
    }

    /// The constructor for a generic n sized vector.
    ChVectorDynamic(int rows) {
        assert(rows >= 0);
        this->rows = rows;
        this->columns = 1;
        this->address = new Real[rows];
    }

    /// Copy constructor
    ChVectorDynamic(const ChVectorDynamic<Real>& msource) {
        this->rows = msource.GetRows();
        this->columns = 1;
        this->address = new Real[this->rows];
        std::memcpy(this->address, msource.address, this->rows * sizeof(Real));
    }

    /// Copy constructor from all types of base matrices
    /// Note! Assumes that the source matrix has one column only! There is no run-time check for the one-column sizing.
    template <class RealB>
    ChVectorDynamic(const ChMatrix<RealB>& msource) {
        assert(msource.GetColumns() == 1);
        this->rows = msource.GetRows();
        this->columns = 1;
        this->address = new Real[this->rows];
        for (int i = 0; i < this->rows; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// Delete allocated heap mem.
    virtual ~ChVectorDynamic() { delete[] this->address; }

    /// Return the length of the vector
    int GetLength() const { return this->rows; }

    /// Assignment operator (from generic other matrix, it always work)
    virtual ChVectorDynamic<Real>& operator=(const ChMatrix<Real>& matbis) override {
        ChMatrix<Real>::operator=(matbis);
        return *this;
    }

    /// Negates vector.
    ChVectorDynamic<Real> operator-() const {
        ChVectorDynamic<Real> result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this vector and another vector.
    template <class RealB>
    ChVectorDynamic<Real> operator+(const ChMatrix<RealB>& matbis) const {
        ChVectorDynamic<Real> result(this->rows);
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this vector and another vector.
    template <class RealB>
    ChVectorDynamic<Real> operator-(const ChMatrix<RealB>& matbis) const {
        ChVectorDynamic<Real> result(this->rows);
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this vector by a scalar value.
    ChVectorDynamic<Real> operator*(const Real factor) const {
        ChVectorDynamic<Real> result(*this);
        result.MatrScale(factor);
        return result;
    }

    /// Reallocate memory for a new size.
    virtual void Resize(int nrows) {
        assert(nrows >= 0);
        if (nrows != this->rows) {
            this->rows = nrows;
            this->columns = 1;
            delete[] this->address;
            this->address = new Real[this->rows];
        }
    }

    virtual void Resize(int nrows, int ncols) override {
        assert(ncols == 1);
        Resize(nrows);
    }

    /// Reset to zeroes and (if needed) changes the size to have row and col
    void Reset(int nrows) {
        Resize(nrows);
        std::memset(this->address, 0, this->rows * sizeof(Real));
    }

    /// Resets to zero
    virtual void Reset() override { std::memset(this->address, 0, this->rows * sizeof(Real)); }

    /// Calculate the WRMS (weighted root-mean-square) norm of this vector.
    ///     norm = sqrt{ sum{(w_i * x_i)^2} / N}
    Real NormWRMS(const ChVectorDynamic<Real>& w) {
        if (this->rows == 0)
            return 0;

        Real sum = 0;
        for (int i = 0; i < this->rows; ++i) {
            Real prod = this->address[i] * w.address[i];
            sum += prod * prod;
        }
        return std::sqrt(sum / this->rows);
    }
};

}  // end namespace chrono

#endif
