//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 1996, 2005, 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// Author: A.Tasora

#ifndef CHVECTORDYNAMIC_H
#define CHVECTORDYNAMIC_H

#include "core/ChCoordsys.h"
#include "core/ChStream.h"
#include "core/ChException.h"
#include "core/ChMatrix.h"
namespace chrono {

//
// FAST MACROS TO SPEEDUP CODE
//

///
/// ChVectorDynamic
///
///  Specialized 'resizeable' vector class where the elements are allocated on heap.
/// This is a 'one column matrix', so one could use ChMatrixDynamic<>(nrows,1) too,
/// but this makes syntax more clear.
///  The size of the vector can be known even at compile-time, and the vector can
/// be freely resized also after creation. The size is unlimited (until you have memory).
///  Although this is a generic type of vector, please do not use it for 3D vectors
/// beause there is already the specific ChVector<> class that implements lot of features
/// for 3D vectors.

template <class Real = double>
class ChVectorDynamic : public ChMatrix<Real> {
  private:
    //
    // DATA
    //

    /// [simply use the  "Real* address" pointer of the base class

  public:
    //
    // CONSTRUCTORS
    //

    /// The default constructor builds a 1 element vector.
    ChVectorDynamic() {
        this->rows = 1;
        this->columns = 1;
        this->address = new Real[1];
        // SetZero(1);
        this->address[0] = 0;
    }

    /// The constructor for a generic n sized vector.
    /// Rows cannot be zero or negative.
    ChVectorDynamic(const int rows) {
        assert(rows >= 0);
        this->rows = rows;
        this->columns = 1;
        this->address = new Real[rows];
        // SetZero(rows);
        for (int i = 0; i < this->rows; ++i)
            this->address[i] = 0;
    }

    /// Copy constructor
    ChVectorDynamic(const ChVectorDynamic<Real>& msource) {
        this->rows = msource.GetRows();
        this->columns = 1;
        this->address = new Real[this->rows];
        // ElementsCopy(this->address, msource.GetAddress(), this->rows);
        for (int i = 0; i < this->rows; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// Copy constructor from all types of base matrices
    /// Note! Assumes that the source matrix has one column only! There is no run-time check for the one-column sizing.
    template <class RealB>
    ChVectorDynamic(const ChMatrix<RealB>& msource) {
        assert(msource.GetColumns() == 1);
        this->rows = msource.GetRows();
        this->columns = 1;
        this->address = new Real[this->rows];
        // ElementsCopy(this->address, msource.GetAddress(), this->rows);
        for (int i = 0; i < this->rows; ++i)
            this->address[i] = (Real)msource.GetAddress()[i];
    }

    /// Destructor
    /// Delete allocated heap mem.
    virtual ~ChVectorDynamic() { delete[] this->address; }

    //
    // OPERATORS
    //

    /// Assignment operator (from generic other matrix, it always work)
    ChVectorDynamic<Real>& operator=(const ChMatrix<Real>& matbis) {
        ChMatrix<Real>::operator=(matbis);
        return *this;
    }

    /// Negates sign of the matrix.
    /// Performance warning: a new object is created.
    ChVectorDynamic<Real> operator-() const {
        ChVectorDynamic<Real> result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this vector and another vector.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChVectorDynamic<Real> operator+(const ChMatrix<RealB>& matbis) const {
        ChVectorDynamic<Real> result(this->rows);
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this vector and another vector.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChVectorDynamic<Real> operator-(const ChMatrix<RealB>& matbis) const {
        ChVectorDynamic<Real> result(this->rows);
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this vector by a scalar value.
    /// Performance warning: a new object is created.
    ChVectorDynamic<Real> operator*(const Real factor) const {
        ChVectorDynamic<Real> result(*this);
        result.MatrScale(factor);
        return result;
    }

    //
    // FUNCTIONS
    //

    /// Reallocate memory for a new size.
    virtual void Resize(int nrows) {
        assert(nrows >= 0);
        if (nrows != this->rows) {
            this->rows = nrows;
            this->columns = 1;
            delete[] this->address;
            this->address = new Real[this->rows];
            // SetZero(this->rows);
            //#pragma omp parallel for if (this->rows>CH_OMP_MATR)
            for (int i = 0; i < this->rows; ++i)
                this->address[i] = 0;
        }
    }

    virtual void Resize(int nrows, int ncols) {
        assert(ncols == 1);
        Resize(nrows);
    }

    /// Reset to zeroes and (if needed) changes the size to have row and col
    void Reset(int nrows) {
        Resize(nrows);
// SetZero(rows);
#pragma omp parallel for if (this->rows > CH_OMP_MATR)
        for (int i = 0; i < this->rows; ++i)
            this->address[i] = 0;
    }

    /// Resets the matrix to zero  (warning: simply sets memory to 0 bytes!)
    void Reset() {
// SetZero(rows*columns); //memset(address, 0, sizeof(Real) * rows * columns);
#pragma omp parallel for if (this->rows > CH_OMP_MATRLIGHT)
        for (int i = 0; i < this->rows; ++i)
            this->address[i] = 0;
    }
};

}  // END_OF_NAMESPACE____

#endif  // END of ChMatrix.h
