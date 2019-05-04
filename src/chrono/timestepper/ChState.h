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

#ifndef CHSTATE_H
#define CHSTATE_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVectorDynamic.h"

namespace chrono {

// forward reference
class ChIntegrable;

/// Class for state of time-integrable objects.
/// This is a vector (one-column matrix), hence inherited from ChMatrixDynamic.
/// It is used in ChIntegrable, ChTimestepper, etc.

class ChState : public ChVectorDynamic<double> {
  public:
    /// Constructors
    explicit ChState(ChIntegrable* mint = 0) : ChVectorDynamic<double>(1) { integrable = mint; }

    explicit ChState(const int nrows, ChIntegrable* mint) : ChVectorDynamic<double>(nrows) { integrable = mint; }

    explicit ChState(const ChMatrixDynamic<double> matr, ChIntegrable* mint) : ChVectorDynamic<double>(matr) {
        integrable = mint;
    }

    /// Copy constructor
    ChState(const ChState& msource) : ChVectorDynamic<double>(msource) { integrable = msource.integrable; };

    /// Multiplies this matrix by a factor, in place
    template <class Real>
    ChState& operator*=(const Real factor) {
        MatrScale(factor);
        return *this;
    }

    /// Increments this matrix by another matrix, in place
    template <class RealB>
    ChState& operator+=(const ChMatrix<RealB>& matbis) {
        MatrInc(matbis);
        return *this;
    }

    /// Decrements this matrix by another matrix, in place
    template <class RealB>
    ChState& operator-=(const ChMatrix<RealB>& matbis) {
        MatrDec(matbis);
        return *this;
    }

    /// Negates sign of the matrix.
    /// Performance warning: a new object is created.
    ChState operator-() const {
        ChState result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChState operator+(const ChMatrix<RealB>& matbis) const {
        ChState result(this->rows, this->integrable);
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChState operator-(const ChMatrix<RealB>& matbis) const {
        ChState result(this->rows, this->integrable);
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this matrix by a scalar value.
    /// Performance warning: a new object is created.
    template <class Real>
    ChState operator*(const Real factor) const {
        ChState result(*this);
        result.MatrScale(factor);
        return result;
    }

    /// Reset to zeroes and (if needed) changes the size.
    void Reset(int nrows, ChIntegrable* mint) {
        ChVectorDynamic<>::Reset(nrows);
        integrable = mint;
    }

    ChIntegrable* GetIntegrable() const { return integrable; }

  private:
    ChIntegrable* integrable;
};

/// Class for incremental form of state of time-integrable objects.
/// Note that for many cases, this would be superfluous, because one could
/// do y_new = y_old + dydt*td, where dydt is a ChState just like y and y_new, but there
/// are cases where such simple "+" operations between vectors is not practical, for instance
/// when integrating rotations in 3D space, where it is better to work with quaternions in y
/// and y_new, but with spinors/angular velocities/etc. in dydt; so dim(y) is not dim(dydt);
/// hence the need of this specific class for increments in states.
/// This is a vector (one-column matrix), hence inherited from ChMatrixDynamic.
/// It is used in ChIntegrable, ChTimestepper, etc.

class ChStateDelta : public ChVectorDynamic<double> {
  public:
    /// Constructors
    explicit ChStateDelta(ChIntegrable* mint = 0) : ChVectorDynamic<double>(1) { integrable = mint; }

    explicit ChStateDelta(const int nrows, ChIntegrable* mint) : ChVectorDynamic<double>(nrows) { integrable = mint; }

    explicit ChStateDelta(const ChMatrixDynamic<double> matr, ChIntegrable* mint) : ChVectorDynamic<double>(matr) {
        integrable = mint;
    }

    /// Copy constructor
    ChStateDelta(const ChStateDelta& msource) : ChVectorDynamic<double>(msource) { integrable = msource.integrable; };

    /// Multiplies this matrix by a factor, in place
    template <class Real>
    ChStateDelta operator*=(const Real factor) {
        MatrScale(factor);
        return *this;
    }

    /// Increments this matrix by another matrix, in place
    template <class RealB>
    ChStateDelta operator+=(const ChMatrix<RealB>& matbis) {
        MatrInc(matbis);
        return *this;
    }

    /// Decrements this matrix by another matrix, in place
    template <class RealB>
    ChStateDelta operator-=(const ChMatrix<RealB>& matbis) {
        MatrDec(matbis);
        return *this;
    }

    /// Negates sign of the matrix.
    /// Performance warning: a new object is created.
    ChStateDelta operator-() const {
        ChStateDelta result(*this);
        result.MatrNeg();
        return result;
    }

    /// Sums this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChStateDelta operator+(const ChMatrix<RealB>& matbis) const {
        ChStateDelta result(this->rows, this->integrable);
        result.MatrAdd(*this, matbis);
        return result;
    }

    /// Subtracts this matrix and another matrix.
    /// Performance warning: a new object is created.
    template <class RealB>
    ChStateDelta operator-(const ChMatrix<RealB>& matbis) const {
        ChStateDelta result(this->rows, this->integrable);
        result.MatrSub(*this, matbis);
        return result;
    }

    /// Multiplies this matrix by a scalar value.
    /// Performance warning: a new object is created.
    template <class Real>
    ChStateDelta operator*(const Real factor) const {
        ChStateDelta result(*this);
        result.MatrScale(factor);
        return result;
    }

    /// Reset to zeroes and (if needed) changes the size.
    void Reset(int nrows, ChIntegrable* mint) {
        ChVectorDynamic<>::Reset(nrows);
        integrable = mint;
    }

    ChIntegrable* GetIntegrable() const { return integrable; }

  private:
    ChIntegrable* integrable;
};

}  // end namespace chrono

#endif
