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
// Authors: Radu Serban
// =============================================================================

#ifndef CHSTATE_H
#define CHSTATE_H

#include <cstdlib>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

// Forward reference
class ChIntegrable;

//// RADU
//// Consider deriving ChStateDelta from ChState
//// as this could reduce code duplication

/// Class for state of time-integrable objects.
/// This is a vector (one-column matrix) which extends ChVectorDynamic.
class ChState : public ChVectorDynamic<double> {
  public:
    explicit ChState(ChIntegrable* intgr = 0) : ChVectorDynamic<double>(1) { integrable = intgr; }

    explicit ChState(Eigen::Index size, ChIntegrable* intgr) : ChVectorDynamic<double>(size) { integrable = intgr; }

    explicit ChState(ChVectorConstRef vec, ChIntegrable* intgr) : ChVectorDynamic<double>(vec) { integrable = intgr; }

    /// Copy constructor
    ChState(const ChState& other) : ChVectorDynamic<double>(other) { integrable = other.integrable; }

    /// This method allows assigning Eigen expressions to ChStateDelta.
    template <typename OtherDerived>
    ChState& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>::operator=(other);
        return *this;
    }

    /*
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
    */

    /// Unary minus operator.
    ChState operator-() {
        ChState result(*this);
        result = result.ChVectorDynamic<>::operator-();
        return result;
    }

    /// Return the sum of this state and another vector.
    ChState operator+(ChVectorConstRef vec) const {
        ChState result(*this);
        result.ChVectorDynamic<>::operator+=(vec);
        return result;
    }

    /// Return the difference of this state vector and another vector.
    ChState operator-(ChVectorConstRef vec) const {
        ChState result(*this);
        result.ChVectorDynamic<>::operator-=(vec);
        return result;
    }

    /// Return this state vector scaled by the given value.
    ChState operator*(double factor) const {
        ChState result(*this);
        result.ChVectorDynamic<>::operator*=(factor);
        return result;
    }

    /// Reset to zeroes and (if needed) changes the size.
    void setZero(Eigen::Index size, ChIntegrable* intgr) {
        ChVectorDynamic<>::setZero(size);
        integrable = intgr;
    }

    ChIntegrable* GetIntegrable() const { return integrable; }

  private:
    ChIntegrable* integrable;
};

/// Class for incremental form of state of time-integrable objects.
/// Note that for many cases, this would be superfluous, because one could
/// do y_new = y_old + dydt*dt, where dydt is a ChState just like y and y_new, but there
/// are cases where such simple "+" operation between vectors is not practical, for instance
/// when integrating rotations in 3D space, where it is better to work with quaternions in y
/// and y_new, but with spinors/angular velocities/etc. in dydt; so dim(y) is not dim(dydt);
/// hence the need of this specific class for increments in states.
/// This is a vector (one-column matrix) which extends ChVectorDynamic.
class ChStateDelta : public ChVectorDynamic<double> {
  public:
    /// Constructors
    explicit ChStateDelta(ChIntegrable* intgr = 0) : ChVectorDynamic<double>(1) { integrable = intgr; }

    explicit ChStateDelta(Eigen::Index size, ChIntegrable* intgr) : ChVectorDynamic<double>(size) {
        integrable = intgr;
    }

    explicit ChStateDelta(ChVectorConstRef vec, ChIntegrable* intgr) : ChVectorDynamic<double>(vec) {
        integrable = intgr;
    }

    /// Copy constructor
    ChStateDelta(const ChStateDelta& other) : ChVectorDynamic<double>(other) { integrable = other.integrable; }

    /// This method allows assigning Eigen expressions to ChStateDelta.
    template <typename OtherDerived>
    ChStateDelta& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
        this->Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>::operator=(other);
        return *this;
    }

    /*
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
    */

    /// Unary minus operator.
    ChStateDelta operator-() {
        ChStateDelta result(*this);
        result = result.ChVectorDynamic<>::operator-();
        return result;
    }

    /// Scale this state by the given value.
    ChStateDelta& operator*=(double factor) {
        ChVectorDynamic<>::operator*=(factor);
        return *this;
    }

    /// Return the sum of this state and another vector.
    ChStateDelta operator+(ChVectorConstRef vec) const {
        ChStateDelta result(*this);
        result.ChVectorDynamic<>::operator+=(vec);
        return result;
    }

    /// Return the difference of this state vector and another vector.
    ChStateDelta operator-(ChVectorConstRef vec) const {
        ChStateDelta result(*this);
        result.ChVectorDynamic<>::operator-=(vec);
        return result;
    }

    /// Return this state vector scaled by the given value.
    ChStateDelta operator*(double factor) const {
        ChStateDelta result(*this);
        result.ChVectorDynamic<>::operator*=(factor);
        return result;
    }

    /// Reset to zeroes and (if needed) changes the size.
    void setZero(Eigen::Index size, ChIntegrable* intgr) {
        ChVectorDynamic<>::setZero(size);
        integrable = intgr;
    }

    ChIntegrable* GetIntegrable() const { return integrable; }

  private:
    ChIntegrable* integrable;
};

}  // end namespace chrono

#endif
