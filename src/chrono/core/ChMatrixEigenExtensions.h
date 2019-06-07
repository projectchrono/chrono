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
//
// Chrono-specific extensions to Eigen::MatrixBase
//
// =============================================================================

#ifndef CHMATRIXEIGENEXTENSIONS_H
#define CHMATRIXEIGENEXTENSIONS_H

/// Set the diagonal elements to the specified value.
/// Note that the off-diagonal elements are not modified.
inline void fillDiagonal(Scalar val) {
    this->diagonal().array() = val;
}

/// Set all elememnts to the specified value.
inline void fill(Scalar val) {
    *this = Constant(rows(), cols(), val);
}

/// Test if this matrix is within given tolerance from specified matrix (element-wise).
template<typename OtherDerived>
inline bool equals(const MatrixBase<OtherDerived>& other, Scalar tolerance) {
    return (derived() - other).cwiseAbs().maxCoeff() <= tolerance;
}

/// Calculate the WRMS (wighted residual mean square) norm of a vector.
template <typename OtherDerived>
Scalar wrmsNorm(
    const MatrixBase<OtherDerived>& weights,
    typename std::enable_if<(MaxRowsAtCompileTime == 1 || MaxColsAtCompileTime == 1) &&
                                (OtherDerived::MaxRowsAtCompileTime == 1 || OtherDerived::MaxColsAtCompileTime == 1),
                            OtherDerived>::type* = 0) const {
    return numext::sqrt(derived().cwiseQuotient(weights).cwiseAbs2().sum() / derived().size());
}

/// Add a scalar to all elements.
const CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const Derived, const ConstantReturnType> operator+(
    const Scalar& val) const {
    return CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const Derived, const ConstantReturnType>(
        derived(), Constant(rows(), cols(), val));
}

/// Add a scalar to all elements.
friend const CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const ConstantReturnType, Derived> operator+(
    const Scalar& val,
    const MatrixBase<Derived>& mat) {
    return CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const ConstantReturnType, Derived>(
        Constant(mat.rows(), mat.cols(), val), mat.derived());
}

/// Paste a 3d vector at the specified location.
////inline void PasteVector(const chrono::ChVector<Scalar>& v, Index startRow, Index startCol) {
////    Matrix<Scalar, 3, 1> ev(v.x(), v.y(), v.z());
////    (*this).block<3, 1>(startRow, startCol) = ev;
////}

#endif
