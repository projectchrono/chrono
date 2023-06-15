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
// Authors: Radu Serban, Alessandro Tasora
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
    derived().diagonal().array() = val;
}

/// Set all coefficients to random values, uniformly distributed in specified range.
inline void fillRandom(Scalar min, Scalar max) {
    derived() = (derived().Random(rows(), cols()) + 1.) * 0.5 * (max - min) + min;
}

/// Test if this matrix is within given tolerance from specified matrix (element-wise).
template<typename OtherDerived>
inline bool equals(const MatrixBase<OtherDerived>& other, Scalar tolerance) {
    return (derived() - other).cwiseAbs().maxCoeff() <= tolerance;
}

/// Calculate the WRMS (weighted residual mean square) norm of a vector.
template <typename OtherDerived>
Scalar wrmsNorm(
    const MatrixBase<OtherDerived>& weights,
    typename std::enable_if<(MaxRowsAtCompileTime == 1 || MaxColsAtCompileTime == 1) &&
                                (OtherDerived::MaxRowsAtCompileTime == 1 || OtherDerived::MaxColsAtCompileTime == 1),
                            OtherDerived>::type* = 0) const {
    if (derived().size() == 0)
        return 0;
    return numext::sqrt(derived().cwiseProduct(weights).cwiseAbs2().sum() / derived().size());
}

/// Add a scalar to all elements.
const CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const Derived, const ConstantReturnType> operator+(
    const Scalar& val) const {
    return CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const Derived, const ConstantReturnType>(
        derived(), derived().Constant(rows(), cols(), val));
}

/// Add a scalar to all elements.
friend const CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const ConstantReturnType, Derived> operator+(
    const Scalar& val,
    const MatrixBase<Derived>& mat) {
    return CwiseBinaryOp<internal::scalar_sum_op<Scalar>, const ConstantReturnType, Derived>(
        mat.derived().Constant(mat.rows(), mat.cols(), val), mat.derived());
}

void ArchiveOut(chrono::ChArchiveOut& marchive) {
    // suggested: use versioning
	marchive.VersionWrite<chrono::ChMatrix_dense_version_tag>(); // btw use the ChMatrixDynamic version tag also for all other templates.

    // stream out all member data

    if (chrono::ChArchiveAsciiDump* mascii = dynamic_cast<chrono::ChArchiveAsciiDump*>(&marchive)) {
        // CUSTOM row x col 'intuitive' table-like log when using ChArchiveAsciiDump:
        mascii->indent();
        mascii->GetStream()->operator<<((int)derived().rows());
        mascii->GetStream()->operator<<(" rows,  ");
        mascii->GetStream()->operator<<((int)derived().cols());
        mascii->GetStream()->operator<<(" columns:\n");
        for (int i = 0; i < derived().rows(); i++) {
            mascii->indent();
            for (int j = 0; j < derived().cols(); j++) {
                (*mascii->GetStream()) << derived()(i, j);
                mascii->GetStream()->operator<<(", ");
            }
            mascii->GetStream()->operator<<("\n");
        }
    } else {
        size_t m_row = derived().rows();
		size_t m_col = derived().cols();
        marchive << chrono::make_ChNameValue("rows", m_row);
        marchive << chrono::make_ChNameValue("columns", m_col);
         
        // NORMAL array-based serialization:
        size_t tot_elements = derived().rows() *  derived().cols();
		double* foo = 0;
        chrono::ChValueSpecific< double* > specVal(foo, "data", 0);
        marchive.out_array_pre(specVal, tot_elements);
		char idname[21]; // only for xml, xml serialization needs unique element name
        for (size_t i = 0; i < tot_elements; i++) {
			sprintf(idname, "%lu", (unsigned long)i);
            marchive << chrono::CHNVP(derived()((Eigen::Index)i), idname);
            marchive.out_array_between(specVal, tot_elements);
        }
        marchive.out_array_end(specVal, tot_elements);
    }
}

void ArchiveIn(chrono::ChArchiveIn& marchive) {
    // suggested: use versioning
    /*int version =*/ marchive.VersionRead<chrono::ChMatrix_dense_version_tag>(); // btw use the ChMatrixDynamic version tag also for all other templates.
	
    // stream in all member data
    size_t m_row; 
	size_t m_col;
    marchive >> chrono::make_ChNameValue("rows", m_row);
    marchive >> chrono::make_ChNameValue("columns", m_col);
	
    derived().resize(m_row, m_col);

    // custom input of matrix data as array
    size_t tot_elements = derived().rows() * derived().cols();
    marchive.in_array_pre("data", tot_elements);
	char idname[20]; // only for xml, xml serialization needs unique element name
    for (size_t i = 0; i < tot_elements; i++) {
		sprintf(idname, "%lu", (unsigned long)i);
        marchive >> chrono::CHNVP(derived()((Eigen::Index)i), idname);
        marchive.in_array_between("data");
    }
    marchive.in_array_end("data");
}

#endif
