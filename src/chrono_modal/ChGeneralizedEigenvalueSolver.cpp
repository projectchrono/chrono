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

#include "chrono_modal/ChGeneralizedEigenvalueSolver.h"


namespace chrono {
Eigen::Map<Eigen::SparseMatrix<double, Eigen::ColMajor, int>> getColMajorSparseMatrix(const ChSparseMatrix& mat) {
    return Eigen::Map<Eigen::SparseMatrix<double, Eigen::ColMajor, int>>(
        const_cast<ChSparseMatrix&>(mat).rows(), const_cast<ChSparseMatrix&>(mat).cols(),
        const_cast<ChSparseMatrix&>(mat).nonZeros(), const_cast<ChSparseMatrix&>(mat).outerIndexPtr(),
        const_cast<ChSparseMatrix&>(mat).innerIndexPtr(), const_cast<ChSparseMatrix&>(mat).valuePtr());
}

void CountNonZerosForEachRow(const ChSparseMatrix& Q, Eigen::VectorXi& nonZerosPerRow, int offset) {
    for (auto row_i = 0; row_i < Q.outerSize(); row_i++) {
        nonZerosPerRow[row_i + offset] +=
            Q.isCompressed() ? Q.outerIndexPtr()[row_i + 1] - Q.outerIndexPtr()[row_i] : Q.innerNonZeroPtr()[row_i];
    }
}

void CountNonZerosForEachRowTransposed(const ChSparseMatrix& Q_transp, Eigen::VectorXi& nonZerosPerRow, int offset) {
    for (auto k = 0; k < Q_transp.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Q_transp, k); it; ++it)
            nonZerosPerRow[it.col() + offset]++;
}

void PlaceMatrix(ChSparseMatrix& HCQ, const ChSparseMatrix& H, int row_start, int col_start, double scale) {
    for (int k = 0; k < H.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(H, k); it; ++it) {
            HCQ.coeffRef(it.row() + row_start, it.col() + col_start) = scale * it.value();
        }
}

}  // namespace chrono