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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/fea/ChMatrixCorotation.h"

namespace chrono {
namespace fea {

/// Perform a corotation (warping) of a K matrix by pre-multiplying
/// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
void ChMatrixCorotation::ComputeCK(ChMatrixConstRef K,     // matrix to corotate
                                   const ChMatrix33<>& R,  // 3x3 rotation matrix
                                   const int nblocks,      // number of rotation blocks
                                   ChMatrixRef CK          // result matrix: C*K
) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        double sum;
        for (int colres = 0; colres < (int)K.cols(); ++colres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += R(row, col) * K((3 * iblock) + col, colres);
                CK((3 * iblock) + row, colres) = sum;
            }
    }
}

/// Perform a corotation (warping) of a K matrix by post-multiplying
/// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
void ChMatrixCorotation::ComputeKCt(ChMatrixConstRef K,     // matrix to corotate
                                    const ChMatrix33<>& R,  // 3x3 rotation matrix (will be used transposed)
                                    const int nblocks,      // number of rotation blocks
                                    ChMatrixRef KC          // result matrix: C*K
) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        double sum;
        for (int rowres = 0; rowres < (int)K.rows(); ++rowres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += K(rowres, col + (3 * iblock)) * R(row, col);
                KC(rowres, row + (3 * iblock)) = sum;
            }
    }
}

void ChMatrixCorotation::ComputeCK(ChMatrixConstRef K,                   // matrix to corotate
                                   const std::vector<ChMatrix33<>*>& R,  // 3x3 rotation matrices
                                   const int nblocks,                    // number of rotation blocks
                                   ChMatrixRef CK                        // result matrix: C*K
) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        const ChMatrix33<>* mR = R[iblock];

        double sum;
        for (int colres = 0; colres < (int)K.cols(); ++colres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += (*mR)(row, col) * K((3 * iblock) + col, colres);
                CK((3 * iblock) + row, colres) = sum;
            }
    }
}

void ChMatrixCorotation::ComputeKCt(ChMatrixConstRef K,                   // matrix to corotate
                                    const std::vector<ChMatrix33<>*>& R,  // 3x3 rotation matrices (used transposed)
                                    const int nblocks,                    // number of rotation blocks
                                    ChMatrixRef KC                        // result matrix: C*K
) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        const ChMatrix33<>* mR = R[iblock];

        double sum;
        for (int rowres = 0; rowres < (int)K.rows(); ++rowres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += K(rowres, col + (3 * iblock)) * (*mR)(row, col);
                KC(rowres, row + (3 * iblock)) = sum;
            }
    }
}

}  // namespace fea
}  // namespace chrono
