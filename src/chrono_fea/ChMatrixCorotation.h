// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHMATRIXCOROTATION_H
#define CHMATRIXCOROTATION_H

#include "chrono_fea/ChApiFEA.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"

namespace chrono {
namespace fea {

/// Perform a corotation (warping) of a K matrix by pre- or post- multiplying
/// it with a C matrix that has 3x3 rotation matrices R as diagonal blocks,
/// so that C*K  means:
/// <pre>
///            [R      ]  [       ]
///            [   R   ] *[   K   ]
///            [      R]  [       ]
/// </pre>
///
/// This is often used in FEA codes to rotate a K local stiffness matrix
/// and to obtain a global stiffness matrix.
/// This class provides methods to do either C*K or also C*K*C' , without
/// explicitly building C, for improved performance.

template <class Real = double>
class ChMatrixCorotation {
  public:
    /// Perform a corotation (warping) of a K matrix by pre-multiplying
    /// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
    static void ComputeCK(const ChMatrix<Real>& K,    ///< matrix to pre-corotate
                          const ChMatrix33<Real>& R,  ///< 3x3 rotation matrix
                          const int nblocks,          ///< number of rotation blocks
                          ChMatrix<Real>& CK          ///< result matrix: C*K
                          );

    /// Perform a corotation (warping) of a K matrix by post-multiplying
    /// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
    static void ComputeKCt(const ChMatrix<Real>& K,    ///< matrix to post-corotate
                           const ChMatrix33<Real>& R,  ///< 3x3 rotation matrix (will be used transposed)
                           const int nblocks,          ///< number of rotation blocks
                           ChMatrix<Real>& KC          ///< result matrix: C*K
                           );

    /// Perform a corotation (warping) of a K matrix by pre-multiplying
    /// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
    /// (generic version with different rotations)
    static void ComputeCK(const ChMatrix<Real>& K,                  ///< matrix to pre-corotate
                          const std::vector<ChMatrix33<Real>*>& R,  ///< 3x3 rotation matrices
                          const int nblocks,                        ///< number of rotation blocks
                          ChMatrix<Real>& CK                        ///< result matrix: C*K
                          );

    /// Perform a corotation (warping) of a K matrix by post-multiplying
    /// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
    /// (generic version with different rotations)
    static void ComputeKCt(const ChMatrix<Real>& K,                  ///< matrix to post-corotate
                           const std::vector<ChMatrix33<Real>*>& R,  ///< 3x3 rotation matrices (used transposed)
                           const int nblocks,                        ///< number of rotation blocks
                           ChMatrix<Real>& KC                        ///< result matrix: C*K
                           );
};

/// Perform a corotation (warping) of a K matrix by pre-multiplying
/// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
template <class Real>
void ChMatrixCorotation<Real>::ComputeCK(const ChMatrix<Real>& K,    ///< matrix to corotate
                                         const ChMatrix33<Real>& R,  ///< 3x3 rotation matrix
                                         const int nblocks,          ///< number of rotation blocks
                                         ChMatrix<Real>& CK          ///< result matrix: C*K
                                         ) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        Real sum;
        for (int colres = 0; colres < K.GetColumns(); ++colres)
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
template <class Real>
void ChMatrixCorotation<Real>::ComputeKCt(const ChMatrix<Real>& K,    ///< matrix to corotate
                                          const ChMatrix33<Real>& R,  ///< 3x3 rotation matrix (will be used transposed)
                                          const int nblocks,          ///< number of rotation blocks
                                          ChMatrix<Real>& KC          ///< result matrix: C*K
                                          ) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        Real sum;
        for (int rowres = 0; rowres < K.GetRows(); ++rowres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += K(rowres, col + (3 * iblock)) * R(row, col);
                KC(rowres, row + (3 * iblock)) = sum;
            }
    }
}

/// generic version

template <class Real>
void ChMatrixCorotation<Real>::ComputeCK(const ChMatrix<Real>& K,                  ///< matrix to corotate
                                         const std::vector<ChMatrix33<Real>*>& R,  ///< 3x3 rotation matrices
                                         const int nblocks,                        ///< number of rotation blocks
                                         ChMatrix<Real>& CK                        ///< result matrix: C*K
                                         ) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        const ChMatrix33<>* mR = R[iblock];

        Real sum;
        for (int colres = 0; colres < K.GetColumns(); ++colres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += mR->GetElement(row, col) * K((3 * iblock) + col, colres);
                CK((3 * iblock) + row, colres) = sum;
            }
    }
}

/// generic version
template <class Real>
void ChMatrixCorotation<Real>::ComputeKCt(
    const ChMatrix<Real>& K,                  ///< matrix to corotate
    const std::vector<ChMatrix33<Real>*>& R,  ///< 3x3 rotation matrices (used transposed)
    const int nblocks,                        ///< number of rotation blocks
    ChMatrix<Real>& KC                        ///< result matrix: C*K
    ) {
    for (int iblock = 0; iblock < nblocks; iblock++) {
        const ChMatrix33<>* mR = R[iblock];

        Real sum;
        for (int rowres = 0; rowres < K.GetRows(); ++rowres)
            for (int row = 0; row < 3; ++row) {
                sum = 0;
                for (int col = 0; col < 3; ++col)
                    sum += K(rowres, col + (3 * iblock)) * mR->GetElement(row, col);
                KC(rowres, row + (3 * iblock)) = sum;
            }
    }
}

}  // end namespace fea
}  // end namespace chrono

#endif
