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

#ifndef CHMATRIXCOROTATION_H
#define CHMATRIXCOROTATION_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix33.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_math
/// @{

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
class ChApi ChMatrixCorotation {
  public:
    /// Perform a corotation (warping) of a K matrix by pre-multiplying
    /// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
    static void ComputeCK(ChMatrixConstRef K,     ///< matrix to pre-corotate
                          const ChMatrix33<>& R,  ///< 3x3 rotation matrix
                          const int nblocks,      ///< number of rotation blocks
                          ChMatrixRef CK          ///< result matrix: C*K
    );

    /// Perform a corotation (warping) of a K matrix by post-multiplying
    /// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
    static void ComputeKCt(ChMatrixConstRef K,     ///< matrix to post-corotate
                           const ChMatrix33<>& R,  ///< 3x3 rotation matrix (will be used transposed)
                           const int nblocks,      ///< number of rotation blocks
                           ChMatrixRef KC          ///< result matrix: C*K
    );

    /// Perform a corotation (warping) of a K matrix by pre-multiplying
    /// it with a C matrix; C has 3x3 rotation matrices R as diagonal blocks
    /// (generic version with different rotations)
    static void ComputeCK(ChMatrixConstRef K,                   ///< matrix to pre-corotate
                          const std::vector<ChMatrix33<>*>& R,  ///< 3x3 rotation matrices
                          const int nblocks,                    ///< number of rotation blocks
                          ChMatrixRef CK                        ///< result matrix: C*K
    );

    /// Perform a corotation (warping) of a K matrix by post-multiplying
    /// it with a transposed C matrix; C has 3x3 rotation matrices R as diagonal blocks
    /// (generic version with different rotations)
    static void ComputeKCt(ChMatrixConstRef K,                   ///< matrix to post-corotate
                           const std::vector<ChMatrix33<>*>& R,  ///< 3x3 rotation matrices (used transposed)
                           const int nblocks,                    ///< number of rotation blocks
                           ChMatrixRef KC                        ///< result matrix: C*K
    );
};

/// @} fea_math

}  // end namespace fea
}  // end namespace chrono

#endif
