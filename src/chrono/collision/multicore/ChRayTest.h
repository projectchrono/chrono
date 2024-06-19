// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Dispatcher for the ray intersection test
//
// =============================================================================

#pragma once

#include "chrono/collision/multicore/ChCollisionData.h"
#include "chrono/collision/multicore/ChConvexShape.h"

namespace chrono {

/// @addtogroup collision_mc
/// @{

/// Class for performing ray intersection tests.
class ChApi ChRayTest {
  public:
    /// Information on ray intersection test result.
    struct RayHitInfo {
        int shapeID;   ///< identifier of closest hit shape
        real3 point;   ///< hit point in absolute frame
        real3 normal;  ///< normal to shape at hit point
        real t;        ///< ray parameter at hit point (in [0,1])
        real dist;     ///< distance to hit point from ray origin
    };

    ChRayTest(std::shared_ptr<ChCollisionData> data);

    /// Check for intersection of the given ray with all collision shapes in the system.
    /// Uses a variant of the 3D Digital Differential Analyser (Akira Fujimoto, "ARTS: Accelerated Ray Tracing Systems",
    /// 1986) to efficiently traverse the broadphase grid and analytical shape-ray intersection tests.
    bool Check(const real3& start,  ///< ray start point
               const real3& end,    ///< ray end point
               RayHitInfo& info     ///< [output] test result info
    );

    /// Return the number of bins visited by the DDA algorithm during the last ray test.
    uint GetNumBinTests() const { return num_bin_tests; }

    /// Return the number of ray-shape checks required by the last ray test.
    uint GetNumShapeTests() const { return num_shape_tests; }

  private:
    /// Dispatcher for analytic functions for ray intersection with primitive shapes.
    bool CheckShape(const ConvexBase& shape,  ///< candidate shape
                    const real3& start,       ///< ray start point
                    const real3& end,         ///< ray end point
                    real3& normal,            ///< [output] normal to shape at intersectin point
                    real& mindist2            ///< [output] smallest squared distance to ray origin
    );

    std::shared_ptr<ChCollisionData> cd_data;  ///< shared collision detection data
    uint num_bin_tests;                        ///< number of bins visited during last ray test
    uint num_shape_tests;                      ///< number of shape checked during last ray test
};

/// @} collision_mc

}  // end namespace chrono
