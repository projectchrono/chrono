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
// Author: Radu Serban
// =============================================================================
// This file contains the definition of SPH and BCE marker types.
// =============================================================================

#ifndef CH_MARKER_TYPE_CUH
#define CH_MARKER_TYPE_CUH

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/ChFsiDataTypesSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Marker (SPH and BCE) groups.
enum class MarkerGroup { FLUID, SOLID, BOUNDARY, NON_FLUID, NON_SOLID, NON_BOUNDARY, ALL };

__host__ __device__ inline bool IsInMarkerGroup(MarkerGroup group, Real code) {
    switch (group) {
        case MarkerGroup::ALL:
            return true;
        case MarkerGroup::FLUID:
            return code < -0.5;
        case MarkerGroup::SOLID:
            return code > 0.5;
        case MarkerGroup::BOUNDARY:
            return code < 0.5 && code > -0.5;
        case MarkerGroup::NON_FLUID:
            return code > -0.5;
        case MarkerGroup::NON_SOLID:
            return code < 0.5;
        case MarkerGroup::NON_BOUNDARY:
            return code > 0.5 || code < -0.5;
    }
    return false;
}

/// Marker (SPH and BCE) type.
enum class MarkerType { SPH_PARTICLE, SPH_HELPER, SPH_GHOST, BCE_WALL, BCE_RIGID, BCE_FLEX1D, BCE_FLEX2D };

__host__ __device__ inline MarkerType GetMarkerType(Real code) {
    if (code > 2.5)                       //
        return MarkerType::BCE_FLEX2D;    // 3    |       |
    else if (code > 1.5)                  //      |       |
        return MarkerType::BCE_FLEX1D;    // 2    | solid |
    else if (code > 0.5)                  //      |       | BCE
        return MarkerType::BCE_RIGID;     // 1    |       |
    else if (code > -0.5)                 // -----+       |
        return MarkerType::BCE_WALL;      // 0    |       |
    else if (code > -1.5)                 // -----+-------+
        return MarkerType::SPH_PARTICLE;  // -1   |       |
    else if (code > -2.5)                 //      |       |
        return MarkerType::SPH_GHOST;     // -2   |       | Fluid
    else                                  //      |       |
        return MarkerType::SPH_HELPER;    // -3   |       |
}

__host__ __device__ inline Real GetMarkerCode(MarkerType type) {
    switch (type) {
        case MarkerType::SPH_HELPER:
            return -3;
        case MarkerType::SPH_GHOST:
            return -2;
        case MarkerType::SPH_PARTICLE:
            return -1;
        case MarkerType::BCE_WALL:
            return 0;
        case MarkerType::BCE_RIGID:
            return 1;
        case MarkerType::BCE_FLEX1D:
            return 2;
        case MarkerType::BCE_FLEX2D:
            return 3;
    }

    return -999;
}

__host__ __device__ inline bool IsSphParticle(Real code) {
    return code < -0.5 && code > -1.5;
}

__host__ __device__ inline bool IsFluidParticle(Real code) {
    return code < -0.5;
}

__host__ __device__ inline bool IsBceMarker(Real code) {
    return code > -0.5;
}

__host__ __device__ inline bool IsBceWallMarker(Real code) {
    return code > -0.5 && code < 0.5;
}

__host__ __device__ inline bool IsBceSolidMarker(Real code) {
    return code > 0.5;
}

/// @} fsisph_physics

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
