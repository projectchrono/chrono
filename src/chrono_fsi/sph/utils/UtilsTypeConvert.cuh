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
// Author: Arman Pazouki, Radu Serban
// =============================================================================
//
// Utility class for conversions between Real and ChVector3 & ChQuaternion.
// =============================================================================

#ifndef CH_FSI_UTILS_TYPECONVERT_H
#define CH_FSI_UTILS_TYPECONVERT_H

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChVector2.h"
#include "chrono/geometry/ChGeometry.h"

#include "chrono_fsi/sph/utils/UtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_utils
/// @{

// --- FSI -> Chrono -----------------------------------------------------------

/// Convert a Real3 data structure to a ChVector3d.
inline ChVector3d ToChVector(const Real3& p3) {
    return ChVector3d(p3.x, p3.y, p3.z);
}

/// Convert a Real2 data structure to a ChVector3d.
inline ChVector3d ToChVector(const Real2& p2) {
    return ChVector3d(p2.x, p2.y, 0.0);
}

/// Convert the first 3 arguments of a Real4 data structure to a ChVector3d.
inline ChVector3d ToChVector(const Real4& p4) {
    return ChVector3d(p4.x, p4.y, p4.z);
}

/// Convert an int3 data structure to a ChVector3i.
inline ChVector3i ToChVector(const int3& p3) {
    return ChVector3i(p3.x, p3.y, p3.z);
}

/// Convert a Real4 data structure to a ChQuaterniond.
inline ChQuaterniond ToChQuaternion(const Real4& q4) {
    return ChQuaterniond(q4.x, q4.y, q4.z, q4.w);
}

/// Convert a RealAABB to a ChAABB.
inline ChAABB ToChAABB(const RealAABB& aabb) {
    return ChAABB(ToChVector(aabb.min), ToChVector(aabb.max));
}

/// Convert an IntAABB to a ChIntAABB.
inline ChIntAABB ToChIntAABB(const IntAABB& aabb) {
    return ChIntAABB(ToChVector(aabb.min), ToChVector(aabb.max));
}

// --- Chrono -> FSI -----------------------------------------------------------

/// Convert a ChVector2d to a Real2 data structure.
inline Real2 ToReal2(const ChVector2d& v2) {
    return mR2(v2.x(), v2.y());
}

/// Convert a ChVector3d to a Real3 data structure.
inline Real3 ToReal3(const ChVector3d& v3) {
    return mR3(v3.x(), v3.y(), v3.z());
}

/// Convert a ChVector3d and a scalar to a Real4 data structure.
inline Real4 ToReal4(const ChVector3d& v3, Real m) {
    return mR4(v3.x(), v3.y(), v3.z(), m);
}

/// Convert a ChQuaterniond to a Real4 data structure.
inline Real4 ToReal4(const ChQuaterniond& q4) {
    return mR4(q4.e0(), q4.e1(), q4.e2(), q4.e3());
}

/// Convert a ChVector3i to an int3 data structure.
inline int3 ToInt3(const ChVector3i& v3) {
    return mI3(v3.x(), v3.y(), v3.z());
}

/// Convert a ChAABB to a RealAABB.
inline RealAABB ToRealAABB(const ChAABB& aabb) {
    return RealAABB(ToReal3(aabb.min), ToReal3(aabb.max));
}

/// Convert a ChIntAABB to an IntAABB.
inline IntAABB ToIntAABB(const ChIntAABB& aabb) {
    return IntAABB(ToInt3(aabb.min), ToInt3(aabb.max));
}

/// @} fsisph_utils

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
