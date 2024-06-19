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

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {
namespace utils {

/// @addtogroup fsi_utils
/// @{

/// Convert a Real3 data structure to a ChVector3d data structure.
CH_FSI_API ChVector3d ToChVector(const Real3& p3);

/// Convert a Real2 data structure to a ChVector3d data structure.
CH_FSI_API ChVector3d ToChVector(const Real2& p2);

/// Convert the first 3 arguments of a Real4 data structure to a ChVector3d data structure.
CH_FSI_API ChVector3d ToChVector(const Real4& p4);

/// Convert a Real4 data structure to a ChQuaternion data structure.
CH_FSI_API ChQuaternion<> ToChQuaternion(const Real4& q4);

// Convert a ChVector2 data structure to a Real2 data structure.
Real2 ToReal2(const ChVector2<>& v2);

/// Convert a ChVector data structure to a Real3 data structure.
CH_FSI_API Real3 ToReal3(const ChVector3<>& v3);

/// Convert a ChVector3d and a scalar to a Real4 data structure.
CH_FSI_API Real4 ToReal4(const ChVector3d& v3, Real m);

/// Convert a ChQuaternion data structure to a Real4 data structure.
CH_FSI_API Real4 ToReal4(const ChQuaternion<>& q4);

/// @} fsi_utils

}  // namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
