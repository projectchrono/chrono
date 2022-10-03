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
// Utility class for Conversions between Real and ChVector & ChQuaternion.
// =============================================================================

#ifndef CH_FSI_UTILS_TYPECONVERT_H
#define CH_FSI_UTILS_TYPECONVERT_H

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {
namespace utils {

/// @addtogroup fsi_utils
/// @{

/// Convert a Real3 data structure to a ChVector data structure.
CH_FSI_API ChVector<> ToChVector(const Real3& p3);

/// Convert a Real2 data structure to a ChVector data structure.
CH_FSI_API ChVector<> ToChVector(const Real2& p2);

/// Convert the first 3 arguments of a Real4 data structure to a ChVector data structure.
CH_FSI_API ChVector<> ToChVector(const Real4& p4);

/// Convert a Real4 data structure to a ChQuaternion data structure.
CH_FSI_API ChQuaternion<> ToChQuaternion(const Real4& q4);

/// Convert a ChVector data structure to a Real3 data structure.
CH_FSI_API Real3 ToReal3(const ChVector<>& v3);

/// Convert a ChVector and a scalar to a Real4 data structure.
CH_FSI_API Real4 ToReal4(const ChVector<>& v3, Real m);

/// Convert a ChQuaternion data structure to a Real4 data structure.
CH_FSI_API Real4 ToReal4(const ChQuaternion<>& q4);

/// @} fsi_utils

}  // namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
