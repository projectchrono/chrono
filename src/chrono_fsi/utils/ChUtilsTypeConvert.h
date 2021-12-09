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
// Author: Arman Pazouki
// =============================================================================
//
// Utility class for Conversions between Real and ChSystem types such as
// (ChVector, ChQuaternion)
// =============================================================================

#ifndef CH_FSI_UTILS_TYPECONVERT_H
#define CH_FSI_UTILS_TYPECONVERT_H

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_utils
/// @{

/// Class for converting Chrono data structure to/from FSI data structure.
class CH_FSI_API ChUtilsTypeConvert {
  public:
    /// Convert a Real3 data structure to a ChVector data structure.
    static ChVector<> Real3ToChVector(Real3 p3);

    /// Convert a Real2 data structure to a ChVector data structure.
    static ChVector<> Real2ToChVector(Real2 p2);

    /// Convert the first 3 arguments of a Real4 data structure to a ChVector data structure.
    static ChVector<> Real4ToChVector(Real4 p4);

    /// Convert a Real4 data structure to a ChQuaternion data structure.
    static ChQuaternion<> Real4ToChQuaternion(Real4 q4);

    /// Convert a ChVector data structure to a Real3 data structure.
    static Real3 ChVectorToReal3(ChVector<> v3);

    /// Convert a ChVector and a scalar to a Real4 data structure.
    static Real4 ChVectorToReal4(ChVector<> v3, Real m);

    /// Convert a ChQuaternion data structure to a Real4 data structure.
    static Real4 ChQuaternionToReal4(ChQuaternion<> q4);

  private:
};

/// @} fsi_utils

}  // end namespace fsi
}  // end namespace chrono

#endif
