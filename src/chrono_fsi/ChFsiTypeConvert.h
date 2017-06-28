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

#ifndef CH_FSI_TYPECONVERT_H_
#define CH_FSI_TYPECONVERT_H_

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/custom_math.h"

namespace chrono {
namespace fsi {

/// Class for converting Chrono data structure to/from FSI data structure.
class CH_FSI_API ChFsiTypeConvert {
  public:
    /// Converts a Real3 data structure to a ChVector data structure.
    static ChVector<> Real3ToChVector(Real3 p3);

    /// Converts the first 3 arguments of a Real4 data structure to a ChVector data structure.
    static ChVector<> Real4ToChVector(Real4 p4);

    /// Converts a Real4 data structure to a ChQuaternion data structure.
    static ChQuaternion<> Real4ToChQuaternion(Real4 q4);

    /// Converts a ChVector data structure to a Real3 data structure.
    static Real3 ChVectorToReal3(ChVector<> v3);

    /// Converts a ChVector and a scalar to a Real4 data structure.
    static Real4 ChVectorRToReal4(ChVector<> v3, Real m);

    /// Converts a ChQuaternion data structure to a Real4 data structure.
    static Real4 ChQuaternionToReal4(ChQuaternion<> q4);

  private:
};

}  // end namespace fsi
}  // end namespace chrono
#endif
