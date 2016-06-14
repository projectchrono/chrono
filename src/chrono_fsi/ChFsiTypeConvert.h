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

class CH_FSI_API ChFsiTypeConvert {
public:
  static ChVector<> Real3ToChVector(Real3 p3);
  static ChVector<> Real4ToChVector(Real4 p4);
  static ChQuaternion<> Real4ToChQuaternion(Real4 q4);
  static Real3 ChVectorToReal3(ChVector<> v3);
  static Real4 ChVectorRToReal4(ChVector<> v3, Real m);
  static Real4 ChQuaternionToReal4(ChQuaternion<> q4);

private:
};

// namespace utils {

// ChVector<> ConvertRealToChVector(Real3 p3);
// ChVector<> ConvertRealToChVector(Real4 p4);
// ChQuaternion<> ConvertToChQuaternion(Real4 q4);
// Real3 ConvertChVectorToR3(ChVector<> v3);
// Real4 ConvertChVectorToR4(ChVector<> v3, Real m);
// Real4 ConvertChQuaternionToR4(ChQuaternion<> q4);

} // end namespace fsi
} // end namespace chrono
#endif
