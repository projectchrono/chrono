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

#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

chrono::ChVector<> ChUtilsTypeConvert::Real3ToChVector(Real3 p3) {
    return chrono::ChVector<>(p3.x, p3.y, p3.z);
}

chrono::ChVector<> ChUtilsTypeConvert::Real4ToChVector(Real4 p4) {
    return Real3ToChVector(mR3(p4));
}

chrono::ChQuaternion<> ChUtilsTypeConvert::Real4ToChQuaternion(Real4 q4) {
    return chrono::ChQuaternion<>(q4.x, q4.y, q4.z, q4.w);
}

Real3 ChUtilsTypeConvert::ChVectorToReal3(chrono::ChVector<> v3) {
    return mR3(v3.x(), v3.y(), v3.z());
}

Real4 ChUtilsTypeConvert::ChVectorRToReal4(chrono::ChVector<> v3, Real m) {
    return mR4(v3.x(), v3.y(), v3.z(), m);
}

Real4 ChUtilsTypeConvert::ChQuaternionToReal4(chrono::ChQuaternion<> q4) {
    return mR4(q4.e0(), q4.e1(), q4.e2(), q4.e3());
}

}  // end namespace fsi
}  // end namespace chrono
