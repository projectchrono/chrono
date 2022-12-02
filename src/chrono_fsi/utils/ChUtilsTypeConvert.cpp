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

#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace utils {

ChVector<> ToChVector(const Real3& p3) {
    return ChVector<>(p3.x, p3.y, p3.z);
}

ChVector<> ToChVector(const Real2& p2) {
    return ChVector<>(p2.x, p2.y, 0.0);
}

ChVector<> ToChVector(const Real4& p4) {
    return ChVector<>(p4.x, p4.y, p4.z);
}

ChQuaternion<> ToChQuaternion(const Real4& q4) {
    return ChQuaternion<>(q4.x, q4.y, q4.z, q4.w);
}

Real3 ToReal3(const ChVector<>& v3) {
    return mR3(v3.x(), v3.y(), v3.z());
}

Real4 ToReal4(const ChVector<>& v3, Real m) {
    return mR4(v3.x(), v3.y(), v3.z(), m);
}

Real4 ToReal4(const ChQuaternion<>& q4) {
    return mR4(q4.e0(), q4.e1(), q4.e2(), q4.e3());
}

}  // namespace utils
}  // end namespace fsi
}  // end namespace chrono
