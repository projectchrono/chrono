// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#include "chrono_vsg/utils/ChConversionsVSG.h"

namespace vsg {

vec3CH::vec3CH(const chrono::ChVector<>& vec) {
    x = static_cast<float>(vec.x());
    y = static_cast<float>(vec.y());
    z = static_cast<float>(vec.z());
}

dvec3CH::dvec3CH(const chrono::ChVector<>& vec) {
    x = vec.x();
    y = vec.y();
    z = vec.z();
}

dmat4CH::dmat4CH(const chrono::ChFrame<>& frame, const chrono::ChVector<>& scale) {
    const auto& v = frame.GetPos();
    const auto& A = frame.GetA();

    value[0] = {scale.x() * A(0), scale.x() * A(3), scale.x() * A(6), 0};
    value[1] = {scale.y() * A(1), scale.y() * A(4), scale.y() * A(7), 0};
    value[2] = {scale.z() * A(2), scale.z() * A(5), scale.z() * A(8), 0};
    value[3] = {v.x(), v.y(), v.z(), 1};
}

dmat4CH::dmat4CH(const chrono::ChFrame<>& frame, double scale) : dmat4CH(frame, chrono::ChVector<>(scale)) {}

}  // namespace vsg
