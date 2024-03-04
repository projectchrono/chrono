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

vec2CH::vec2CH(const chrono::ChVector2<>& vec) {
    x = static_cast<float>(vec.x());
    y = static_cast<float>(vec.y());
}

vec3CH::vec3CH(const chrono::ChVector<>& vec) {
    x = static_cast<float>(vec.x());
    y = static_cast<float>(vec.y());
    z = static_cast<float>(vec.z());
}

vec3CH::vec3CH(const chrono::ChColor& col) {
    x = col.R;
    y = col.G;
    z = col.B;
}

dvec3CH::dvec3CH(const chrono::ChVector<>& vec) {
    x = vec.x();
    y = vec.y();
    z = vec.z();
}

vec4CH::vec4CH(const chrono::ChVector<>& vec, double w) {
    x = static_cast<float>(vec.x());
    y = static_cast<float>(vec.y());
    z = static_cast<float>(vec.z());
    w = static_cast<float>(w);
}

vec4CH::vec4CH(const chrono::ChColor& col, float a) {
    x = col.R;
    y = col.G;
    z = col.B;
    w = a;
}

dmat4CH::dmat4CH(const chrono::ChFrame<>& frame, const chrono::ChVector<>& scale) {
    const auto& v = frame.GetPos();
    const auto& A = frame.GetA();

    value[0].set(scale.x() * A(0), scale.x() * A(3), scale.x() * A(6), 0);
    value[1].set(scale.y() * A(1), scale.y() * A(4), scale.y() * A(7), 0);
    value[2].set(scale.z() * A(2), scale.z() * A(5), scale.z() * A(8), 0);
    value[3].set(v.x(), v.y(), v.z(), 1);
}

dmat4CH::dmat4CH(const chrono::ChFrame<>& frame, double scale) : dmat4CH(frame, chrono::ChVector<>(scale)) {}

}  // namespace vsg
