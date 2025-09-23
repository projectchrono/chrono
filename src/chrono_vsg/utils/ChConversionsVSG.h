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

#ifndef CH_CONVERSIONS_VSG_H
#define CH_CONVERSIONS_VSG_H

#include <vsg/maths/mat4.h>
#include <vsg/maths/vec3.h>

#include <vsgImGui/imgui.h>

#include "chrono/assets/ChColor.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vsg/ChApiVSG.h"

namespace vsg {

class CH_VSG_API vec2CH : public vec2 {
  public:
    vec2CH(const chrono::ChVector2d& vec);
};

class CH_VSG_API vec3CH : public vec3 {
  public:
    vec3CH(const chrono::ChVector3d& vec);
    vec3CH(const chrono::ChColor& col);
};

class CH_VSG_API dvec3CH : public dvec3 {
  public:
    dvec3CH(const chrono::ChVector3d& vec);
};

class CH_VSG_API vec4CH : public vec4 {
  public:
    vec4CH(const chrono::ChVector3d& vec, double w);
    vec4CH(const chrono::ChColor& col, float a = 1);
};

class CH_VSG_API dmat4CH : public dmat4 {
  public:
    dmat4CH(const chrono::ChFrame<>& frame, const chrono::ChVector3d& scale);
    dmat4CH(const chrono::ChFrame<>& frame, double scale);
};

}  // namespace vsg

class CH_VSG_API ImVec4CH : public ImVec4 {
  public:
    ImVec4CH(const chrono::ChColor& color, float alpha);
};

inline std::ostream& operator<<(std::ostream& out, const vsg::vec2& v) {
    out << v.x << "  " << v.y;
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const vsg::vec3& v) {
    out << v.x << "  " << v.y << " " << v.z;
    return out;
}

#endif
