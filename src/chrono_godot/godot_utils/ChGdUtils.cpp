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
// Authors: Asher Elmquist
// =============================================================================
//
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================
#include <core/math/quat.h>
#include <core/math/transform.h>
#include <core/vector.h>
//#include <scene/main/viewport.h>

// ChGodotIncludes
#include "chrono_godot/godot_utils/ChGdUtils.h"

namespace chrono {
namespace gd {

Vector3 GDVector(ChVector<> vec) {
    return Vector3(float(vec.x()), float(vec.y()), float(vec.z()));
}
Quat GDQuat(ChQuaternion<> quat) {
    ChVector<> vec;
    double ang;
    quat.Q_to_AngAxis(ang, vec);
    return Quat(GDVector(vec), ang);
}

Transform GDTransform(ChQuaternion<> q) {
    Transform t;
    ChVector<> vec;
    double ang;
    q.Q_to_AngAxis(ang, vec);
    t.rotate(GDVector(vec), ang);
    return t;
}

}  // namespace gd
}  // namespace chrono
