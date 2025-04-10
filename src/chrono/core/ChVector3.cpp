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
// Authors: Radu Serban
// =============================================================================

#include "chrono/core/ChVector3.h"

namespace chrono {

const ChVector3d VNULL(0., 0., 0.);
const ChVector3d VECT_X(1., 0., 0.);
const ChVector3d VECT_Y(0., 1., 0.);
const ChVector3d VECT_Z(0., 0., 1.);

ChApi ChVector3d operator*(double s, const ChVector3i& V) {
    return ChVector3d(V.x() * s, V.y() * s, V.z() * s);
}

}  // end namespace chrono
