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

#include "chrono/core/ChCoordsys.h"

namespace chrono {

const ChCoordsys<double> CSYSNULL(ChVector<>(0, 0, 0), ChQuaternion<>(0, 0, 0, 0));
const ChCoordsys<double> CSYSNORM(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

Coordsys Force2Dcsys(const Coordsys& cs) {
    Coordsys res;
    res = cs;
    res.pos.z() = 0;
    res.rot.e1() = 0;
    res.rot.e2() = 0;
    return (res);
}

}  // end namespace chrono
