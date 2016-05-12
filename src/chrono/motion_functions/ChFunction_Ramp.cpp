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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/motion_functions/ChFunction_Ramp.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
ChClassRegister<ChFunction_Ramp> a_registration_ramp;

ChFunction_Ramp::ChFunction_Ramp(const ChFunction_Ramp& other) {
    y0 = other.y0;
    ang = other.ang;
}

}  // end namespace chrono