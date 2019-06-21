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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle powertrain.
//
// =============================================================================

#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

CH_VEHICLE_API const WheelID FRONT_LEFT(0, LEFT);
CH_VEHICLE_API const WheelID FRONT_RIGHT(0, RIGHT);
CH_VEHICLE_API const WheelID REAR_LEFT(1, LEFT);
CH_VEHICLE_API const WheelID REAR_RIGHT(1, RIGHT);

}  // end namespace vehicle
}  // end namespace chrono
