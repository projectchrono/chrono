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

#include "chrono_vehicle/ChPowertrain.h"

namespace chrono {
namespace vehicle {

ChPowertrain::ChPowertrain() : m_drive_mode(FORWARD) {
}

}  // end namespace vehicle
}  // end namespace chrono
