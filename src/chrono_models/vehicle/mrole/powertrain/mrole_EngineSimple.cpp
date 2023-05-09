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
// Authors: Radu Serban
// =============================================================================
//
// mrole simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#include "chrono_models/vehicle/mrole/powertrain/mrole_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// Static variables
const double mrole_EngineSimple::m_max_torque = 2400;
const double mrole_EngineSimple::m_max_power = 530000;
const double mrole_EngineSimple::m_max_speed = 10000;

mrole_EngineSimple::mrole_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // end namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
