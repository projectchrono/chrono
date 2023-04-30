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
// MAN_5t simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#include "chrono_models/vehicle/man/powertrain/MAN_5t_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace man {

// Static variables
const double MAN_5t_EngineSimple::m_max_torque = 890;
const double MAN_5t_EngineSimple::m_max_power = 188000;
const double MAN_5t_EngineSimple::m_max_speed = 10000;

MAN_5t_EngineSimple::MAN_5t_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
