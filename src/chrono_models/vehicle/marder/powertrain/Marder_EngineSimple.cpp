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
// Marder simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#include "chrono_models/vehicle/marder/powertrain/Marder_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace marder {

// Static variables
const double Marder_EngineSimple::m_max_torque = 2400;
const double Marder_EngineSimple::m_max_power = 530000;
const double Marder_EngineSimple::m_max_speed = 10000;

Marder_EngineSimple::Marder_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // end namespace Marder
}  // end namespace vehicle
}  // end namespace chrono

