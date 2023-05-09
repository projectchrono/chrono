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
// HMMWV simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/powertrain/HMMWV_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// Static variables
const double HMMWV_EngineSimple::m_max_torque = 330;
const double HMMWV_EngineSimple::m_max_power = 110000;
const double HMMWV_EngineSimple::m_max_speed = 10000;

HMMWV_EngineSimple::HMMWV_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
