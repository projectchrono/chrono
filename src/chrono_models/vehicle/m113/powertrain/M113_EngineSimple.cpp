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
// M113 simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#include "chrono_models/vehicle/m113/powertrain/M113_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// Static variables
const double M113_EngineSimple::m_max_torque = 445;
const double M113_EngineSimple::m_max_power = 156000;
const double M113_EngineSimple::m_max_speed = 10000;

M113_EngineSimple::M113_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // end namespace M113
}  // end namespace vehicle
}  // end namespace chrono

