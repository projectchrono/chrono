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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
Gator_EngineSimple::Gator_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

// Static variables
const double Gator_EngineSimple::m_max_torque = 200;                      // N.m
const double Gator_EngineSimple::m_max_power = 14000;                     // TODO verify (~20bhp)
const double Gator_EngineSimple::m_max_speed = 3500 * (CH_C_2PI / 60.0);  // rad/s

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
