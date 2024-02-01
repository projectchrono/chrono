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

#include "chrono_models/vehicle/generic/powertrain/Generic_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double Generic_EngineSimple::m_max_torque = 330;
const double Generic_EngineSimple::m_max_power = 110000;
const double Generic_EngineSimple::m_max_speed = 10000;

Generic_EngineSimple::Generic_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
