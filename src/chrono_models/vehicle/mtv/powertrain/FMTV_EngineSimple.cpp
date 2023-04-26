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
// FMTV simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#include "chrono_models/vehicle/mtv/powertrain/FMTV_EngineSimple.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables
const double FMTV_EngineSimple::m_max_torque = 1160;
const double FMTV_EngineSimple::m_max_power = 202000;
const double FMTV_EngineSimple::m_max_speed = 10000;

FMTV_EngineSimple::FMTV_EngineSimple(const std::string& name) : ChEngineSimple(name) {}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
