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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// CityBus wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/citybus/CityBus_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_Wheel::m_mass = 30.0;
const ChVector<> CityBus_Wheel::m_inertia(0.6, 0.63, 0.6);

const double CityBus_Wheel::m_radius = 0.33365;
const double CityBus_Wheel::m_width = 0.205;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_Wheel::CityBus_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "citybus/CityBusRim.obj";
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
