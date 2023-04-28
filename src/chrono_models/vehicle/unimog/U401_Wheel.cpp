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
// HMMWV wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/unimog/U401_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace unimog {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double U401_Wheel::m_mass = 10.0;
const ChVector<> U401_Wheel::m_inertia(0.3308, 0.5194, 0.3308);

const double U401_Wheel::m_radius = 0.252;
const double U401_Wheel::m_width = 0.063;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
U401_Wheel::U401_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "unimog/U401_Wheel.obj";
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

