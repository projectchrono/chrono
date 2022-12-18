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
// mrole wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/mrole/mrole_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double mrole_Wheel::m_mass = 115.0;
const ChVector<> mrole_Wheel::m_inertia(0.46, 0.65, 0.46);

const double mrole_Wheel::m_radius = 0.3429;
const double mrole_Wheel::m_width = 0.3408;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_Wheel::mrole_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "mrole/meshes/mrole_rim.obj";
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
