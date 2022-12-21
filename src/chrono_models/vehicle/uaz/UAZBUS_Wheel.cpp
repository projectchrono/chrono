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
// UAZBUS wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double UAZBUS_Wheel::m_mass = 12.0;
const ChVector<> UAZBUS_Wheel::m_inertia(0.24, 0.42, 0.24);

const double UAZBUS_Wheel::m_radius = 0.2032;
const double UAZBUS_Wheel::m_width = 0.1524;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Wheel::UAZBUS_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "uaz/uaz_rim.obj";
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
