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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// BMW E90 wheel subsystem front
// Data from https://www.rsu.de (Tire Dealer)
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_models/vehicle/bmw/BMW_E90_FrontWheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double BMW_E90_FrontWheel::m_mass = 18.0;
const ChVector3d BMW_E90_FrontWheel::m_inertia(0.767875, 1.383875, 0.767875);

const double BMW_E90_FrontWheel::m_radius = 0.2286;
const double BMW_E90_FrontWheel::m_width = 0.2032;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_FrontWheel::BMW_E90_FrontWheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "bmw/wheel/bmw_e90_front_Wheel.obj";
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
