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
// Authors: Radu Serban, Justin Madsen, Rainer Gericke
// =============================================================================
//
// FEDA wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/feda/FEDA_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_Wheel::m_mass = 18.8;
const ChVector<> FEDA_Wheel::m_inertia(0.4634, 0.6243, 0.4634);

const double FEDA_Wheel::m_radius = 0.28575;
const double FEDA_Wheel::m_width = 0.254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Wheel::FEDA_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "feda/meshes/feda_rim.obj";
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
