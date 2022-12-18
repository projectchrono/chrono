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
// Authors: Radu Serban, Justin Madsen, Asher Elmquist
// =============================================================================
//
// Sedan wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/sedan/Sedan_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Sedan_Wheel::m_mass = 13.2;
const ChVector<> Sedan_Wheel::m_inertia(0.24, 0.42, 0.24);

const double Sedan_Wheel::m_radius = 0.2286;
const double Sedan_Wheel::m_width = 0.205;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_Wheel::Sedan_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "sedan/sedan_rim.obj";
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
