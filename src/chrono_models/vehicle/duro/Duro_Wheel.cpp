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
// Authors: Rainer Gericke
// =============================================================================
//
// Bucher Duro 4x4 wheel subsystem 9.00x20
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/duro/Duro_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Duro_Wheel::m_mass = 25.0;
const ChVector<> Duro_Wheel::m_inertia(0.94, 1.59, 0.94);

const double Duro_Wheel::m_radius = 0.254;
const double Duro_Wheel::m_width = 0.2286;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Duro_Wheel::Duro_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "duro/Duro_Wheel.obj";
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
