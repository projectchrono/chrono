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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MAN Kat 1 wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_Wheel::m_mass = 30.0;
const ChVector<> MAN_5t_Wheel::m_inertia(0.6, 0.63, 0.6);

const double MAN_5t_Wheel::m_radius = 0.254;
const double MAN_5t_Wheel::m_width = 0.254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_Wheel::MAN_5t_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "MAN_Kat1/meshes/MAN_rim.obj";
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
