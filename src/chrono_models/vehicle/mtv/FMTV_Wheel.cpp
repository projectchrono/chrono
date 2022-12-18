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
// FMTV wheel subsystem
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/mtv/FMTV_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FMTV_Wheel::m_mass = 30.0;
const ChVector<> FMTV_Wheel::m_inertia(0.6, 0.63, 0.6);

const double FMTV_Wheel::m_radius = 0.254;
const double FMTV_Wheel::m_width = 0.254;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FMTV_Wheel::FMTV_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "mtv/meshes/MTV_rim.obj";
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
