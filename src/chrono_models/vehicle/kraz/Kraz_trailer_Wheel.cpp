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
// Kraz trailer wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_models/vehicle/kraz/Kraz_trailer_Wheel.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {
namespace kraz {

const double Kraz_trailer_Wheel::m_mass = 30.0;
const ChVector<> Kraz_trailer_Wheel::m_inertia(0.6, 0.63, 0.6);

const double Kraz_trailer_Wheel::m_radius = 0.28575;
const double Kraz_trailer_Wheel::m_width = 0.29845;

Kraz_trailer_Wheel::Kraz_trailer_Wheel(const std::string& name) : ChWheel(name) {
    m_vis_mesh_file = "longhaul/meshes/SemiTrailer_rim.obj";
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
