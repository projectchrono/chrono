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
// Generic vehicle Magic Formula tire subsystem
//
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/generic/tire/Generic_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace generic {

// Static variables
const double Generic_Pac02Tire::m_mass = 37.6;
const ChVector<> Generic_Pac02Tire::m_inertia(3.84, 6.69, 3.84);
const std::string Generic_Pac02Tire::m_meshFile_left = "generic/tire/generic_tire_coarse.obj";
const std::string Generic_Pac02Tire::m_meshFile_right = "generic/tire/generic_tire_coarse.obj";

// -----------------------------------------------------------------------------

Generic_Pac02Tire::Generic_Pac02Tire(const std::string& name, unsigned int pressure_level) : ChPac02Tire(name) {}

void Generic_Pac02Tire::SetMFParams() {
    std::string tir_file;

    tir_file = "generic/tire/Generic_Pac02Tire.tir";

    SetMFParamsByFile(vehicle::GetDataFile(tir_file));
}

// -----------------------------------------------------------------------------

void Generic_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void Generic_Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace generic
}  // end namespace vehicle
}  // end namespace chrono
