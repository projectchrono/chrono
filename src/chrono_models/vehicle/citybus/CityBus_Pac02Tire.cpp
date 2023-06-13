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
// CityBus Magic Formula tire subsystem
//
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/citybus/CityBus_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// Static variables
const double CityBus_Pac02Tire::m_mass = 68.6;
const ChVector<> CityBus_Pac02Tire::m_inertia(6.104, 11.144, 6.104);
const std::string CityBus_Pac02Tire::m_meshFile_left = "citybus/CityBusTire.obj";
const std::string CityBus_Pac02Tire::m_meshFile_right = "citybus/CityBusTire.obj";

// -----------------------------------------------------------------------------

CityBus_Pac02Tire::CityBus_Pac02Tire(const std::string& name, unsigned int pressure_level) : ChPac02Tire(name) {}

void CityBus_Pac02Tire::SetMFParams() {
    std::string tir_file;

    tir_file = "citybus/tire/CityBus_Pac02Tire.tir";

    SetMFParamsByFile(vehicle::GetDataFile(tir_file));
}

// -----------------------------------------------------------------------------

void CityBus_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void CityBus_Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
