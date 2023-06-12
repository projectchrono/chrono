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
// Sedan Magic Formula tire subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/sedan/Sedan_Pac02Tire.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// Static variables
const double Sedan_Pac02Tire::m_mass = 11.5;
const ChVector<> Sedan_Pac02Tire::m_inertia(0.156, 0.679, 0.156);
const std::string Sedan_Pac02Tire::m_meshFile_left = "sedan/sedan_tire.obj";
const std::string Sedan_Pac02Tire::m_meshFile_right = "sedan/sedan_tire.obj";

// -----------------------------------------------------------------------------

Sedan_Pac02Tire::Sedan_Pac02Tire(const std::string& name, unsigned int pressure_level) : ChPac02Tire(name) {}

void Sedan_Pac02Tire::SetMFParams() {
    std::string tir_file;

    tir_file = "sedan/tire/Sedan_Pac02Tire.tir";

    SetMFParamsByFile(vehicle::GetDataFile(tir_file));
}

// -----------------------------------------------------------------------------

void Sedan_Pac02Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPac02Tire::AddVisualizationAssets(vis);
    }
}

void Sedan_Pac02Tire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPac02Tire::RemoveVisualizationAssets();
}

}  // namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

