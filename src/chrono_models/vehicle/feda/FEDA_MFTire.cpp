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
// FEDA Magic Formula tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/feda/FEDA_MFTire.h"

namespace chrono {
namespace vehicle {
namespace feda {

// Static variables
const double FEDA_MFTire::m_mass = 55.4;
const ChVector<> FEDA_MFTire::m_inertia(6.39, 11.31, 6.39);
const std::string FEDA_MFTire::m_meshFile_left = "feda/meshes/feda_tire_fine.obj";
const std::string FEDA_MFTire::m_meshFile_right = "feda/meshes/feda_tire_fine.obj";

// -----------------------------------------------------------------------------

FEDA_MFTire::FEDA_MFTire(const std::string& name, unsigned int pressure_level) : ChMFTire(name) {}

void FEDA_MFTire::SetMFParams() {
    // Convert the given tire inflation pressure from Pa to PSI
    const double kPa2PSI = 0.145038;
    double pressure_psi = (m_pressure / 1000) * kPa2PSI;

    // Clamp the given tire inflation pressure to one of the available 4 levels
    std::string tir_file;
    if (pressure_psi < 50)
        tir_file = "feda/tires/335_65R22_5_G275MSA_40psi.tir";
    else if (pressure_psi < 65)
        tir_file = "feda/tires/335_65R22_5_G275MSA_60psi.tir";
    else if (pressure_psi < 82.5)
        tir_file = "feda/tires/335_65R22_5_G275MSA_70psi.tir";
    else
        tir_file = "feda/tires/335_65R22_5_G275MSA_95psi.tir";

    SetMFParamsByFile(vehicle::GetDataFile(tir_file));
}

// -----------------------------------------------------------------------------

void FEDA_MFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChMFTire::AddVisualizationAssets(vis);
    }
}

void FEDA_MFTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChMFTire::RemoveVisualizationAssets();
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
