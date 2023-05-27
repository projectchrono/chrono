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
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// FEDA PAC02 tire subsystem
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

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_MFTire::m_mass = 55.4;
const ChVector<> FEDA_MFTire::m_inertia(6.39, 11.31, 6.39);

const std::string FEDA_MFTire::m_meshFile_left = "feda/meshes/feda_tire_fine.obj";
const std::string FEDA_MFTire::m_meshFile_right = "feda/meshes/feda_tire_fine.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_MFTire::FEDA_MFTire(const std::string& name, unsigned int pressure_level)
    : ChMFTire(name), m_tire_inflation_pressure_level(pressure_level) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void FEDA_MFTire::SetMFParams() {
    switch (m_tire_inflation_pressure_level) {
        case 1:
            GetLog() << "Tire Inflation Pressure set to 40 psi\n";
            SetParametersLevel1();
            break;
        default:
        case 2:
            GetLog() << "Tire Inflation Pressure set to 60 psi\n";
            SetParametersLevel2();
            break;
        case 3:
            GetLog() << "Tire Inflation Pressure set to 70 psi\n";
            SetParametersLevel3();
            break;
        case 4:
            GetLog() << "Tire Inflation Pressure set to 95 psi\n";
            SetParametersLevel4();
            break;
    }
}

void FEDA_MFTire::SetParametersLevel1() {
    std::string dataFile("feda/tires/335_65R22_5_G275MSA_40psi.tir");
    SetMFParamsByFile(dataFile);
}

void FEDA_MFTire::SetParametersLevel2() {
    // begin of variables set up
    std::string dataFile("feda/tires/335_65R22_5_G275MSA_60psi.tir");
    SetMFParamsByFile(dataFile);
}

void FEDA_MFTire::SetParametersLevel3() {
    // begin of variables set up
    std::string dataFile("feda/tires/335_65R22_5_G275MSA_70psi.tir");
    SetMFParamsByFile(dataFile);
}

void FEDA_MFTire::SetParametersLevel4() {
    // begin of variables set up
    std::string dataFile("feda/tires/335_65R22_5_G275MSA_95psi.tir");
    SetMFParamsByFile(dataFile);
}

// -----------------------------------------------------------------------------
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

