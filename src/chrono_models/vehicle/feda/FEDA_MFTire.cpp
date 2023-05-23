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
    : ChMFTire(name), m_use_vert_map(false), m_tire_inflation_pressure_level(pressure_level) {}

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

    // setting vertical table
    m_use_vert_map = true;
    m_vert_map.AddPoint(0, 0);
    m_vert_map.AddPoint(0.005, 1505.41);
    m_vert_map.AddPoint(0.01, 3154);
    m_vert_map.AddPoint(0.015, 4931.89);
    m_vert_map.AddPoint(0.02, 6825.2);
    m_vert_map.AddPoint(0.025, 8820.06);
    m_vert_map.AddPoint(0.03, 10902.6);
    m_vert_map.AddPoint(0.035, 13058.9);
    m_vert_map.AddPoint(0.04, 15275.2);
    m_vert_map.AddPoint(0.045, 17537.5);
    m_vert_map.AddPoint(0.05, 19832);
    m_vert_map.AddPoint(0.055, 22144.8);
    m_vert_map.AddPoint(0.06, 24462);
    m_vert_map.AddPoint(0.065, 26769.8);
    m_vert_map.AddPoint(0.07, 29054.2);
    m_vert_map.AddPoint(0.075, 31301.4);
    m_vert_map.AddPoint(0.08, 33497.6);
    m_vert_map.AddPoint(0.085, 35628.8);
    m_vert_map.AddPoint(0.09, 37681.2);
    m_vert_map.AddPoint(0.095, 39640.9);
    m_vert_map.AddPoint(0.1, 41494);

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

void FEDA_MFTire::SetParametersLevel2() {
    // begin of variables set up
    std::string dataFile("feda/tires/335_65R22_5_G275MSA_60psi.tir");
    SetMFParamsByFile(dataFile);
    
    // setting vertical table
    m_use_vert_map = true;
    
    m_vert_map.AddPoint(0, 0);
    m_vert_map.AddPoint(0.005, 2004.06);
    m_vert_map.AddPoint(0.01, 4242.26);
    m_vert_map.AddPoint(0.015, 6688.46);
    m_vert_map.AddPoint(0.02, 9316.51);
    m_vert_map.AddPoint(0.025, 12100.2);
    m_vert_map.AddPoint(0.03, 15013.5);
    m_vert_map.AddPoint(0.035, 18030.2);
    m_vert_map.AddPoint(0.04, 21124.2);
    m_vert_map.AddPoint(0.045, 24269.2);
    m_vert_map.AddPoint(0.05, 27439.2);
    m_vert_map.AddPoint(0.055, 30607.9);
    m_vert_map.AddPoint(0.06, 33749.4);
    m_vert_map.AddPoint(0.065, 36837.3);
    m_vert_map.AddPoint(0.07, 39845.5);
    m_vert_map.AddPoint(0.075, 42748);
    m_vert_map.AddPoint(0.08, 45518.5);
    m_vert_map.AddPoint(0.085, 48130.9);
    m_vert_map.AddPoint(0.09, 50559.1);
    m_vert_map.AddPoint(0.095, 52776.8);
    m_vert_map.AddPoint(0.1, 54758);

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

void FEDA_MFTire::SetParametersLevel3() {
    // begin of variables set up

    m_use_vert_map = false;

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

void FEDA_MFTire::SetParametersLevel4() {
    // begin of variables set up

    m_use_vert_map = false;

    // setting bottoming table
    m_use_bott_map = true;
    m_bott_map.AddPoint(0, 0);
    m_bott_map.AddPoint(0.10546, 0);
    m_bott_map.AddPoint(0.30546, 563080);
}

double FEDA_MFTire::GetNormalStiffnessForce(double depth) const {
    if (m_use_vert_map)
        if (m_use_bott_map) {
            return m_vert_map.Get_y(depth) + m_bott_map.Get_y(depth);
        } else {
            return m_vert_map.Get_y(depth);
        }
    else if (m_use_bott_map) {
        return depth * m_par.VERTICAL_STIFFNESS + m_bott_map.Get_y(depth);
    } else {
        return depth * m_par.VERTICAL_STIFFNESS;
    }
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

