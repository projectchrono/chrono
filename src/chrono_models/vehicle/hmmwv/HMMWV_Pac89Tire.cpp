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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// HMMWV PAC89 tire subsystem
//
// Coefficents were pulled from the Adams/Tire help - Adams 2017.1.
// https://simcompanion.mscsoftware.com/infocenter/index?page=content&id=DOC11293&cat=2017.1_ADAMS_DOCS&actp=LIST
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_Pac89Tire::m_normalDamping = 350;

const double HMMWV_Pac89Tire::m_mass = 37.6;
const ChVector<> HMMWV_Pac89Tire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_Pac89Tire::m_meshName = "hmmwv_tire_POV_geom";
const std::string HMMWV_Pac89Tire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Pac89Tire::HMMWV_Pac89Tire(const std::string& name) : ChPac89Tire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Pac89Tire::SetPac89Params() {
    m_unloaded_radius = 0.464;
    m_width = 0.318;

    m_rolling_resistance = 0.015;
    m_lateral_stiffness = 261065.0;
    m_measured_side = LEFT;

    // Parameter Set Converted from TMeasy, Truck characteristic
    // Influence of camber is ignored

    m_PacCoeff.A0 = 1.49975356208205;
    m_PacCoeff.A1 = -4.84987524731462;
    m_PacCoeff.A2 = 812.449795340733;
    m_PacCoeff.A3 = 2613.92367840654;
    m_PacCoeff.A4 = 48.857910109076;
    m_PacCoeff.A5 = 0.0;
    m_PacCoeff.A6 = -0.00879541881020228;
    m_PacCoeff.A7 = 0.376999015041155;
    m_PacCoeff.A8 = 0.0;
    m_PacCoeff.A9 = 0.0;
    m_PacCoeff.A10 = 0.0;
    m_PacCoeff.A11 = 0.0;
    m_PacCoeff.A12 = 0.0;
    m_PacCoeff.A13 = 0.0;

    m_PacCoeff.B0 = 1.50018802672136;
    m_PacCoeff.B1 = -15.7761466722458;
    m_PacCoeff.B2 = 1022.11238546683;
    m_PacCoeff.B3 = -2.55317715303733;
    m_PacCoeff.B4 = 208.777316195246;
    m_PacCoeff.B5 = 0.0073134908964823;
    m_PacCoeff.B6 = -0.00376410345674027;
    m_PacCoeff.B7 = 0.156330736057758;
    m_PacCoeff.B8 = -1.15310023217878;
    m_PacCoeff.B9 = 0.0;
    m_PacCoeff.B10 = 0.0;

    m_PacCoeff.C0 = 2.34;
    m_PacCoeff.C1 = 0.990427;
    m_PacCoeff.C2 = 2.96848;
    m_PacCoeff.C3 = -0.277098;
    m_PacCoeff.C4 = -0.944859;
    m_PacCoeff.C5 = 0.0;
    m_PacCoeff.C6 = 0.0027699;
    m_PacCoeff.C7 = -0.0001151;
    m_PacCoeff.C8 = 0.10;
    m_PacCoeff.C9 = -1.3329;
    m_PacCoeff.C10 = 0.0;
    m_PacCoeff.C11 = 0.0;
    m_PacCoeff.C12 = 0.0;
    m_PacCoeff.C13 = 0.0;
    m_PacCoeff.C14 = 0.0;
    m_PacCoeff.C15 = 0.0;
    m_PacCoeff.C16 = 0.0;
    m_PacCoeff.C17 = 0.0;
}

double HMMWV_Pac89Tire::GetNormalStiffnessForce(double depth) const {
    // corresponding depths = 0 : 0.01 : 0.03
    // double normalforcetabel[11] = {0.0, 2300.0, 5000.0, 8100.0};
    // modified for tire format "37x12.5x16.5 50 psi"
    // average vertical stiffness = 327000 N/m
    double normalforcetabel[11] = {0.0, 2877.0, 6254.0, 10132.0};

    depth = depth * (depth > 0);  // Ensure that depth is positive;

    double position = (3. * (depth / 0.03));

    // Linear extrapolation if the depth is at or greater than the maximum depth in the table (.030)
    if (position >= 3) {
        return (10132.0 + (depth - 0.03) * 10132.0 / .03);
    }
    // Linearly interpolate between the table entries
    else {
        double scale = std::ceil(position) - position;
        return (normalforcetabel[int(std::floor(position))] * (1 - scale) +
                normalforcetabel[int(std::floor(position) + 1)] * scale);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Pac89Tire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChPac89Tire::AddVisualizationAssets(vis);
    }
}

void HMMWV_Pac89Tire::RemoveVisualizationAssets() {
    ChPac89Tire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
