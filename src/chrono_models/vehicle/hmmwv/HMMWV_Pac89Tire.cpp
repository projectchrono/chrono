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

const double HMMWV_Pac89Tire::m_normalDamping = 250;

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

    m_unloaded_radius = 326.0/1000;
    m_width = 245.0/1000;
    m_rolling_resistance = 0.0;
    m_lateral_stiffness = 190*1000.; // N/mm -> N/m
    m_measured_side = LEFT;

    m_PacCoeff.A0 = 1.650;
    m_PacCoeff.A1 = -34.0;
    m_PacCoeff.A2 = 1250.0;
    m_PacCoeff.A3 = 3036.0;
    m_PacCoeff.A4 = 12.80;
    m_PacCoeff.A5 = 0.00501;
    m_PacCoeff.A6 = -0.02103;
    m_PacCoeff.A7 = 0.77394;
    m_PacCoeff.A8 = 0.0022890;
    m_PacCoeff.A9 = 0.013442;
    m_PacCoeff.A10 = 0.003709;
    m_PacCoeff.A11 = 19.1656;
    m_PacCoeff.A12 = 1.21356;
    m_PacCoeff.A13 = 6.26206;

    m_PacCoeff.B0 = 1.67272;
    m_PacCoeff.B1 = -9.46;
    m_PacCoeff.B2 = 1490.0;
    m_PacCoeff.B3 = 30.0;
    m_PacCoeff.B4 = 176.0;
    m_PacCoeff.B5 = 0.08860;
    m_PacCoeff.B6 = 0.00402;
    m_PacCoeff.B7 = -0.06150;
    m_PacCoeff.B8 = 0.20;
    m_PacCoeff.B9 = 0.02990;
    m_PacCoeff.B10 = -0.176;

    m_PacCoeff.C0 = 2.34;
    m_PacCoeff.C1 = 1.4950;
    m_PacCoeff.C2 = 6.416654;
    m_PacCoeff.C3 = -3.57403;
    m_PacCoeff.C4 = -0.087737;
    m_PacCoeff.C5 = 0.098410;
    m_PacCoeff.C6 = 0.0027699;
    m_PacCoeff.C7 = -0.0001151;
    m_PacCoeff.C8 = 0.10;
    m_PacCoeff.C9 = -1.3329;
    m_PacCoeff.C10 = 0.025501;
    m_PacCoeff.C11 = -0.02357;
    m_PacCoeff.C12 = 0.03027;
    m_PacCoeff.C13 = -0.0647;
    m_PacCoeff.C14 = 0.0211329;
    m_PacCoeff.C15 = 0.89469;
    m_PacCoeff.C16 = -0.099443;
    m_PacCoeff.C17 = -3.336941;

}

double HMMWV_Pac89Tire::GetNormalStiffnessForce(double depth) const {
    // corresponding depths = 0 : 0.01 : 0.03
    double normalforcetabel[11] = {0.0,
                                   2300.0,
                                   5000.0,
                                   8100.0};

    depth = depth * (depth > 0);  // Ensure that depth is positive;

    double position = (3. * (depth / 0.03));

    // Linear extrapolation if the depth is at or greater than the maximum depth in the table (.030)
    if (position >= 3) {
        return (8100.0 + (depth - 0.03) * 8100.0/.03);
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
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    }
    else {
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
