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
// Authors: Rainer Gericke, Asher Elmquist, Evan Hoerl, Shuo He
// =============================================================================
//
// CityBus TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/citybus/CityBus_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string CityBus_TMeasyTire::m_meshName = "citybus_tire_POV_geom";
const std::string CityBus_TMeasyTire::m_meshFile = "citybus/CityBusTire.obj";

const double CityBus_TMeasyTire::m_mass = 68.6;
const ChVector<> CityBus_TMeasyTire::m_inertia(6.104, 11.144, 6.104);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_TMeasyTire::CityBus_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_TMeasyTire::SetTMeasyParams() {
    const double lbs2N = 4.4482216153;
    unsigned int li = 152;
    const double in2m = 0.0254;
    double h = (1.05 - 16 * in2m) / 2.0;
    double w = 0.295;
    double r = h / w;
    double rimdia = 16.0 * in2m;
    double pinfl_li = 850.0 * 1000;
    double pinfl_use = 850.0 * 4000;

    double load = 45000;

    GuessTruck80Par(load,    // tire load [N]
                    w,       // tire width [m]
                    r,       // aspect ratio []
                    rimdia,  // rim diameter [m]
                    pinfl_li, pinfl_use);
}

void CityBus_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/37x12.5x16.5_" + GetName() + ".gpl";
    WritePlots(filename, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CityBus_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetStatic(true);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void CityBus_TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by WVP_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
