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
// Authors: Rainer Gericke
// =============================================================================
//
// HMMWV TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/hmmwv/HMMWV_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string HMMWV_TMeasyTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string HMMWV_TMeasyTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_TMeasyTire::HMMWV_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_TMeasyTire::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // weight per tire aprox. 10000 N -> LI = 108

    unsigned int li = 108;  // guessed from load spec. of the vehicle
    const double in2m = 0.0254;
    double h = (37.0 - 16.5) * in2m / 2.0;
    double w = 12.5 * in2m;
    double r = h / w;
    double rimdia = 16.5 * in2m;
    double m = 45.4;

    GuessTruck80Par(li,      // load index
                    w,       // tire width
                    r,       // aspect ratio
                    rimdia,  // rim diameter
                    m        // rubber mass
    );
    // optional: write a plot file (gnuplot) to check the characteristics
    // the plots are being generated at initialization of this class
    // look at the plot: gnuplot 37x12.5x16.5_FL.gpl
    // inside gnuplot use the command load '37x12.5x16.5_FL.gpl'
    std::string plotName = "37x12.5x16.5";
    plotName += this->GetName() + ".gpl";
    WritePlots(plotName, "37x12.5x16.5");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_TMeasyTire::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by WVP_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
