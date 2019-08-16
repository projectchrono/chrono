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
// UAZBUS TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/uaz/UAZBUS_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string UAZBUS_TMeasyTireFront::m_meshName = "hmmwv_tire_POV_geom";
const std::string UAZBUS_TMeasyTireFront::m_meshFile = "hmmwv/hmmwv_tire.obj";

const double UAZBUS_TMeasyTireFront::m_mass = 37.6;
const ChVector<> UAZBUS_TMeasyTireFront::m_inertia(3.84, 6.69, 3.84);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_TMeasyTireFront::UAZBUS_TMeasyTireFront(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_TMeasyTireFront::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)
    
    const double lbs2N = 4.4482216153;
    unsigned int li = 121;  // UAZ data
    const double in2m = 0.0254;
    double w = 0.225;
    double r = 0.75;
    double rimdia = 16.0 * in2m;
    double pres_li = 590000;
    double pres_use = 220000;
    
    GuessTruck80Par(li,     // tire load index []
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia,  // rim diameter [m],
                    pres_li, // infl. pressure for load index
                    pres_use // infl. pressure for usage
    );
    
    
}

void UAZBUS_TMeasyTireFront::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/225_75R16_" + GetName() + ".gpl";
    WritePlots(filename, "225/75R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_TMeasyTireFront::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void UAZBUS_TMeasyTireFront::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by UAZBUS_TMeasyTireFront::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

const std::string UAZBUS_TMeasyTireRear::m_meshName = "hmmwv_tire_POV_geom";
const std::string UAZBUS_TMeasyTireRear::m_meshFile = "hmmwv/hmmwv_tire.obj";

const double UAZBUS_TMeasyTireRear::m_mass = 37.6;
const ChVector<> UAZBUS_TMeasyTireRear::m_inertia(3.84, 6.69, 3.84);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_TMeasyTireRear::UAZBUS_TMeasyTireRear(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_TMeasyTireRear::SetTMeasyParams() {
    // Tire Size = 37 x 12.5 x 16.5 Load Range D
    // Tire Load 3850 lbs at 50 psi (Goodyear Military Tire Brochure 6th Edition)
    
    const double lbs2N = 4.4482216153;
    unsigned int li = 121;  // UAZ data
    const double in2m = 0.0254;
    double w = 0.225;
    double r = 0.75;
    double rimdia = 16.0 * in2m;
    double pres_li = 590000;
    double pres_use = 240000;
    
    GuessTruck80Par(li,     // tire load index []
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia,  // rim diameter [m],
                    pres_li, // infl. pressure for load index
                    pres_use // infl. pressure for usage
    );
    
    
}

void UAZBUS_TMeasyTireRear::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/225_75R16_" + GetName() + ".gpl";
    WritePlots(filename, "225/75R20");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void UAZBUS_TMeasyTireRear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        trimesh->Transform(ChVector<>(0, GetOffset(), 0), ChMatrix33<>(1));
        m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->GetSpindle()->AddAsset(m_trimesh_shape);
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void UAZBUS_TMeasyTireRear::RemoveVisualizationAssets() {
    ChTMeasyTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by UAZBUS_TMeasyTireRear::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    auto it = std::find(assets.begin(), assets.end(), m_trimesh_shape);
    if (it != assets.end())
        assets.erase(it);
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
