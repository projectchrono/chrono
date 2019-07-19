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
// Authors: Radu Serban
// =============================================================================
//
// LuGre tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LugreTire::LugreTire(const std::string& filename) : ChLugreTire(""), m_discLocs(NULL), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

LugreTire::LugreTire(const rapidjson::Document& d) : ChLugreTire(""), m_discLocs(NULL), m_has_mesh(false) {
    Create(d);
}

LugreTire::~LugreTire() {
    delete m_discLocs;
}

void LugreTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read tire radius, mass, and inertia
    m_radius = d["Radius"].GetDouble();
    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    // Read disc locations
    m_numDiscs = d["Disc Locations"].Size();
    m_discLocs = new double[m_numDiscs];
    for (int i = 0; i < m_numDiscs; i++) {
        m_discLocs[i] = d["Disc Locations"][SizeType(i)].GetDouble();
    }

    // Read normal stiffness and damping
    m_normalStiffness = d["Normal Stiffness"].GetDouble();
    m_normalDamping = d["Normal Damping"].GetDouble();

    // Read LuGre model parameters
    m_sigma0[0] = d["Lugre Parameters"]["sigma0"][0u].GetDouble();  // longitudinal
    m_sigma0[1] = d["Lugre Parameters"]["sigma0"][1u].GetDouble();  // lateral

    m_sigma1[0] = d["Lugre Parameters"]["sigma1"][0u].GetDouble();  // longitudinal
    m_sigma1[1] = d["Lugre Parameters"]["sigma1"][1u].GetDouble();  // lateral

    m_sigma2[0] = d["Lugre Parameters"]["sigma2"][0u].GetDouble();  // longitudinal
    m_sigma2[1] = d["Lugre Parameters"]["sigma2"][1u].GetDouble();  // lateral

    m_Fc[0] = d["Lugre Parameters"]["Fc"][0u].GetDouble();  // longitudinal
    m_Fc[1] = d["Lugre Parameters"]["Fc"][1u].GetDouble();  // lateral

    m_Fs[0] = d["Lugre Parameters"]["Fs"][0u].GetDouble();  // longitudinal
    m_Fs[1] = d["Lugre Parameters"]["Fs"][1u].GetDouble();  // lateral

    m_vs[0] = d["Lugre Parameters"]["vs"][0u].GetDouble();  // longitudinal
    m_vs[1] = d["Lugre Parameters"]["vs"][1u].GetDouble();  // lateral

    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename")) {
            m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
            m_meshName = d["Visualization"]["Mesh Name"].GetString();
            m_has_mesh = true;
        }
    }
}

// -----------------------------------------------------------------------------
void LugreTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    }
    else {
        ChLugreTire::AddVisualizationAssets(vis);
    }
}

void LugreTire::RemoveVisualizationAssets() {
    ChLugreTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by LugreTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
