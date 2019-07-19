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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Vehicle wheel constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Wheel::Wheel(const std::string& filename) : ChWheel(""), m_radius(0), m_width(0), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

Wheel::Wheel(const rapidjson::Document& d) : ChWheel(""), m_radius(0), m_width(0), m_has_mesh(false) {
    Create(d);
}

void Wheel::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read mass and inertia
    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    // Check how to visualize this wheel.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename")) {
            m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
            m_meshName = d["Visualization"]["Mesh Name"].GetString();
            m_has_mesh = true;
        }

        m_radius = d["Visualization"]["Radius"].GetDouble();
        m_width = d["Visualization"]["Width"].GetDouble();
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_spindle->AddAsset(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_trimesh_shape);
    if (it != m_spindle->GetAssets().end())
        m_spindle->GetAssets().erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
