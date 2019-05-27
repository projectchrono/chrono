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
// Pacejka tire constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/PacejkaTire.h"
//
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
//
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
PacejkaTire::PacejkaTire(const std::string& filename) : ChPacejkaTire("", ""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

PacejkaTire::PacejkaTire(const rapidjson::Document& d) : ChPacejkaTire("", ""), m_has_mesh(false) {
    Create(d);
}

void PacejkaTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    // Read name of Pacejka parameter file
    assert(d.HasMember("Pacejka Parameter File"));
    m_paramFile = vehicle::GetDataFile(d["Pacejka Parameter File"].GetString());

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
void PacejkaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_trimesh_shape->SetStatic(true);
        m_wheel->AddAsset(m_trimesh_shape);
    } else {
        ChPacejkaTire::AddVisualizationAssets(vis);
    }
}

void PacejkaTire::RemoveVisualizationAssets() {
    ChPacejkaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by PacejkaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

}  // end namespace vehicle
}  // end namespace chrono
