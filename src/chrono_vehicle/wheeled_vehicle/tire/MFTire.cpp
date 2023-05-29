// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MFTire tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/MFTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

MFTire::MFTire(const std::string& filename) : ChMFTire(""), m_mass(0), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MFTire::MFTire(const rapidjson::Document& d) : ChMFTire(""), m_mass(0), m_has_mesh(false) {
    Create(d);
}

void MFTire::Create(const rapidjson::Document& d) {
    // Invoke base class method
    ChPart::Create(d);

    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    // Check if TIR specification file provided
    if (d.HasMember("TIR Specification File")) {
        m_tir_file = d["TIR Specification File"].GetString();
    } else {
        // Tire parameters explicitly specified in JSON file

        //// TODO
    }

    m_visualization_width = 0;
    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename Left") && d["Visualization"].HasMember("Mesh Filename Right")) {
            m_meshFile_left = d["Visualization"]["Mesh Filename Left"].GetString();
            m_meshFile_right = d["Visualization"]["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }

        if (d["Visualization"].HasMember("Width")) {
            m_visualization_width = d["Visualization"]["Width"].GetDouble();
        }
    }
}

void MFTire::SetMFParams() {
    if (!m_tir_file.empty()) {
        SetMFParamsByFile(vehicle::GetDataFile(m_tir_file));
    } else {
        //// TODO
    }
}

void MFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChMFTire::AddVisualizationAssets(vis);
    }
}

void MFTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChMFTire::RemoveVisualizationAssets();
}

}  // namespace vehicle
}  // namespace chrono
