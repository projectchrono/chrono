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
// Double roller model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/roller/DoubleRoller.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DoubleRoller::DoubleRoller(const std::string& filename) : ChDoubleRoller(""), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

DoubleRoller::DoubleRoller(const rapidjson::Document& d) : ChDoubleRoller(""), m_has_mesh(false) {
    Create(d);
}

void DoubleRoller::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read wheel geometry and mass properties
    assert(d.HasMember("Roller"));
    m_roller_radius = d["Roller"]["Radius"].GetDouble();
    m_roller_width = d["Roller"]["Width"].GetDouble();
    m_roller_gap = d["Roller"]["Gap"].GetDouble();
    m_roller_mass = d["Roller"]["Mass"].GetDouble();
    m_roller_inertia = ReadVectorJSON(d["Roller"]["Inertia"]);

    // Read contact material data
    assert(d.HasMember("Contact Material"));
    m_mat_info = ReadMaterialInfoJSON(d["Contact Material"]);

    // Read wheel visualization
    if (d.HasMember("Visualization")) {
        assert(d["Visualization"].HasMember("Mesh"));
        m_meshFile = d["Visualization"]["Mesh"].GetString();
        m_has_mesh = true;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DoubleRoller::CreateContactMaterial(ChContactMethod contact_method) {
    m_material = m_mat_info.CreateMaterial(contact_method);
}

void DoubleRoller::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh =
            geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(m_meshFile), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetMutable(false);
        m_wheel->AddVisualShape(trimesh_shape);
    } else {
        ChDoubleRoller::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
