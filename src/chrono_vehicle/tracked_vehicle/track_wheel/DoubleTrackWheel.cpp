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
// Double track-wheel model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/tracked_vehicle/track_wheel/DoubleTrackWheel.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DoubleTrackWheel::DoubleTrackWheel(const std::string& filename) : ChDoubleTrackWheel(""), m_has_mesh(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    std::cout << "Loaded JSON " << filename << std::endl;
}

DoubleTrackWheel::DoubleTrackWheel(const rapidjson::Document& d) : ChDoubleTrackWheel(""), m_has_mesh(false) {
    Create(d);
}

void DoubleTrackWheel::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read wheel geometry and mass properties
    assert(d.HasMember("Wheel"));
    m_wheel_radius = d["Wheel"]["Radius"].GetDouble();
    m_wheel_width = d["Wheel"]["Width"].GetDouble();
    m_wheel_gap = d["Wheel"]["Gap"].GetDouble();
    m_wheel_mass = d["Wheel"]["Mass"].GetDouble();
    m_wheel_inertia = ReadVectorJSON(d["Wheel"]["Inertia"]);

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
void DoubleTrackWheel::CreateContactMaterial(ChContactMethod contact_method) {
    m_material = m_mat_info.CreateMaterial(contact_method);
}

void DoubleTrackWheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetVehicleDataFile(m_meshFile), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetMutable(false);
        m_wheel->AddVisualShape(trimesh_shape);
    } else {
        ChDoubleTrackWheel::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
