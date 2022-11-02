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
// Band-ANCF track shoe constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeBandANCF.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
TrackShoeBandANCF::TrackShoeBandANCF(const std::string& filename) : ChTrackShoeBandANCF(""), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackShoeBandANCF::TrackShoeBandANCF(const rapidjson::Document& d) : ChTrackShoeBandANCF(""), m_has_mesh(false) {
    Create(d);
}

void TrackShoeBandANCF::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    assert(d.HasMember("Belt Width"));
    m_belt_width = d["Belt Width"].GetDouble();

    assert(d.HasMember("Shoe Height"));
    m_shoe_height = d["Shoe Height"].GetDouble();

    // Read tread body geometry and mass properties
    assert(d.HasMember("Tread"));
    m_tread_mass = d["Tread"]["Mass"].GetDouble();
    m_tread_inertias = ReadVectorJSON(d["Tread"]["Inertia"]);
    m_tread_length = d["Tread"]["Length"].GetDouble();
    m_tread_thickness = d["Tread"]["Thickness"].GetDouble();
    m_tooth_tip_length = d["Tread"]["Tooth Tip Length"].GetDouble();
    m_tooth_base_length = d["Tread"]["Tooth Base Length"].GetDouble();
    m_tooth_width = d["Tread"]["Tooth Width"].GetDouble();
    m_tooth_height = d["Tread"]["Tooth Height"].GetDouble();
    m_tooth_arc_radius = d["Tread"]["Tooth Arc Radius"].GetDouble();

    // Read web geometry and mass properties
    assert(d.HasMember("Web"));
    m_num_elements_length = d["Web"]["Number Elements Length"].GetInt();
    m_num_elements_width = d["Web"]["Number Elements Width"].GetInt();

    m_web_mass = d["Web"]["Mass"].GetDouble();
    m_web_inertias = ReadVectorJSON(d["Web"]["Inertia"]);
    m_web_length = d["Web"]["Length"].GetDouble();
    m_web_thickness = d["Web"]["Thickness"].GetDouble();
    m_steel_thickness = d["Web"]["Steel Layer Thickness"].GetDouble();

    // Read guide pin geometry
    assert(d.HasMember("Guide Pin"));
    m_guide_box_dims = ReadVectorJSON(d["Guide Pin"]["Dimensions"]);
    m_guide_box_offset_x = d["Guide Pin"]["Offset"].GetDouble();

    // Read contact material information
    assert(d.HasMember("Contact Materials"));
    assert(d["Contact Materials"].HasMember("Pad Material"));
    assert(d["Contact Materials"].HasMember("Body Material"));
    assert(d["Contact Materials"].HasMember("Guide Material"));
    assert(d["Contact Materials"].HasMember("Tooth Material"));

    m_pad_matinfo = ReadMaterialInfoJSON(d["Contact Materials"]["Pad Material"]);
    m_body_matinfo = ReadMaterialInfoJSON(d["Contact Materials"]["Body Material"]);
    m_guide_matinfo = ReadMaterialInfoJSON(d["Contact Materials"]["Guide Material"]);
    m_tooth_matinfo = ReadMaterialInfoJSON(d["Contact Materials"]["Tooth Material"]);

    // Read wheel visualization
    if (d.HasMember("Visualization")) {
        assert(d["Visualization"].HasMember("Mesh"));
        m_meshFile = d["Visualization"]["Mesh"].GetString();
        m_has_mesh = true;
    }

    // Set name for procedurally-generated tread visualization mesh.
    m_tread_meshName = GetName();
}

// -----------------------------------------------------------------------------
void TrackShoeBandANCF::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh =
            geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(m_meshFile), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetMutable(false);
        m_shoe->AddVisualShape(trimesh_shape);
    } else {
        ChTrackShoeBandANCF::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
