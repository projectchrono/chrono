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
// Tracked vehicle continuous-band sprocket model constructed with data from file
// (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketBand.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SprocketBand::SprocketBand(const std::string& filename) : ChSprocketBand(""), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SprocketBand::SprocketBand(const rapidjson::Document& d) : ChSprocketBand(""), m_has_mesh(false) {
    Create(d);
}

void SprocketBand::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read inertia properties
    m_num_teeth = d["Number Teeth"].GetInt();
    m_gear_mass = d["Gear Mass"].GetDouble();
    m_gear_inertia = ReadVectorJSON(d["Gear Inertia"]);
    m_axle_inertia = d["Axle Inertia"].GetDouble();
    m_separation = d["Gear Separation"].GetDouble();

    // Read lateral backlash (for contact against detracking)
    m_lateral_backlash = d["Lateral Backlash"].GetDouble();

    // Read profile information
    assert(d.HasMember("Profile"));
    m_gear_outer_radius = d["Profile"]["Outer Radius"].GetDouble();
    m_gear_base_width = d["Profile"]["Base Width"].GetDouble();
    m_gear_tip_width = d["Profile"]["Tip Width"].GetDouble();
    m_gear_tooth_depth = d["Profile"]["Tooth Depth"].GetDouble();
    m_gear_arc_radius = d["Profile"]["Arc Radius"].GetDouble();
    m_gear_RA = d["Profile"]["Assembly Radius"].GetDouble();

    // Read contact material data
    assert(d.HasMember("Contact Material"));
    m_mat_info = ReadMaterialInfoJSON(d["Contact Material"]);

    // Read sprocket visualization
    if (d.HasMember("Visualization")) {
        assert(d["Visualization"].HasMember("Mesh"));
        m_meshFile = d["Visualization"]["Mesh"].GetString();
        m_has_mesh = true;
    }
}

void SprocketBand::CreateContactMaterial(ChContactMethod contact_method) {
    m_material = m_mat_info.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SprocketBand::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh =
            geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(m_meshFile), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_meshFile).stem());
        trimesh_shape->SetMutable(false);
        m_gear->AddVisualShape(trimesh_shape);
    } else {
        ChSprocketBand::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
