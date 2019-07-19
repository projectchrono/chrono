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

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SprocketBand::SprocketBand(const std::string& filename) : ChSprocketBand(""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

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

    // Read profile information
    assert(d.HasMember("Profile"));
    m_gear_outer_radius = d["Profile"]["Outer Radius"].GetDouble();
    m_gear_base_width = d["Profile"]["Base Width"].GetDouble();
    m_gear_tip_width = d["Profile"]["Tip Width"].GetDouble();
    m_gear_tooth_depth = d["Profile"]["Tooth Depth"].GetDouble();
    m_gear_arc_radius = d["Profile"]["Arc Radius"].GetDouble();
    m_gear_guide_wheel_width = d["Profile"]["Guide Wheel Width"].GetDouble();
    m_gear_guide_wheel_gap = d["Profile"]["Guide Wheel Gap"].GetDouble();
    m_gear_RA = d["Profile"]["Assembly Radius"].GetDouble();

    // Read contact material data
    assert(d.HasMember("Contact Material"));

    float mu = d["Contact Material"]["Coefficient of Friction"].GetFloat();
    float cr = d["Contact Material"]["Coefficient of Restitution"].GetFloat();

    SetContactFrictionCoefficient(mu);
    SetContactRestitutionCoefficient(cr);

    if (d["Contact Material"].HasMember("Properties")) {
        float ym = d["Contact Material"]["Properties"]["Young Modulus"].GetFloat();
        float pr = d["Contact Material"]["Properties"]["Poisson Ratio"].GetFloat();
        SetContactMaterialProperties(ym, pr);
    }
    if (d["Contact Material"].HasMember("Coefficients")) {
        float kn = d["Contact Material"]["Coefficients"]["Normal Stiffness"].GetFloat();
        float gn = d["Contact Material"]["Coefficients"]["Normal Damping"].GetFloat();
        float kt = d["Contact Material"]["Coefficients"]["Tangential Stiffness"].GetFloat();
        float gt = d["Contact Material"]["Coefficients"]["Tangential Damping"].GetFloat();
        SetContactMaterialCoefficients(kn, gn, kt, gt);
    }

    // Read sprocket visualization
    if (d.HasMember("Visualization")) {
        assert(d["Visualization"].HasMember("Mesh Filename"));
        assert(d["Visualization"].HasMember("Mesh Name"));
        m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
        m_meshName = d["Visualization"]["Mesh Name"].GetString();
        m_has_mesh = true;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SprocketBand::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        trimesh_shape->SetStatic(true);
        m_gear->AddAsset(trimesh_shape);
    } else {
        ChSprocketBand::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
