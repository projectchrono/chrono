// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Tracked vehicle single-pin sprocket model constructed with data from file
// (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/sprocket/SprocketSinglePin.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// This utility function returns a ChVector from the specified JSON array
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);

    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SprocketSinglePin::SprocketSinglePin(const std::string& filename) : ChSprocketSinglePin(""), m_vis_type(VisualizationType::NONE) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

SprocketSinglePin::SprocketSinglePin(const rapidjson::Document& d) : ChSprocketSinglePin(""), m_vis_type(VisualizationType::NONE) {
    Create(d);
}

void SprocketSinglePin::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Read inertia properties
    m_num_teeth = d["Number Teeth"].GetInt();
    m_gear_mass = d["Gear Mass"].GetDouble();
    m_gear_inertia = loadVector(d["Gear Inertia"]);
    m_axle_inertia = d["Axle Inertia"].GetDouble();
    m_separation = d["Gear Separation"].GetDouble();

    // Read profile information
    assert(d.HasMember("Profile"));
    m_gear_RT = d["Profile"]["Addenum Radius"].GetDouble();
    m_gear_R = d["Profile"]["Arc Radius"].GetDouble();
    m_gear_RC = d["Profile"]["Arc Centers Radius"].GetDouble();
    m_gear_RA = d["Profile"]["Assembly Radius"].GetDouble();

    // Read contact material data
    assert(d.HasMember("Contact Material"));

    float mu = d["Contact Material"]["Coefficient of Friction"].GetDouble();
    float cr = d["Contact Material"]["Coefficient of Restitution"].GetDouble();

    SetContactFrictionCoefficient(mu);
    SetContactRestitutionCoefficient(cr);

    if (d["Contact Material"].HasMember("Properties")) {
        float ym = d["Contact Material"]["Properties"]["Young Modulus"].GetDouble();
        float pr = d["Contact Material"]["Properties"]["Poisson Ratio"].GetDouble();
        SetContactMaterialProperties(ym, pr);
    }
    if (d["Contact Material"].HasMember("Coefficients")) {
        float kn = d["Contact Material"]["Coefficients"]["Normal Stiffness"].GetDouble();
        float gn = d["Contact Material"]["Coefficients"]["Normal Damping"].GetDouble();
        float kt = d["Contact Material"]["Coefficients"]["Tangential Stiffness"].GetDouble();
        float gt = d["Contact Material"]["Coefficients"]["Tangential Damping"].GetDouble();
        SetContactMaterialCoefficients(kn, gn, kt, gt);
    }

    // Read sprocket visualization
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename")) {
            m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
            m_meshName = d["Visualization"]["Mesh Name"].GetString();
            m_vis_type = VisualizationType::MESH;
        } else {
            m_vis_type = VisualizationType::PRIMITIVES;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SprocketSinglePin::AddGearVisualization() {
    switch (m_vis_type) {
        case VisualizationType::PRIMITIVES:
            ChSprocket::AddGearVisualization();
            break;
        case VisualizationType::MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_meshName);
            m_gear->AddAsset(trimesh_shape);
            break;
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
