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
// Double-pin track shoe constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/TrackShoeDoublePin.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TrackShoeDoublePin::TrackShoeDoublePin(const std::string& filename) : ChTrackShoeDoublePin(""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

TrackShoeDoublePin::TrackShoeDoublePin(const rapidjson::Document& d) : ChTrackShoeDoublePin(""), m_has_mesh(false) {
    Create(d);
}

void TrackShoeDoublePin::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read shoe body geometry and mass properties
    assert(d.HasMember("Shoe"));

    m_shoe_length = d["Shoe"]["Length"].GetDouble();
    m_shoe_width = d["Shoe"]["Width"].GetDouble();
    m_shoe_height = d["Shoe"]["Height"].GetDouble();
    m_shoe_mass = d["Shoe"]["Mass"].GetDouble();
    m_shoe_inertia = ReadVectorJSON(d["Shoe"]["Inertia"]);

    // Read connector body geometry and mass properties
    assert(d.HasMember("Connector"));
    m_connector_radius = d["Connector"]["Radius"].GetDouble();
    m_connector_length = d["Connector"]["Length"].GetDouble();
    m_connector_width = d["Connector"]["Width"].GetDouble();
    m_connector_mass = d["Connector"]["Mass"].GetDouble();
    m_connector_inertia = ReadVectorJSON(d["Connector"]["Inertia"]);

    // Read contact geometry data
    assert(d.HasMember("Contact Geometry"));
    assert(d["Contact Geometry"].HasMember("Shoe"));

    m_pad_box_dims = ReadVectorJSON(d["Contact Geometry"]["Shoe"]["Pad Dimensions"]);
    m_pad_box_loc = ReadVectorJSON(d["Contact Geometry"]["Shoe"]["Pad Location"]);
    m_guide_box_dims = ReadVectorJSON(d["Contact Geometry"]["Shoe"]["Guide Dimensions"]);
    m_guide_box_loc = ReadVectorJSON(d["Contact Geometry"]["Shoe"]["Guide Location"]);

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

    // Read wheel visualization
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
void TrackShoeDoublePin::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        trimesh_shape->SetStatic(true);
        m_shoe->AddAsset(trimesh_shape);
    } else {
        ChTrackShoeDoublePin::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
