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
// Vehicle rigid chassis model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChCompositeInertia.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/RigidChassis.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);

    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigidChassis::RigidChassis(const std::string& filename) : ChChassis(""), m_has_mesh(false) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

RigidChassis::RigidChassis(const rapidjson::Document& d) : ChChassis(""), m_has_mesh(false) {
    Create(d);
}

void RigidChassis::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    SetName(d["Name"].GetString());

    // Read inertia properties for all sub-somponents
    // and calculate composite inertia properties
    assert(d.HasMember("Components"));
    assert(d["Components"].IsArray());
    int num_comp = d["Components"].Size();

    utils::CompositeInertia composite;

    for (int i = 0; i < num_comp; i++) {
        const Value& comp = d["Components"][i];
        ChVector<> loc = loadVector(comp["Centroidal Frame"]["Location"]);
        ChQuaternion<> rot = loadQuaternion(comp["Centroidal Frame"]["Orientation"]);
        double mass = comp["Mass"].GetDouble();
        ChVector<> inertiaXX = loadVector(comp["Moments of Inertia"]);
        ChVector<> inertiaXY = loadVector(comp["Products of Inertia"]);
        bool is_void = comp["Void"].GetBool();

        ChMatrix33<> inertia(inertiaXX);
        inertia.SetElement(0, 1, inertiaXY.x());
        inertia.SetElement(0, 2, inertiaXY.y());
        inertia.SetElement(1, 2, inertiaXY.z());
        inertia.SetElement(1, 0, inertiaXY.x());
        inertia.SetElement(2, 0, inertiaXY.y());
        inertia.SetElement(2, 1, inertiaXY.z());

        composite.AddComponent(ChFrame<>(loc, rot), mass, inertia, is_void);
    }

    m_mass = composite.GetMass();
    m_inertia = composite.GetInertia();
    m_COM_loc = composite.GetCOM();

    // Extract driver position
    m_driverCsys.pos = loadVector(d["Driver Position"]["Location"]);
    m_driverCsys.rot = loadQuaternion(d["Driver Position"]["Orientation"]);

    // Read chassis visualization
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
void RigidChassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        m_body->AddAsset(trimesh_shape);
    } else {
        ChChassis::AddVisualizationAssets(vis);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
