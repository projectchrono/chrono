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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Vehicle wheel constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "thirdparty/rapidjson/filereadstream.h"

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
Wheel::Wheel(const std::string& filename) : m_vis(NONE) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

Wheel::Wheel(const rapidjson::Document& d) : m_vis(NONE), m_radius(0), m_width(0) {
    Create(d);
}

void Wheel::Create(const rapidjson::Document& d) {
    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Read mass and inertia
    m_mass = d["Mass"].GetDouble();
    m_inertia = loadVector(d["Inertia"]);

    // Check how to visualize this wheel.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename")) {
            m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
            m_meshName = d["Visualization"]["Mesh Name"].GetString();
            m_vis = MESH;
        } else {
            m_vis = PRIMITIVES;
        }

        m_radius = d["Visualization"]["Radius"].GetDouble();
        m_width = d["Visualization"]["Width"].GetDouble();
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Wheel::Initialize(std::shared_ptr<ChBody> spindle) {
    // Call the base class initialization function
    ChWheel::Initialize(spindle);

    // Attach visualization
    switch (m_vis) {
        case PRIMITIVES: {
            auto cyl = std::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().rad = m_radius;
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, m_width / 2, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -m_width / 2, 0);
            spindle->AddAsset(cyl);

            auto tex = std::make_shared<ChTexture>();
            tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            spindle->AddAsset(tex);

            break;
        }
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);

            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_meshName);
            spindle->AddAsset(trimesh_shape);

            auto mcolor = std::make_shared<ChColorAsset>(0.3f, 0.3f, 0.3f);
            spindle->AddAsset(mcolor);

            break;
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
