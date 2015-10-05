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
// Rigid terrain
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "thirdparty/rapidjson/document.h"
#include "thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(chrono::ChSystem* system) {
    // Create the ground body and add it to the system.
    m_ground = ChSharedPtr<ChBody>(new ChBody(system->GetContactMethod()));
    m_ground->SetIdentifier(-1);
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);
    system->AddBody(m_ground);

    // Create the default color asset
    m_color = ChSharedPtr<ChColorAsset>(new ChColorAsset);
    m_color->SetColor(ChColor(1, 1, 1));
    m_ground->AddAsset(m_color);
}

// -----------------------------------------------------------------------------
// Constructor from JSON file
// -----------------------------------------------------------------------------
static ChColor loadColor(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);

    return ChColor(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

RigidTerrain::RigidTerrain(chrono::ChSystem* system, const std::string& filename) {
    // Create the ground body and add it to the system.
    m_ground = ChSharedPtr<ChBody>(new ChBody(system->GetContactMethod()));
    m_ground->SetIdentifier(-1);
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);
    system->AddBody(m_ground);

    // Create the default color asset
    m_color = ChSharedPtr<ChColorAsset>(new ChColorAsset);
    m_color->SetColor(ChColor(1, 1, 1));
    m_ground->AddAsset(m_color);

    // Open the JSON file and read data
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Read material properties
    float mu = d["Material Properties"]["Coefficient of Friction"].GetDouble();
    float cr = d["Material Properties"]["Coefficient of Restitution"].GetDouble();
    float ym = d["Material Properties"]["Young Modulus"].GetDouble();
    float pr = d["Material Properties"]["Poisson Ratio"].GetDouble();
    SetContactMaterial(mu, cr, ym, pr);

    // Read visualization data
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Color")) {
            ChColor color = loadColor(d["Visualization"]["Color"]);
            SetColor(color);
        }

        if (d["Visualization"].HasMember("Texture File")) {
            std::string tex_file = d["Visualization"]["Texture File"].GetString();
            double sx = 1;
            double sy = 1;
            if (d["Visualization"].HasMember("Texture Scaling")) {
                sx = d["Visualization"]["Texture Scaling"][0u].GetDouble();
                sy = d["Visualization"]["Texture Scaling"][1u].GetDouble();
            }
            SetTexture(vehicle::GetDataFile(tex_file), sx, sy);
        }
    }

    // Read geometry and initialize terrain
    if (d["Geometry"].HasMember("Size")) {
        double sx = d["Geometry"]["Size"][0u].GetDouble();
        double sy = d["Geometry"]["Size"][1u].GetDouble();
        double h = d["Geometry"]["Height"].GetDouble();
        Initialize(h, sx, sy);
    } else {
        std::string mesh_file = d["Geometry"]["Mesh Filename"].GetString();
        std::string mesh_name = d["Geometry"]["Mesh Name"].GetString();
        Initialize(mesh_file, mesh_name);
    }
}

// -----------------------------------------------------------------------------
// Set the material properties, according to the underlying contact model
// -----------------------------------------------------------------------------
void RigidTerrain::SetContactMaterial(float friction_coefficient,
                                      float restitution_coefficient,
                                      float young_modulus,
                                      float poisson_ratio) {
    switch (m_ground->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            m_ground->GetMaterialSurface()->SetFriction(friction_coefficient);
            m_ground->GetMaterialSurface()->SetRestitution(restitution_coefficient);
            break;
        case ChMaterialSurfaceBase::DEM:
            m_ground->GetMaterialSurfaceDEM()->SetFriction(friction_coefficient);
            m_ground->GetMaterialSurfaceDEM()->SetRestitution(restitution_coefficient);
            m_ground->GetMaterialSurfaceDEM()->SetYoungModulus(young_modulus);
            m_ground->GetMaterialSurfaceDEM()->SetPoissonRatio(poisson_ratio);
            break;
    }
}

// -----------------------------------------------------------------------------
// Set the color of the visualization assets
// -----------------------------------------------------------------------------
void RigidTerrain::SetColor(ChColor color) {
    m_color->SetColor(color);
}

// -----------------------------------------------------------------------------
// Set the texture and texture scaling
// -----------------------------------------------------------------------------
void RigidTerrain::SetTexture(const std::string tex_file, float tex_scale_x, float tex_scale_y) {
    ChSharedPtr<ChTexture> texture(new ChTexture);
    texture->SetTextureFilename(tex_file);
    texture->SetTextureScale(tex_scale_x, tex_scale_y);
    m_ground->AddAsset(texture);
}

// -----------------------------------------------------------------------------
// Initialize the terrain as a rigid box
// -----------------------------------------------------------------------------
void RigidTerrain::Initialize(double height, double sizeX, double sizeY) {
    double depth = 10;
    m_ground->GetCollisionModel()->ClearModel();
    m_ground->GetCollisionModel()->AddBox(0.5 * sizeX, 0.5 * sizeY, 0.5 * depth,
                                          ChVector<>(0, 0, height - 0.5 * depth));
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().Size = ChVector<>(0.5 * sizeX, 0.5 * sizeY, 0.5 * depth);
    box->GetBoxGeometry().Pos = ChVector<>(0, 0, height - 0.5 * depth);
    m_ground->AddAsset(box);
    m_ground->GetCollisionModel()->BuildModel();

    m_isBox = true;
    m_height = height;
}

// -----------------------------------------------------------------------------
// Initialize the terrain as a rigid mesh
// -----------------------------------------------------------------------------
void RigidTerrain::Initialize(const std::string& mesh_file, const std::string& mesh_name) {
    //// TODO

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(mesh_file, false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(mesh_name);
    m_ground->AddAsset(trimesh_shape);

    m_isBox = false;
}

// -----------------------------------------------------------------------------
// Return the terrain height at the specified location
// -----------------------------------------------------------------------------
double RigidTerrain::GetHeight(double x, double y) const {
    if (m_isBox)
        return m_height;

    //// TODO
    return 0;
}

// -----------------------------------------------------------------------------
// Return the terrain normal at the specified location
// -----------------------------------------------------------------------------
ChVector<> RigidTerrain::GetNormal(double x, double y) const {
    if (m_isBox)
        return ChVector<>(0, 0, 1);

    //// TODO
    return ChVector<>(0, 0, 1);
}

}  // end namespace chrono
