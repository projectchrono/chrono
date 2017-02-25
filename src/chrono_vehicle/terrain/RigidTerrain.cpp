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
#include <cmath>

#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceDEM.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_thirdparty/Easy_BMP/EasyBMP.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
RigidTerrain::RigidTerrain(ChSystem* system)
    : m_type(FLAT),
      m_height(0),
      m_vis_enabled(true),
      m_friction(0.7f),
      m_restitution(0.1f),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_kn(2e5f),
      m_gn(40),
      m_kt(2e5f),
      m_gt(20) {
    // Create the ground body and add it to the system.
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetIdentifier(-1);
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);
    system->AddBody(m_ground);

    // Create the default color asset
    m_color = std::make_shared<ChColorAsset>();
    m_color->SetColor(ChColor(1, 1, 1));
    m_ground->AddAsset(m_color);
}

// -----------------------------------------------------------------------------
// Constructor from JSON file
// -----------------------------------------------------------------------------
static ChColor loadColor(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);

    return ChColor(a[0u].GetFloat(), a[1u].GetFloat(), a[2u].GetFloat());
}

RigidTerrain::RigidTerrain(ChSystem* system, const std::string& filename)
    : m_friction(0.7f),
      m_restitution(0.1f),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_kn(2e5f),
      m_gn(40),
      m_kt(2e5f),
      m_gt(20) {
    // Create the ground body and add it to the system.
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetIdentifier(-1);
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(true);
    system->AddBody(m_ground);

    // Create the default color asset
    m_color = std::make_shared<ChColorAsset>();
    m_color->SetColor(ChColor(1, 1, 1));
    m_ground->AddAsset(m_color);

    // Open the JSON file and read data
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

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

    // Read visualization data
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Color")) {
            ChColor color = loadColor(d["Visualization"]["Color"]);
            SetColor(color);
        }

        if (d["Visualization"].HasMember("Texture File")) {
            std::string tex_file = d["Visualization"]["Texture File"].GetString();
            float sx = 1;
            float sy = 1;
            if (d["Visualization"].HasMember("Texture Scaling")) {
                sx = d["Visualization"]["Texture Scaling"][0u].GetFloat();
                sy = d["Visualization"]["Texture Scaling"][1u].GetFloat();
            }
            SetTexture(vehicle::GetDataFile(tex_file), sx, sy);
        }
    }

    // Read geometry and initialize terrain
    m_vis_enabled = true;
    if (d["Geometry"].HasMember("Height")) {
        double sx = d["Geometry"]["Size"][0u].GetDouble();
        double sy = d["Geometry"]["Size"][1u].GetDouble();
        double h = d["Geometry"]["Height"].GetDouble();
        Initialize(h, sx, sy);
    } else if (d["Geometry"].HasMember("Mesh Filename")) {
        std::string mesh_file = d["Geometry"]["Mesh Filename"].GetString();
        std::string mesh_name = d["Geometry"]["Mesh Name"].GetString();
        Initialize(vehicle::GetDataFile(mesh_file), mesh_name);
    } else if (d["Geometry"].HasMember("Height Map Filename")) {
        std::string bmp_file = d["Geometry"]["Height Map Filename"].GetString();
        std::string mesh_name = d["Geometry"]["Mesh Name"].GetString();
        double sx = d["Geometry"]["Size"][0u].GetDouble();
        double sy = d["Geometry"]["Size"][1u].GetDouble();
        double hMin = d["Geometry"]["Height Range"][0u].GetDouble();
        double hMax = d["Geometry"]["Height Range"][1u].GetDouble();
        Initialize(vehicle::GetDataFile(bmp_file), mesh_name, sx, sy, hMin, hMax);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigidTerrain::SetContactMaterialProperties(float young_modulus, float poisson_ratio) {
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

void RigidTerrain::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
    m_kn = kn;
    m_gn = gn;
    m_kt = kt;
    m_gt = gt;
}

// -----------------------------------------------------------------------------
// Set the material properties, according to the underlying contact model
// -----------------------------------------------------------------------------
void RigidTerrain::ApplyContactMaterial() {
    switch (m_ground->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            m_ground->GetMaterialSurface()->SetFriction(m_friction);
            m_ground->GetMaterialSurface()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurfaceBase::DEM:
            m_ground->GetMaterialSurfaceDEM()->SetFriction(m_friction);
            m_ground->GetMaterialSurfaceDEM()->SetRestitution(m_restitution);
            m_ground->GetMaterialSurfaceDEM()->SetYoungModulus(m_young_modulus);
            m_ground->GetMaterialSurfaceDEM()->SetPoissonRatio(m_poisson_ratio);
            m_ground->GetMaterialSurfaceDEM()->SetKn(m_kn);
            m_ground->GetMaterialSurfaceDEM()->SetGn(m_gn);
            m_ground->GetMaterialSurfaceDEM()->SetKt(m_kt);
            m_ground->GetMaterialSurfaceDEM()->SetGt(m_gt);
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
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(tex_file);
    texture->SetTextureScale(tex_scale_x, tex_scale_y);
    m_ground->AddAsset(texture);
}

// -----------------------------------------------------------------------------
// Initialize the terrain as a rigid box
// -----------------------------------------------------------------------------
void RigidTerrain::Initialize(double height, double sizeX, double sizeY, bool tiled, double max_tile_size) {
    double depth = 10;

    m_ground->GetCollisionModel()->ClearModel();
    if (tiled) {
        int nX = (int)std::ceil(sizeX / max_tile_size);
        int nY = (int)std::ceil(sizeY / max_tile_size);
        double sizeX1 = sizeX / nX;
        double sizeY1 = sizeY / nY;
        for (int ix = 0; ix < nX; ix++) {
            for (int iy = 0; iy < nY; iy++) {
                m_ground->GetCollisionModel()->AddBox(
                    0.5 * sizeX1, 0.5 * sizeY1, 0.5 * depth,
                    ChVector<>((sizeX1 - sizeX) / 2 + ix * sizeX1, (sizeY1 - sizeY) / 2 + iy * sizeY1,
                               height - 0.5 * depth));
            }
        }
    } else {
        m_ground->GetCollisionModel()->AddBox(0.5 * sizeX, 0.5 * sizeY, 0.5 * depth,
                                              ChVector<>(0, 0, height - 0.5 * depth));
    }
    m_ground->GetCollisionModel()->BuildModel();

    ApplyContactMaterial();

    if (m_vis_enabled) {
        auto box = std::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = ChVector<>(0.5 * sizeX, 0.5 * sizeY, 0.5 * depth);
        box->GetBoxGeometry().Pos = ChVector<>(0, 0, height - 0.5 * depth);
        m_ground->AddAsset(box);
    }

    m_type = FLAT;
    m_height = height;
}

// -----------------------------------------------------------------------------
// Initialize the terrain from a specified mesh file.
// -----------------------------------------------------------------------------
void RigidTerrain::Initialize(const std::string& mesh_file, const std::string& mesh_name, double sweep_sphere_radius) {
    m_trimesh.LoadWavefrontMesh(mesh_file, true, true);

    // Create the visualization asset.
    if (m_vis_enabled) {
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(m_trimesh);
        trimesh_shape->SetName(mesh_name);
        m_ground->AddAsset(trimesh_shape);
    }

    // Create contact geometry.
    m_ground->GetCollisionModel()->ClearModel();
    m_ground->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0), ChMatrix33<>(1), sweep_sphere_radius);
    m_ground->GetCollisionModel()->BuildModel();

    ApplyContactMaterial();

    m_mesh_name = mesh_name;
    m_type = MESH;
}

// -----------------------------------------------------------------------------
// Initialize the terrain from a specified height map.
// -----------------------------------------------------------------------------
void RigidTerrain::Initialize(const std::string& heightmap_file,
                              const std::string& mesh_name,
                              double sizeX,
                              double sizeY,
                              double hMin,
                              double hMax) {
    // Read the BMP file and extract number of pixels.
    BMP hmap;
    if (!hmap.ReadFromFile(heightmap_file.c_str())) {
        throw ChException("Cannot open height map BMP file");
    }
    int nv_x = hmap.TellWidth();
    int nv_y = hmap.TellHeight();

    // Construct a triangular mesh of sizeX x sizeY.
    // Each pixel in the BMP represents a vertex.
    // The gray level of a pixel is mapped to the height range, with black corresponding
    // to hMin and white corresponding to hMax.
    // UV coordinates are mapped in [0,1] x [0,1].
    // We use smoothed vertex normals.
    double dx = sizeX / (nv_x - 1);
    double dy = sizeY / (nv_y - 1);
    double h_scale = (hMax - hMin) / 255;
    double x_scale = 1.0 / (nv_x - 1);
    double y_scale = 1.0 / (nv_y - 1);
    unsigned int n_verts = nv_x * nv_y;
    unsigned int n_faces = 2 * (nv_x - 1) * (nv_y - 1);

    // Resize mesh arrays.
    m_trimesh.getCoordsVertices().resize(n_verts);
    m_trimesh.getCoordsNormals().resize(n_verts);
    m_trimesh.getCoordsUV().resize(n_verts);
    m_trimesh.getCoordsColors().resize(n_verts);

    m_trimesh.getIndicesVertexes().resize(n_faces);
    m_trimesh.getIndicesNormals().resize(n_faces);

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Readibility aliases
    std::vector<ChVector<> >& vertices = m_trimesh.getCoordsVertices();
    std::vector<ChVector<> >& normals = m_trimesh.getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = m_trimesh.getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = m_trimesh.getIndicesNormals();

    // Load mesh vertices.
    // Note that pixels in a BMP start at top-left corner.
    // We order the vertices starting at the bottom-left corner, row after row.
    // The bottom-left corner corresponds to the point (-sizeX/2, -sizeY/2).
    std::cout << "Load vertices..." << std::endl;
    unsigned int iv = 0;
    for (int iy = nv_y - 1; iy >= 0; --iy) {
        double y = 0.5 * sizeY - iy * dy;
        for (int ix = 0; ix < nv_x; ++ix) {
            double x = ix * dx - 0.5 * sizeX;
            // Calculate equivalent gray level (RGB -> YUV)
            ebmpBYTE red = hmap(ix, iy)->Red;
            ebmpBYTE green = hmap(ix, iy)->Green;
            ebmpBYTE blue = hmap(ix, iy)->Blue;
            double gray = 0.299 * red + 0.587 * green + 0.114 * blue;
            // Map gray level to vertex height
            double z = hMin + gray * h_scale;
            // Set vertex location
            vertices[iv] = ChVector<>(x, y, z);
            // Initialize vertex normal to (0, 0, 0).
            normals[iv] = ChVector<>(0, 0, 0);
            // Assign color white to all vertices
            m_trimesh.getCoordsColors()[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            m_trimesh.getCoordsUV()[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time).
    // Specify the face vertices counter-clockwise.
    // Set the normal indices same as the vertex indices.
    std::cout << "Load faces..." << std::endl;
    unsigned int it = 0;
    for (int iy = nv_y - 2; iy >= 0; --iy) {
        for (int ix = 0; ix < nv_x - 1; ++ix) {
            int v0 = ix + nv_x * iy;
            idx_vertices[it] = ChVector<int>(v0, v0 + nv_x + 1, v0 + nv_x);
            idx_normals[it] = ChVector<int>(v0, v0 + nv_x + 1, v0 + nv_x);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nv_x + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nv_x + 1);
            ++it;
        }
    }

    // Calculate normals and then average the normals from all adjacent faces.
    for (unsigned int it = 0; it < n_faces; ++it) {
        // Calculate the triangle normal as a normalized cross product.
        ChVector<> nrm = Vcross(vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]],
                                vertices[idx_vertices[it][2]] - vertices[idx_vertices[it][0]]);
        nrm.Normalize();
        // Increment the normals of all incident vertices by the face normal
        normals[idx_normals[it][0]] += nrm;
        normals[idx_normals[it][1]] += nrm;
        normals[idx_normals[it][2]] += nrm;
        // Increment the count of all incident vertices by 1
        accumulators[idx_normals[it][0]] += 1;
        accumulators[idx_normals[it][1]] += 1;
        accumulators[idx_normals[it][2]] += 1;
    }

    // Set the normals to the average values.
    for (unsigned int in = 0; in < n_verts; ++in) {
        normals[in] /= (double)accumulators[in];
    }

    // Create the visualization asset.
    if (m_vis_enabled) {
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(m_trimesh);
        trimesh_shape->SetName(mesh_name);
        m_ground->AddAsset(trimesh_shape);
    }

    // Create contact geometry.
    m_ground->GetCollisionModel()->ClearModel();
    m_ground->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0));
    m_ground->GetCollisionModel()->BuildModel();

    ApplyContactMaterial();

    m_mesh_name = mesh_name;
    m_type = HEIGHT_MAP;
}

// -----------------------------------------------------------------------------
// Export the terrain mesh (if any) as a macro in a PovRay include file.
// -----------------------------------------------------------------------------
void RigidTerrain::ExportMeshPovray(const std::string& out_dir) {
    switch (m_type) {
        case MESH:
            utils::WriteMeshPovray(m_trimesh, m_mesh_name, out_dir, ChColor(1, 1, 1));
            break;
        case HEIGHT_MAP:
            utils::WriteMeshPovray(m_trimesh, m_mesh_name, out_dir, ChColor(1, 1, 1), ChVector<>(0, 0, 0),
                                   ChQuaternion<>(1, 0, 0, 0), true);
            break;
        default:
                break;
    }
}

// -----------------------------------------------------------------------------
// Return the terrain height at the specified location
// -----------------------------------------------------------------------------
double RigidTerrain::GetHeight(double x, double y) const {
    switch (m_type) {
        case FLAT:
            return m_height;
        case MESH: {
            double height = 0;
            //// TODO
            return height;
        }
        case HEIGHT_MAP: {
            double height = 0;
            //// TODO
            return height;
        }
        default:
            return 0;
    }
}

// -----------------------------------------------------------------------------
// Return the terrain normal at the specified location
// -----------------------------------------------------------------------------
ChVector<> RigidTerrain::GetNormal(double x, double y) const {
    switch (m_type) {
        case FLAT:
            return ChVector<>(0, 0, 1);
        case MESH: {
            ChVector<> normal(0, 0, 1);
            //// TODO
            return normal;
        }
        case HEIGHT_MAP: {
            ChVector<> normal(0, 0, 1);
            //// TODO
            return normal;
        }
        default:
            return ChVector<>(0, 0, 1);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
