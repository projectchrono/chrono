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
// Authors: Alessandro Tasora, Radu Serban
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
#include "chrono_vehicle/terrain/DeformableTerrain.h"

#include "thirdparty/Easy_BMP/EasyBMP.h"
#include "thirdparty/rapidjson/document.h"
#include "thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
DeformableTerrain::DeformableTerrain(ChSystem* system) {
    
    this->SetSystem(system);

    // Create the default mesh asset
    m_color = ChSharedPtr<ChColorAsset>(new ChColorAsset);
    m_color->SetColor(ChColor(1, 1, 1));
    this->AddAsset(m_color);

    // Create the default triangle mesh asset
    m_trimesh_shape = ChSharedPtr<ChTriangleMeshShape>(new ChTriangleMeshShape);
    this->AddAsset(m_trimesh_shape);
    m_trimesh_shape->SetWireframe(true);

    Initialize(0,3,3,10,10);
}



// -----------------------------------------------------------------------------
// Set the color of the visualization assets
// -----------------------------------------------------------------------------
void DeformableTerrain::SetColor(ChColor color) {
    m_color->SetColor(color);
}

// -----------------------------------------------------------------------------
// Set the texture and texture scaling
// -----------------------------------------------------------------------------
void DeformableTerrain::SetTexture(const std::string tex_file, float tex_scale_x, float tex_scale_y) {
    ChSharedPtr<ChTexture> texture(new ChTexture);
    texture->SetTextureFilename(tex_file);
    texture->SetTextureScale(tex_scale_x, tex_scale_y);
    this->AddAsset(texture);
}

// -----------------------------------------------------------------------------
// Initialize the terrain as a flat grid
// -----------------------------------------------------------------------------

void DeformableTerrain::Initialize(double height, double sizeX, double sizeY, int nX, int nY) {

    m_trimesh_shape->GetMesh().Clear();
    // Readibility aliases
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh().getCoordsVertices();
    std::vector<ChVector<> >& normals = m_trimesh_shape->GetMesh().getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = m_trimesh_shape->GetMesh().getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = m_trimesh_shape->GetMesh().getIndicesNormals();
    std::vector<ChVector<> >& uv_coords = m_trimesh_shape->GetMesh().getCoordsUV();
    std::vector<ChVector<float> >& colors = m_trimesh_shape->GetMesh().getCoordsColors();

    unsigned int nvx = nX+1;
    unsigned int nvy = nY+1;
    double dx = sizeX / nX;
    double dy = sizeY / nY;
    unsigned int n_verts = nvx * nvy;
    unsigned int n_faces = 2 * nX * nY;
    double x_scale = 1.0 / nX;
    double y_scale = 1.0 / nY;

    // Resize mesh arrays.
    vertices.resize(n_verts);
    normals.resize(n_verts);
    uv_coords.resize(n_verts);
    colors.resize(n_verts);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    unsigned int iv = 0;
    for (int iy = nvy-1; iy >= 0; --iy) {
        double y = 0.5 * sizeY - iy * dy;
        for (int ix = 0; ix < nvx; ++ix) {
            double x = ix * dx - 0.5 * sizeX;
            // Set vertex location
            vertices[iv] = ChVector<>(x, height, y);
            // Initialize vertex normal to Y up
            normals[iv] = ChVector<>(0, 1, 0);
            // Assign color white to all vertices
            colors[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            uv_coords[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
            ++iv;
        }
    }
    unsigned int it = 0;
    for (int iy = nvy - 2; iy >= 0; --iy) {
        for (int ix = 0; ix < nvx - 1; ++ix) {
            int v0 = ix + nvx * iy;
            idx_vertices[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            idx_normals[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            ++it;
        }
    }

}

// -----------------------------------------------------------------------------
// Initialize the terrain from a specified .obj mesh file.
// -----------------------------------------------------------------------------
void DeformableTerrain::Initialize(const std::string& mesh_file) {

    m_trimesh_shape->GetMesh().Clear();
    m_trimesh_shape->GetMesh().LoadWavefrontMesh(mesh_file, true, true);

}

// -----------------------------------------------------------------------------
// Initialize the terrain from a specified height map.
// -----------------------------------------------------------------------------
void DeformableTerrain::Initialize(const std::string& heightmap_file,
                              const std::string& mesh_name,
                              double sizeX,
                              double sizeY,
                              double hMin,
                              double hMax) {
    m_trimesh_shape->GetMesh().Clear();

    // Read the BMP file nd extract number of pixels.
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
    m_trimesh_shape->GetMesh().getCoordsVertices().resize(n_verts);
    m_trimesh_shape->GetMesh().getCoordsNormals().resize(n_verts);
    m_trimesh_shape->GetMesh().getCoordsUV().resize(n_verts);
    m_trimesh_shape->GetMesh().getCoordsColors().resize(n_verts);

    m_trimesh_shape->GetMesh().getIndicesVertexes().resize(n_faces);
    m_trimesh_shape->GetMesh().getIndicesNormals().resize(n_faces);

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Readibility aliases
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh().getCoordsVertices();
    std::vector<ChVector<> >& normals = m_trimesh_shape->GetMesh().getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = m_trimesh_shape->GetMesh().getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = m_trimesh_shape->GetMesh().getIndicesNormals();

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
            m_trimesh_shape->GetMesh().getCoordsColors()[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            m_trimesh_shape->GetMesh().getCoordsUV()[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
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
        ChVector<> nrm = Vcross(vertices[idx_vertices[it].y] - vertices[idx_vertices[it].x],
                                vertices[idx_vertices[it].z] - vertices[idx_vertices[it].x]);
        nrm.Normalize();
        // Increment the normals of all incident vertices by the face normal
        normals[idx_normals[it].x] += nrm;
        normals[idx_normals[it].y] += nrm;
        normals[idx_normals[it].z] += nrm;
        // Increment the count of all incident vertices by 1
        accumulators[idx_normals[it].x] += 1;
        accumulators[idx_normals[it].y] += 1;
        accumulators[idx_normals[it].z] += 1;
    }

    // Set the normals to the average values.
    for (unsigned int in = 0; in < n_verts; ++in) {
        normals[in] /= (double)accumulators[in];
    }
}

/*
// -----------------------------------------------------------------------------
// Return the terrain height at the specified location
// -----------------------------------------------------------------------------
double DeformableTerrain::GetHeight(double x, double y) const {
    return 0; 
    //***TODO***
}

// -----------------------------------------------------------------------------
// Return the terrain normal at the specified location
// -----------------------------------------------------------------------------
ChVector<> DeformableTerrain::GetNormal(double x, double y) const {
    return VECT_Y; 
    //***TODO***
}
*/


// Updates the forces and the geometry
void DeformableTerrain::Update(double mytime, bool update_assets) {

    // Readibility aliases
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh().getCoordsVertices();

    collision::ChCollisionSystem::ChRayhitResult mrayhit_result;

    for (int i=0; i< vertices.size(); ++i) {
        ChVector<> from = vertices[i];
        ChVector<> to   = vertices[i];
        ChVector<> D    = ChVector<>(0,1,0);
        from = to - D*0.5;

        this->GetSystem()->GetCollisionSystem()->RayHit(from,to,mrayhit_result);
        if (mrayhit_result.hit == true) {
            vertices[i] = mrayhit_result.abs_hitPoint;
        }
    }

    // parent inheritance
    ChPhysicsItem::Update(mytime, update_assets);
}



}  // end namespace vehicle
}  // end namespace chrono
