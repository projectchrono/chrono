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
    m_color->SetColor(ChColor(0.3, 0.3, 0.3));
    this->AddAsset(m_color);

    // Create the default triangle mesh asset
    m_trimesh_shape = ChSharedPtr<ChTriangleMeshShape>(new ChTriangleMeshShape);
    this->AddAsset(m_trimesh_shape);
    m_trimesh_shape->SetWireframe(false);


    Bekker_Kphi = 2e6;
    Bekker_Kc = 0;
    Bekker_n = 1.1;
    Mohr_cohesion = 50;
    Mohr_friction = 20;
    Janosi_shear = 0.01;

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
    //colors.resize(n_verts);
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
            //colors[iv] = ChVector<float>(1, 1, 1);
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

    // Reset and initialize computation data:
    //
    p_vertices_initial= vertices;
    p_speeds.resize( vertices.size());
    p_step_sinkage.resize( vertices.size());
    p_sinkage.resize( vertices.size());
    p_kshear.resize( vertices.size());
    p_area.resize( vertices.size());
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

    // Reset and initialize computation data:
    //
    p_vertices_initial= vertices;
    p_speeds.resize( vertices.size());
    p_step_sinkage.resize( vertices.size());
    p_sinkage.resize( vertices.size());
    p_kshear.resize( vertices.size());
    p_area.resize( vertices.size());
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



void DeformableTerrain::UpdateInternalForces() {
    
    // Readibility aliases
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh().getCoordsVertices();
    std::vector<ChVector<> >& normals = m_trimesh_shape->GetMesh().getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = m_trimesh_shape->GetMesh().getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = m_trimesh_shape->GetMesh().getIndicesNormals();

    // 
    // Reset the load list
    //

    this->GetLoadList().clear();

    //
    // Compute (pseudo)areas per node
    //
    
    // for a X-Z rectangular grid-like mesh it is simply area[i]= xsize/xsteps * zsize/zsteps, 
    // but the following is more general, also for generic meshes:
    for (unsigned int iv = 0; iv < vertices.size(); ++iv) {
        p_area[iv] = 0;
    }
    for (unsigned int it = 0; it < idx_vertices.size(); ++it) {
        ChVector<> AB = vertices[idx_vertices[it].y] - vertices[idx_vertices[it].x];
        ChVector<> AC = vertices[idx_vertices[it].z] - vertices[idx_vertices[it].x];
        AB.y=0;
        AC.y=0;
        double triangle_area = 0.5*(Vcross(AB,AC)).Length();
        p_area[idx_normals[it].x] += triangle_area /3.0;
        p_area[idx_normals[it].y] += triangle_area /3.0;
        p_area[idx_normals[it].z] += triangle_area /3.0;
    }


    //
    // Perform ray-hit test to detect the contact point sinkage
    // 
    
    collision::ChCollisionSystem::ChRayhitResult mrayhit_result;

    for (int i=0; i< vertices.size(); ++i) {
        ChVector<> from = vertices[i];
        ChVector<> to   = vertices[i];
        ChVector<> N    = ChVector<>(0,1,0);
        from = to - N*0.5;

        p_step_sinkage[i]=0;

        this->GetSystem()->GetCollisionSystem()->RayHit(from,to,mrayhit_result);
        if (mrayhit_result.hit == true) {
            p_step_sinkage[i] = Vdot(( mrayhit_result.abs_hitPoint - vertices[i] ), N);
            vertices[i] = mrayhit_result.abs_hitPoint;
            p_sinkage[i]=  Vdot(( vertices[i] - p_vertices_initial[i] ), N);

            if (ChContactable* contactable = dynamic_cast<ChContactable*>(mrayhit_result.hitModel->GetPhysicsItem())) {
                p_speeds[i] = contactable->GetContactPointSpeed(vertices[i]);
            }
            
            ChVector<> T = -p_speeds[i];
            T.y=0;
            T.Normalize();

            // accumulate shear for Janosi-Hanamoto
            p_kshear[i] += Vdot(p_speeds[i],-T) * this->GetSystem()->GetStep();

            // Compute i-th force:
            ChVector<> Fn;
            ChVector<> Ft;
            // Bekker formula, neglecting Bekker_Kc and 'b'
            double sigma = this->Bekker_Kphi * pow(-p_sinkage[i], this->Bekker_n ); 
            // Mohr-Coulomb
            double tau_max = this->Mohr_cohesion + sigma * tan(this->Mohr_friction*CH_C_DEG_TO_RAD);
            // Janosi-Hanamoto
            double tau = tau_max * (1.0 - exp(- (p_kshear[i]/this->Janosi_shear)));
            
            Fn = N * p_area[i] * sigma;
            Ft = T * p_area[i] * tau;

            if (ChBody* rigidbody = dynamic_cast<ChBody*>(mrayhit_result.hitModel->GetPhysicsItem())) {
                // Trick, since 'rigidbody' was not a new ChBody() object, but an already used pointer because
                // mrayhit_result.hitModel->GetPhysicsItem() cannot return it as shared ptr, just raw ptr..
                ChSharedPtr<ChBody> srigidbody(rigidbody); 
                srigidbody->AddRef(); // avoid deleting when out of the if() scope
                ChSharedPtr<ChLoadBodyForce> mload(new ChLoadBodyForce(srigidbody, Fn+Ft, false, vertices[i], false));
                this->Add(mload);
            }
        }
    }

    //
    // Update the normals
    // 

    std::vector<int> accumulators(vertices.size(), 0);

    // Calculate normals and then average the normals from all adjacent faces.
    for (unsigned int it = 0; it < idx_vertices.size(); ++it) {
        // Calculate the triangle normal as a normalized cross product.
        ChVector<> nrm = -Vcross(vertices[idx_vertices[it].y] - vertices[idx_vertices[it].x],
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
    for (unsigned int in = 0; in < vertices.size(); ++in) {
        normals[in] /= (double)accumulators[in];
    }


    // 
    // Compute the forces 
    //
    

    // Use the SCM soil contact model as described in the paper:
    // "Parameter Identification of a Planetary Rover Wheel–Soil
    // Contact Model via a Bayesian Approach", A.Gallina, R. Krenn et al.


    // 
    // Update visual asset
    //

    

    // Not needed because Update() will happen anyway
    //  ChPhysicsItem::Update(0, true);
}



}  // end namespace vehicle
}  // end namespace chrono
