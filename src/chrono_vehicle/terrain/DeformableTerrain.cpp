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
// Implementation of the DeformableTerrain wrapper class
// -----------------------------------------------------------------------------
DeformableTerrain::DeformableTerrain(ChSystem* system) {
    m_ground = std::make_shared<DeformableSoil>(system);
    system->Add(m_ground);
}
    
// Return the terrain height at the specified location
double DeformableTerrain::GetHeight(double x, double y) const {
    //// TODO
    return 0;
}

// Return the terrain normal at the specified location
ChVector<> DeformableTerrain::GetNormal(double x, double y) const {
    //// TODO
    return m_ground->plane.TransformDirectionLocalToParent(ChVector<>(0, 1, 0));
}

// Set the color of the visualization assets
void DeformableTerrain::SetColor(ChColor color) {
    m_ground->m_color->SetColor(color);
}

// Set the texture and texture scaling
void DeformableTerrain::SetTexture(const std::string tex_file, float tex_scale_x, float tex_scale_y) {
    std::shared_ptr<ChTexture> texture(new ChTexture);
    texture->SetTextureFilename(tex_file);
    texture->SetTextureScale(tex_scale_x, tex_scale_y);
    m_ground->AddAsset(texture);
}

// Set the plane reference.
void DeformableTerrain::SetPlane(ChCoordsys<> mplane) { m_ground->plane = mplane; }

// Get the plane reference.
const ChCoordsys<>& DeformableTerrain::GetPlane() const { return m_ground->plane; }

// Enable bulldozing effect.
void DeformableTerrain::SetBulldozingFlow(bool mb) {
    m_ground->do_bulldozing = mb;
}

bool DeformableTerrain::GetBulldozingFlow() const {
    return m_ground->do_bulldozing;
}

// Set properties of the SCM soil model
void DeformableTerrain::SetSoilParametersSCM(
    double mBekker_Kphi,    // Kphi, frictional modulus in Bekker model
    double mBekker_Kc,      // Kc, cohesive modulus in Bekker model
    double mBekker_n,       // n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double mMohr_cohesion,  // Cohesion in, Pa, for shear failure
    double mMohr_friction,  // Friction angle (in degrees!), for shear failure
    double mJanosi_shear,   // J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
    double melastic_K       // elastic stiffness K (must be > Kphi; very high values gives the original SCM model)
    ) {
    m_ground->Bekker_Kphi = mBekker_Kphi;
    m_ground->Bekker_Kc = mBekker_Kc;
    m_ground->Bekker_n = mBekker_n;
    m_ground->Mohr_cohesion = mMohr_cohesion;
    m_ground->Mohr_friction = mMohr_friction;
    m_ground->Janosi_shear = mJanosi_shear;
    m_ground->elastic_K = ChMax(melastic_K, mBekker_Kphi);
}

void DeformableTerrain::SetBulldozingParameters(double mbulldozing_erosion_angle,     ///< angle of erosion of the displaced material (in degrees!)
                                 double mbulldozing_flow_factor   ///< growth of lateral volume respect to pressed volume
                                 ) {
    m_ground->bulldozing_erosion_angle = mbulldozing_erosion_angle;
    m_ground->bulldozing_flow_factor = mbulldozing_flow_factor;
}


// Set the color plot type.
void DeformableTerrain::SetPlotType(DataPlotType mplot, double mmin, double mmax) {
    m_ground->plot_type = mplot;
    m_ground->plot_v_min = mmin;
    m_ground->plot_v_max = mmax;
}

// Initialize the terrain as a flat grid
void DeformableTerrain::Initialize(double height, double sizeX, double sizeY, int divX, int divY) {
    m_ground->Initialize(height, sizeX, sizeY, divX, divY);
}

// Initialize the terrain from a specified .obj mesh file.
void DeformableTerrain::Initialize(const std::string& mesh_file) {
    m_ground->Initialize(mesh_file);
}

// Initialize the terrain from a specified height map.
void DeformableTerrain::Initialize(const std::string& heightmap_file,
                                   const std::string& mesh_name,
                                   double sizeX,
                                   double sizeY,
                                   double hMin,
                                   double hMax) {
    m_ground->Initialize(heightmap_file, mesh_name, sizeX, sizeY, hMin, hMax);
}

// -----------------------------------------------------------------------------
// Implementation of DeformableSoil
// -----------------------------------------------------------------------------

// Constructor.
DeformableSoil::DeformableSoil(ChSystem* system) {
    this->SetSystem(system);

    // Create the default mesh asset
    m_color = std::shared_ptr<ChColorAsset>(new ChColorAsset);
    m_color->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    this->AddAsset(m_color);

    // Create the default triangle mesh asset
    m_trimesh_shape = std::shared_ptr<ChTriangleMeshShape>(new ChTriangleMeshShape);
    this->AddAsset(m_trimesh_shape);
    m_trimesh_shape->SetWireframe(false);

    do_bulldozing = false;
    bulldozing_flow_factor = 1.2;
    bulldozing_erosion_angle = 40;

    Bekker_Kphi = 2e6;
    Bekker_Kc = 0;
    Bekker_n = 1.1;
    Mohr_cohesion = 50;
    Mohr_friction = 20;
    Janosi_shear = 0.01;
    elastic_K = 50000000;

    Initialize(0,3,3,10,10);
    
    plot_type = DeformableTerrain::PLOT_NONE;
    plot_v_min = 0;
    plot_v_max = 0.2;
}

// Initialize the terrain as a flat grid
void DeformableSoil::Initialize(double height, double sizeX, double sizeY, int nX, int nY) {
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
        for (unsigned int ix = 0; ix < nvx; ++ix) {
            double x = ix * dx - 0.5 * sizeX;
            // Set vertex location
            vertices[iv] = plane * ChVector<>(x, height, y);
            // Initialize vertex normal to Y up
            normals[iv] = plane.TransformDirectionLocalToParent(ChVector<>(0, 1, 0));
            // Assign color white to all vertices
            //colors[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            uv_coords[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
            ++iv;
        }
    }

    unsigned int it = 0;
    for (int iy = nvy - 2; iy >= 0; --iy) {
        for (unsigned int ix = 0; ix < nvx - 1; ++ix) {
            int v0 = ix + nvx * iy;
            idx_vertices[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            idx_normals[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            ++it;
        }
    }

    // Needed! precomputes aux.topology 
    // data structures for the mesh, aux. material data, etc.
    SetupAuxData();
}

// Initialize the terrain from a specified .obj mesh file.
void DeformableSoil::Initialize(const std::string& mesh_file) {
    m_trimesh_shape->GetMesh().Clear();
    m_trimesh_shape->GetMesh().LoadWavefrontMesh(mesh_file, true, true);
}

// Initialize the terrain from a specified height map.
void DeformableSoil::Initialize(const std::string& heightmap_file,
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
            vertices[iv] = plane * ChVector<>(x, z, y);
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

    // Needed! precomputes aux.topology 
    // data structures for the mesh, aux. material data, etc.
    SetupAuxData();
}

// Set up auxiliary data structures.
void DeformableSoil::SetupAuxData() {
    // better readability:
    std::vector<ChVector<int> >& idx_vertices = m_trimesh_shape->GetMesh().getIndicesVertexes();
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh().getCoordsVertices();

    // Reset and initialize computation data:
    //
    p_vertices_initial= vertices;
    p_speeds.resize( vertices.size());
    p_step_plastic_flow.resize( vertices.size());
    p_sinkage.resize( vertices.size());
    p_sinkage_plastic.resize( vertices.size());
    p_sinkage_elastic.resize( vertices.size());
    p_kshear.resize( vertices.size());
    p_area.resize( vertices.size());
    p_sigma.resize( vertices.size());
    p_sigma_yeld.resize( vertices.size());
    p_tau.resize( vertices.size());  
    p_id_island.resize (vertices.size());
    p_erosion.resize(vertices.size());

    connected_vertexes.resize( vertices.size() );
    for (unsigned int iface = 0; iface < idx_vertices.size(); ++iface) {
        connected_vertexes[idx_vertices[iface].x].insert(idx_vertices[iface].y);
        connected_vertexes[idx_vertices[iface].x].insert(idx_vertices[iface].z);
        connected_vertexes[idx_vertices[iface].y].insert(idx_vertices[iface].x);
        connected_vertexes[idx_vertices[iface].y].insert(idx_vertices[iface].z);
        connected_vertexes[idx_vertices[iface].z].insert(idx_vertices[iface].x);
        connected_vertexes[idx_vertices[iface].z].insert(idx_vertices[iface].y);
    }
}

// Reset the list of forces, and fills it with forces from a soil contact model.
void DeformableSoil::UpdateInternalForces() {
    // Readibility aliases
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh().getCoordsVertices();
    std::vector<ChVector<> >& normals = m_trimesh_shape->GetMesh().getCoordsNormals();
    std::vector<ChVector<float> >& colors =  m_trimesh_shape->GetMesh().getCoordsColors();
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
        AB = plane.TransformDirectionParentToLocal(AB);
        AC = plane.TransformDirectionParentToLocal(AC);
        AB.y=0;
        AC.y=0;
        double triangle_area = 0.5*(Vcross(AB,AC)).Length();
        p_area[idx_normals[it].x] += triangle_area /3.0;
        p_area[idx_normals[it].y] += triangle_area /3.0;
        p_area[idx_normals[it].z] += triangle_area /3.0;
    }

    ChVector<> N    = plane.TransformDirectionLocalToParent(ChVector<>(0,1,0));

    //
    // Perform ray-hit test to detect the contact point sinkage
    // 
    
    collision::ChCollisionSystem::ChRayhitResult mrayhit_result;

    for (int i=0; i< vertices.size(); ++i) {
        p_sigma[i] = 0;
        p_sinkage_elastic[i] = 0;
        p_step_plastic_flow[i]=0;
        p_erosion[i] = false;

        ChVector<> to   = vertices[i] +N*0.01; 
        ChVector<> from = to - N*0.5;

        this->GetSystem()->GetCollisionSystem()->RayHit(from,to,mrayhit_result);
        if (mrayhit_result.hit == true) {
            double test_sinkage = - Vdot(( mrayhit_result.abs_hitPoint - p_vertices_initial[i] ), N);

            if (ChContactable* contactable = dynamic_cast<ChContactable*>(mrayhit_result.hitModel->GetPhysicsItem())) {
                p_speeds[i] = contactable->GetContactPointSpeed(vertices[i]);
            }
            
            ChVector<> T = -p_speeds[i];
            T = plane.TransformDirectionParentToLocal(T);
            T.y=0;
            T = plane.TransformDirectionLocalToParent(T);
            T.Normalize();   

            // Compute i-th force:
            ChVector<> Fn;
            ChVector<> Ft;

            // Elastic try:
            p_sigma[i] = elastic_K * (test_sinkage - p_sinkage_plastic[i]);

            // Handle unilaterality:
            if (p_sigma[i] <0) {
                p_sigma[i] =0;
            } else {
                p_sinkage[i] = test_sinkage;

                // Accumulate shear for Janosi-Hanamoto
                p_kshear[i] += Vdot(p_speeds[i],-T) * this->GetSystem()->GetStep();

                // Plastic correction:
                if (p_sigma[i] > p_sigma_yeld[i]) {
                    // Bekker formula, neglecting Bekker_Kc and 'b'
                    p_sigma[i] = this->Bekker_Kphi * pow(p_sinkage[i], this->Bekker_n );
                    p_sigma_yeld[i]= p_sigma[i];
                    double old_sinkage_plastic = p_sinkage_plastic[i];
                    p_sinkage_plastic[i] = p_sinkage[i] - p_sigma[i]/elastic_K;
                    p_step_plastic_flow[i] =
                        (p_sinkage_plastic[i] - old_sinkage_plastic) / this->GetSystem()->GetStep();
                }

                p_sinkage_elastic[i] = p_sinkage[i] - p_sinkage_plastic[i];

                // Mohr-Coulomb
                double tau_max = this->Mohr_cohesion + p_sigma[i] * tan(this->Mohr_friction*CH_C_DEG_TO_RAD);

                // Janosi-Hanamoto
                p_tau[i] = tau_max * (1.0 - exp(- (p_kshear[i]/this->Janosi_shear)));
            
                Fn = N * p_area[i] * p_sigma[i];
                Ft = T * p_area[i] * p_tau[i];

                if (ChBody* rigidbody = dynamic_cast<ChBody*>(mrayhit_result.hitModel->GetPhysicsItem())) {
                    // [](){} Trick: no deletion for this shared ptr, since 'rigidbody' was not a new ChBody() 
                    // object, but an already used pointer because mrayhit_result.hitModel->GetPhysicsItem() 
                    // cannot return it as shared_ptr, as needed by the ChLoadBodyForce:
                    std::shared_ptr<ChBody> srigidbody(rigidbody, [](ChBody*){}); 
                    std::shared_ptr<ChLoadBodyForce> mload(
                        new ChLoadBodyForce(srigidbody, Fn + Ft, false, vertices[i], false));
                    this->Add(mload);
                }

                // Update mesh representation
                vertices[i] = p_vertices_initial[i] - N * p_sinkage[i];

            } // end positive contact force

        } // end successfull hit test

    } // end loop on vertexes



    //
    // Flow material to the side of rut, using heuristics
    // 

    if (do_bulldozing) {
        std::set<int> touched_vertexes;
        for (int iv = 0; iv< vertices.size(); ++iv) {
            p_id_island[iv] = 0;
            if (p_sigma[iv]>0)
                touched_vertexes.insert(iv);
        }

        std::set<int> domain_boundaries;

        // Compute contact islands (and their displaced material) by flood-filling the mesh
        int id_island = 0;
        for (auto fillseed = touched_vertexes.begin(); fillseed != touched_vertexes.end(); fillseed = touched_vertexes.begin()) {
            // new island:
            ++id_island;
            std::set<int> fill_front;

            std::set<int> boundary;
            int n_vert_boundary = 0;
            double tot_area_boundary = 0;

            int n_vert_island = 1;
            double tot_step_flow_island = p_area[*fillseed] * p_step_plastic_flow[*fillseed] * this->GetSystem()->GetStep();
            double tot_Nforce_island = p_area[*fillseed] * p_sigma[*fillseed];
            fill_front.insert(*fillseed);
            p_id_island[*fillseed] = id_island;
            touched_vertexes.erase(fillseed);
            while (fill_front.size() >0) {
                // fill next front
                std::set<int> fill_front_2;
                for (auto ifront : fill_front) {
                    for (auto ivconnect : connected_vertexes[ifront]) {
                        if ((p_sigma[ivconnect]>0) && (p_id_island[ivconnect]==0)) {
                            ++n_vert_island;
                            tot_step_flow_island += p_area[ivconnect] * p_step_plastic_flow[ivconnect] * this->GetSystem()->GetStep();
                            tot_Nforce_island += p_area[ivconnect] * p_sigma[ivconnect];
                            fill_front_2.insert(ivconnect);
                            p_id_island[ivconnect] = id_island;
                            touched_vertexes.erase(ivconnect);
                        } 
                        else if ((p_sigma[ivconnect] == 0) && (p_id_island[ivconnect] <= 0) && (p_id_island[ivconnect] != -id_island)) {
                            ++n_vert_boundary;
                            tot_area_boundary += p_area[ivconnect];
                            p_id_island[ivconnect] = -id_island; // negative to mark as boundary
                            boundary.insert(ivconnect);
                        }
                    }
                }
                // advance to next front
                fill_front= fill_front_2;
            }
            //GetLog() << " island " << id_island << " flow volume =" << tot_step_flow_island << " N force=" << tot_Nforce_island << "\n"; 

            // Raise the boundary because of material flow (it gives a sharp spike around the
            // island boundary, but later we'll use the erosion algorithm to smooth it out)
            double tot_length_boundary = (double)boundary.size() * sqrt(0.5*tot_area_boundary / (double)boundary.size() ); // approx.
            double tot_width_boundary = tot_area_boundary/tot_length_boundary;
            
            for (auto ibv : boundary) {
                double raise_y = bulldozing_flow_factor * ((p_area[ibv]/tot_area_boundary) *  (1/p_area[ibv]) * tot_step_flow_island);
                vertices[ibv]           += N * raise_y;
                p_vertices_initial[ibv] += N * raise_y;
            }

            domain_boundaries.insert(boundary.begin(), boundary.end());

        }// end for islands


        // Erosion domain area select, by topologically dilation of all the 
        // boundaries of the islands:
        std::set<int> domain_erosion= domain_boundaries;
        for (auto ie : domain_boundaries)
            p_erosion[ie] = true;
        std::set<int> front_erosion = domain_boundaries;
        for (int iloop = 0; iloop <10; ++iloop) {
            std::set<int> front_erosion2;
            for(auto is : front_erosion) {
                for (auto ivconnect : connected_vertexes[is]) {
                    if ((p_id_island[ivconnect]==0) && (p_erosion[ivconnect]==0)) {
                        front_erosion2.insert(ivconnect);
                        p_erosion[ivconnect] = true;
                    }
                }
            }
            domain_erosion.insert(front_erosion2.begin(), front_erosion2.end());
            front_erosion = front_erosion2;
        }
        // Erosion smoothing algorithm on domain
        for (int ismo = 0; ismo <3; ++ismo) {
            for (auto is : domain_erosion) {
                double my = vertices[is].y;
                for (auto ivc : connected_vertexes[is]) {
                    ChVector<> vis = this->plane.TransformParentToLocal(vertices[is]);
                    if (p_sigma[ivc] == 0) {
                        ChVector<> vic = this->plane.TransformParentToLocal(vertices[ivc]);
                        ChVector<> vdist = vic-vis;
                        vdist.y=0;
                        double ddist = vdist.Length();
                        double dy = my - vertices[ivc].y;
                        double dy_lim = ddist * tan(bulldozing_erosion_angle*CH_C_DEG_TO_RAD);
                        if (dy>dy_lim) {
                            ChVector<> DV = ((dy-dy_lim)*0.5/(double)connected_vertexes[is].size()) * this->plane.TransformDirectionLocalToParent(VECT_Y);
                            vertices[is]  -= DV;
                            vertices[ivc] += DV;
                        }
                    }
                }
            }
        }

    } // end bulldozing flow 



    //
    // Update the visualization colors
    // 
    if (plot_type != DeformableTerrain::PLOT_NONE) {
        colors.resize(vertices.size());
        for (size_t iv = 0; iv< vertices.size(); ++iv) {
            ChColor mcolor;
            switch (plot_type) {
                case DeformableTerrain::PLOT_SINKAGE:
                    mcolor = ChColor::ComputeFalseColor(p_sinkage[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_SINKAGE_ELASTIC:
                    mcolor = ChColor::ComputeFalseColor(p_sinkage_elastic[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_SINKAGE_PLASTIC:
                    mcolor = ChColor::ComputeFalseColor(p_sinkage_plastic[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_STEP_PLASTIC_FLOW:
                    mcolor = ChColor::ComputeFalseColor(p_step_plastic_flow[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_K_JANOSI:
                    mcolor = ChColor::ComputeFalseColor(p_kshear[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_PRESSURE:
                    mcolor = ChColor::ComputeFalseColor(p_sigma[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_PRESSURE_YELD:
                    mcolor = ChColor::ComputeFalseColor(p_sigma_yeld[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_SHEAR:
                    mcolor = ChColor::ComputeFalseColor(p_tau[iv], plot_v_min, plot_v_max);
                    break;
                case DeformableTerrain::PLOT_ISLAND_ID:
                    mcolor = ChColor(0,0,1);
                    if (p_erosion[iv] == true)
                        mcolor = ChColor(1,1,1);
                    if (p_id_island[iv] >0)
                        mcolor = ChColor::ComputeFalseColor(4 +(p_id_island[iv] % 8), 0, 12);
                    if (p_id_island[iv] <0)
                        mcolor = ChColor(0,0,0);
                    break;
                case DeformableTerrain::PLOT_IS_TOUCHED:
                    if (p_sigma[iv]>0)
                        mcolor = ChColor(1,0,0);
                    else 
                        mcolor = ChColor(0,0,1);
                    break;
            }
            colors[iv] = {mcolor.R, mcolor.G, mcolor.B};
        }
    } else {
        colors.clear();
    }

    //
    // Update the visualization normals
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
