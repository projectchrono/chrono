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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Deformable terrain based on SCM (Soil Contact Model) from DLR
// (Krenn & Hirzinger)
//
// =============================================================================

#include <cstdio>
#include <cmath>
#include <queue>

#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/utils/ChConvexHull.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/Easy_BMP/EasyBMP.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the SCMDeformableTerrain wrapper class
// -----------------------------------------------------------------------------
SCMDeformableTerrain::SCMDeformableTerrain(ChSystem* system) {
    m_ground = std::make_shared<SCMDeformableSoil>(system);
    system->Add(m_ground);
}
    
// Return the terrain height at the specified location
double SCMDeformableTerrain::GetHeight(double x, double y) const {
    //// TODO
    return 0;
}

// Return the terrain normal at the specified location
ChVector<> SCMDeformableTerrain::GetNormal(double x, double y) const {
    //// TODO
    return m_ground->plane.TransformDirectionLocalToParent(ChVector<>(0, 1, 0));
}

// Return the terrain coefficient of friction at the specified location
float SCMDeformableTerrain::GetCoefficientFriction(double x, double y) const {
    return m_friction_fun ? (*m_friction_fun)(x, y) : 0.8f;
}

// Set the color of the visualization assets
void SCMDeformableTerrain::SetColor(ChColor color) {
    m_ground->m_color->SetColor(color);
}

// Set the texture and texture scaling
void SCMDeformableTerrain::SetTexture(const std::string tex_file, float tex_scale_x, float tex_scale_y) {
    std::shared_ptr<ChTexture> texture(new ChTexture);
    texture->SetTextureFilename(tex_file);
    texture->SetTextureScale(tex_scale_x, tex_scale_y);
    m_ground->AddAsset(texture);
}

// Set the plane reference.
void SCMDeformableTerrain::SetPlane(ChCoordsys<> mplane) { m_ground->plane = mplane; }

// Get the plane reference.
const ChCoordsys<>& SCMDeformableTerrain::GetPlane() const { return m_ground->plane; }

// Get the trimesh that defines the ground shape.
const std::shared_ptr<ChTriangleMeshShape> SCMDeformableTerrain::GetMesh() const { return m_ground->m_trimesh_shape; }

// Enable bulldozing effect.
void SCMDeformableTerrain::SetBulldozingFlow(bool mb) {
    m_ground->do_bulldozing = mb;
}

bool SCMDeformableTerrain::GetBulldozingFlow() const {
    return m_ground->do_bulldozing;
}

// Set properties of the SCM soil model
void SCMDeformableTerrain::SetSoilParametersSCM(
    double mBekker_Kphi,    // Kphi, frictional modulus in Bekker model
    double mBekker_Kc,      // Kc, cohesive modulus in Bekker model
    double mBekker_n,       // n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double mMohr_cohesion,  // Cohesion in, Pa, for shear failure
    double mMohr_friction,  // Friction angle (in degrees!), for shear failure
    double mJanosi_shear,   // J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
    double melastic_K,      // elastic stiffness K (must be > Kphi; very high values gives the original SCM model)
    double mdamping_R       // vertical damping R, per unit area (vertical speed proportional, it is zero in original SCM model)
    ) {
    m_ground->Bekker_Kphi = mBekker_Kphi;
    m_ground->Bekker_Kc = mBekker_Kc;
    m_ground->Bekker_n = mBekker_n;
    m_ground->Mohr_cohesion = mMohr_cohesion;
    m_ground->Mohr_friction = mMohr_friction;
    m_ground->Janosi_shear = mJanosi_shear;
    m_ground->elastic_K = ChMax(melastic_K, mBekker_Kphi);
    m_ground->damping_R = mdamping_R;
}

void SCMDeformableTerrain::SetBulldozingParameters(double mbulldozing_erosion_angle,     ///< angle of erosion of the displaced material (in degrees!)
                                 double mbulldozing_flow_factor,  ///< growth of lateral volume respect to pressed volume
                                 int mbulldozing_erosion_n_iterations, ///< number of erosion refinements per timestep 
                                 int mbulldozing_erosion_n_propagations ///< number of concentric vertex selections subject to erosion 
                                 ) {
    m_ground->bulldozing_erosion_angle = mbulldozing_erosion_angle;
    m_ground->bulldozing_flow_factor = mbulldozing_flow_factor;
    m_ground->bulldozing_erosion_n_iterations = mbulldozing_erosion_n_iterations;
    m_ground->bulldozing_erosion_n_propagations = mbulldozing_erosion_n_propagations;
}

void SCMDeformableTerrain::SetAutomaticRefinement(bool mr) { 
    m_ground->do_refinement = mr;
}

bool SCMDeformableTerrain::GetAutomaticRefinement() const {
    return m_ground->do_refinement;
}

void SCMDeformableTerrain::SetAutomaticRefinementResolution(double mr) {
    m_ground->refinement_resolution = mr;
}

double SCMDeformableTerrain::GetAutomaticRefinementResolution() const {
    return m_ground->refinement_resolution;
}

void SCMDeformableTerrain::SetTestHighOffset(double mr) {
    m_ground->test_high_offset = mr;
}

double SCMDeformableTerrain::GetTestHighOffset() const {
    return m_ground->test_high_offset;
}

// Set the color plot type.
void SCMDeformableTerrain::SetPlotType(DataPlotType mplot, double mmin, double mmax) {
    m_ground->plot_type = mplot;
    m_ground->plot_v_min = mmin;
    m_ground->plot_v_max = mmax;
}

// Enable moving patch
void SCMDeformableTerrain::EnableMovingPatch(std::shared_ptr<ChBody> body,
                                             const ChVector<>& point_on_body,
                                             double dimX,
                                             double dimY) {
    m_ground->m_body = body;
    m_ground->m_body_point = point_on_body;
    m_ground->m_patch_dim = ChVector2<>(dimX, dimY);
    m_ground->m_moving_patch = true;
}

// Initialize the terrain as a flat grid
void SCMDeformableTerrain::Initialize(double height, double sizeX, double sizeY, int divX, int divY) {
    m_ground->Initialize(height, sizeX, sizeY, divX, divY);
}

// Initialize the terrain from a specified .obj mesh file.
void SCMDeformableTerrain::Initialize(const std::string& mesh_file) {
    m_ground->Initialize(mesh_file);
}

// Initialize the terrain from a specified height map.
void SCMDeformableTerrain::Initialize(const std::string& heightmap_file,
                                   const std::string& mesh_name,
                                   double sizeX,
                                   double sizeY,
                                   double hMin,
                                   double hMax) {
    m_ground->Initialize(heightmap_file, mesh_name, sizeX, sizeY, hMin, hMax);
}

TerrainForce SCMDeformableTerrain::GetContactForce(std::shared_ptr<ChBody> body) const {
    auto itr = m_ground->m_contact_forces.find(body.get());
    if (itr != m_ground->m_contact_forces.end())
        return itr->second;

    TerrainForce frc;
    frc.point = body->GetPos();
    frc.force = ChVector<>(0, 0, 0);
    frc.moment = ChVector<>(0, 0, 0);
    return frc;
}

void SCMDeformableTerrain::PrintStepStatistics(std::ostream& os) const {
    os << " Timers:" << std::endl;
    os << "   Calculate areas:         " << m_ground->m_timer_calc_areas() << std::endl;
    os << "   Ray casting:             " << m_ground->m_timer_ray_casting() << std::endl;
    if (m_ground->do_refinement)
        os << "   Refinements:             " << m_ground->m_timer_refinement() << std::endl;
    if (m_ground->do_bulldozing)
        os << "   Bulldozing:              " << m_ground->m_timer_bulldozing() << std::endl;
    os << "   Visualization:           " << m_ground->m_timer_visualization() << std::endl;

    os << " Counters:" << std::endl;
    os << "   Number vertices:         " << m_ground->m_num_vertices << std::endl;
    os << "   Number ray-casts:        " << m_ground->m_num_ray_casts << std::endl;
    os << "   Number faces:            " << m_ground->m_num_faces << std::endl;
    if (m_ground->do_refinement)
        os << "   Number faces refinement: " << m_ground->m_num_marked_faces << std::endl;
}

// -----------------------------------------------------------------------------
// Implementation of SCMDeformableSoil
// -----------------------------------------------------------------------------

// Constructor.
SCMDeformableSoil::SCMDeformableSoil(ChSystem* system) {
    this->SetSystem(system);

    // Create the default mesh asset
    m_color = std::shared_ptr<ChColorAsset>(new ChColorAsset);
    m_color->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    this->AddAsset(m_color);

    // Create the default triangle mesh asset
    m_trimesh_shape = std::shared_ptr<ChTriangleMeshShape>(new ChTriangleMeshShape);
    this->AddAsset(m_trimesh_shape);
    m_trimesh_shape->SetWireframe(true);

    do_bulldozing = false;
    bulldozing_flow_factor = 1.2;
    bulldozing_erosion_angle = 40;
    bulldozing_erosion_n_iterations = 3;
    bulldozing_erosion_n_propagations = 10;

    do_refinement = false;
    refinement_resolution = 0.01;

    Bekker_Kphi = 2e6;
    Bekker_Kc = 0;
    Bekker_n = 1.1;
    Mohr_cohesion = 50;
    Mohr_friction = 20;
    Janosi_shear = 0.01;
    elastic_K = 50000000;

    Initialize(0,3,3,10,10);
    
    plot_type = SCMDeformableTerrain::PLOT_NONE;
    plot_v_min = 0;
    plot_v_max = 0.2;

    test_high_offset = 0.1;
    test_low_offset = 0.5;

    last_t = 0;

    m_moving_patch = false;
}

// Initialize the terrain as a flat grid
void SCMDeformableSoil::Initialize(double height, double sizeX, double sizeY, int nX, int nY) {
    m_trimesh_shape->GetMesh()->Clear();
    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    std::vector<ChVector<> >& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<> >& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = trimesh->getIndicesNormals();
    std::vector<ChVector<> >& uv_coords = trimesh->getCoordsUV();
    std::vector<ChVector<float> >& colors = trimesh->getCoordsColors();

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

    // Precompute aux. topology data structures for the mesh, aux. material data, etc.
    SetupAuxData();
}

// Initialize the terrain from a specified .obj mesh file.
void SCMDeformableSoil::Initialize(const std::string& mesh_file) {
    m_trimesh_shape->GetMesh()->Clear();
    m_trimesh_shape->GetMesh()->LoadWavefrontMesh(mesh_file, true, true);

    // Precompute aux. topology data structures for the mesh, aux. material data, etc.
    SetupAuxData();
}

// Initialize the terrain from a specified height map.
void SCMDeformableSoil::Initialize(const std::string& heightmap_file,
                              const std::string& mesh_name,
                              double sizeX,
                              double sizeY,
                              double hMin,
                              double hMax) {
    auto trimesh = m_trimesh_shape->GetMesh();
    trimesh->Clear();

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
    trimesh->getCoordsVertices().resize(n_verts);
    trimesh->getCoordsNormals().resize(n_verts);
    trimesh->getCoordsUV().resize(n_verts);
    trimesh->getCoordsColors().resize(n_verts);

    trimesh->getIndicesVertexes().resize(n_faces);
    trimesh->getIndicesNormals().resize(n_faces);

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Readability aliases
    std::vector<ChVector<> >& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<> >& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int> >& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = trimesh->getIndicesNormals();

    // Load mesh vertices.
    // Note that pixels in a BMP start at top-left corner.
    // We order the vertices starting at the bottom-left corner, row after row.
    // The bottom-left corner corresponds to the point (-sizeX/2, -sizeY/2).
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
            trimesh->getCoordsColors()[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            trimesh->getCoordsUV()[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time).
    // Specify the face vertices counter-clockwise.
    // Set the normal indices same as the vertex indices.
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

    // Precompute aux. topology data structures for the mesh, aux. material data, etc.
    SetupAuxData();
}

// Set up auxiliary data structures.
void SCMDeformableSoil::SetupAuxData() {
    // better readability:
    std::vector<ChVector<int> >& idx_vertices = m_trimesh_shape->GetMesh()->getIndicesVertexes();
    std::vector<ChVector<> >& vertices = m_trimesh_shape->GetMesh()->getCoordsVertices();

    // Reset and initialize computation data:
    //
    p_vertices_initial= vertices;
    p_speeds.resize( vertices.size());
    p_step_plastic_flow.resize( vertices.size());
    p_level.resize( vertices.size());
    p_level_initial.resize( vertices.size());
    p_hit_level.resize( vertices.size());
    p_sinkage.resize( vertices.size());
    p_sinkage_plastic.resize( vertices.size());
    p_sinkage_elastic.resize( vertices.size());
    p_kshear.resize( vertices.size());
    p_area.resize( vertices.size());
    p_sigma.resize( vertices.size());
    p_sigma_yeld.resize( vertices.size());
    p_tau.resize( vertices.size());  
    p_massremainder.resize( vertices.size());  
    p_id_island.resize (vertices.size());
    p_erosion.resize(vertices.size());

    for (int i=0; i< vertices.size(); ++i) {
        p_level[i] = plane.TransformParentToLocal(vertices[i]).y();
        p_level_initial[i] = p_level[i];
    }

    connected_vertexes.resize( vertices.size() );
    for (unsigned int iface = 0; iface < idx_vertices.size(); ++iface) {
        connected_vertexes[idx_vertices[iface][0]].insert(idx_vertices[iface][1]);
        connected_vertexes[idx_vertices[iface][0]].insert(idx_vertices[iface][2]);
        connected_vertexes[idx_vertices[iface][1]].insert(idx_vertices[iface][0]);
        connected_vertexes[idx_vertices[iface][1]].insert(idx_vertices[iface][2]);
        connected_vertexes[idx_vertices[iface][2]].insert(idx_vertices[iface][0]);
        connected_vertexes[idx_vertices[iface][2]].insert(idx_vertices[iface][1]);
    }

    m_trimesh_shape->GetMesh()->ComputeNeighbouringTriangleMap(this->tri_map);
}

// Reset the list of forces, and fills it with forces from a soil contact model.
void SCMDeformableSoil::ComputeInternalForces() {
    m_timer_calc_areas.reset();
    m_timer_ray_casting.reset();
    m_timer_refinement.reset();
    m_timer_bulldozing.reset();
    m_timer_visualization.reset();

    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    std::vector<ChVector<> >& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<> >& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<float> >& colors = trimesh->getCoordsColors();
    std::vector<ChVector<int> >& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int> >& idx_normals = trimesh->getIndicesNormals();
    
    // 
    // Reset the load list and map of contact forces
    //

    this->GetLoadList().clear();
    m_contact_forces.clear();

    //
    // Compute (pseudo)areas per node
    //
    
    m_num_vertices = vertices.size();
    m_num_faces = idx_vertices.size();
    m_timer_calc_areas.start();

    // for a X-Z rectangular grid-like mesh it is simply area[i]= xsize/xsteps * zsize/zsteps, 
    // but the following is more general, also for generic meshes:
    for (unsigned int iv = 0; iv < vertices.size(); ++iv) {
        p_area[iv] = 0;
    }
    for (unsigned int it = 0; it < idx_vertices.size(); ++it) {
        ChVector<> AB = vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]];
        ChVector<> AC = vertices[idx_vertices[it][2]] - vertices[idx_vertices[it][1]];
        AB = plane.TransformDirectionParentToLocal(AB);
        AC = plane.TransformDirectionParentToLocal(AC);
        AB.y()=0;
        AC.y()=0;
        double triangle_area = 0.5*(Vcross(AB,AC)).Length();
        p_area[idx_normals[it][0]] += triangle_area /3.0;
        p_area[idx_normals[it][1]] += triangle_area /3.0;
        p_area[idx_normals[it][2]] += triangle_area /3.0;
    }

    m_timer_calc_areas.stop();

    ChVector<> N = plane.TransformDirectionLocalToParent(ChVector<>(0, 1, 0));

    //
    // Perform ray casting test to detect the contact point sinkage
    // 
    
    m_timer_ray_casting.start();
    m_num_ray_casts = 0;

    // If enabled, update the extent of the moving patch (no ray-hit tests performed outside)
    ChVector2<> patch_min;
    ChVector2<> patch_max;
    if (m_moving_patch) {
        ChVector<> center = m_body->GetFrame_REF_to_abs().TransformPointLocalToParent(m_body_point);
        patch_min.x() = center.x() - m_patch_dim.x() / 2;
        patch_min.y() = center.y() - m_patch_dim.y() / 2;
        patch_max.x() = center.x() + m_patch_dim.x() / 2;
        patch_max.y() = center.y() + m_patch_dim.y() / 2;
    }

    // Loop through all vertices.
    // - set default SCM quantities (in case no ray-hit)
    // - skip vertices outside moving patch (if option enabled)
    // - cast ray and record result in a map (key: vertex index)
    // - initialize patch id to -1 (not set)
    struct HitRecord {
        ChContactable* contactable;  // pointer to hit object
        ChVector<> abs_point;        // hit point, expressed in global frame
        int patch_id;                // index of associated patch id
    };
    std::unordered_map<int, HitRecord> hits;

    for (int i = 0; i < vertices.size(); ++i) {
        // Initialize SCM quantities at current vertex
        p_sigma[i] = 0;
        p_sinkage_elastic[i] = 0;
        p_step_plastic_flow[i] = 0;
        p_erosion[i] = false;
        p_level[i] = plane.TransformParentToLocal(vertices[i]).y();
        p_hit_level[i] = 1e9;

        // Skip vertices outside moving patch
        if (m_moving_patch) {
            if (vertices[i].x() < patch_min.x() || vertices[i].x() > patch_max.x() || vertices[i].y() < patch_min.y() ||
                vertices[i].y() > patch_max.y()) {
                continue;
            }
        }

        // Perform ray casting from current vertex
        collision::ChCollisionSystem::ChRayhitResult mrayhit_result;
        ChVector<> to = vertices[i] + N * test_high_offset;
        ChVector<> from = to - N * test_low_offset;
        this->GetSystem()->GetCollisionSystem()->RayHit(from, to, mrayhit_result);
        m_num_ray_casts++;
        if (mrayhit_result.hit) {
            HitRecord record = { mrayhit_result.hitModel->GetContactable(), mrayhit_result.abs_hitPoint, -1 };
            hits.insert(std::make_pair(i, record));
        }
    }

    // Loop through all hit vertices and determine to which contact patch they belong.
    // We use here the connected_vertexes map (from a vertex to its adjacent vertices) which is
    // set up at initialization and updated when the mesh is refined (if refinement is enabled).
    // Use a queue-based flood-filling algorithm.
    int num_patches = 0;
    for (auto& h : hits) {
        int i = h.first;
        if (h.second.patch_id != -1)                               // move on if vertex already assigned to a patch
            continue;                                              //
        std::queue<int> todo;                                      //
        h.second.patch_id = num_patches++;                         // assign this vertex to a new patch
        todo.push(i);                                              // add vertex to end of queue
        while (!todo.empty()) {                                    //
            auto crt = hits.find(todo.front());                    // current vertex is first element in queue
            todo.pop();                                            // remove first element of queue
            auto crt_i = crt->first;                               //
            auto crt_patch = crt->second.patch_id;                 //
            for (const auto& nbr_i : connected_vertexes[crt_i]) {  // loop over all neighbors
                auto nbr = hits.find(nbr_i);                       // look for neighbor in list of hit vertices
                if (nbr == hits.end())                             // move on if neighbor is not a hit vertex
                    continue;                                      //
                if (nbr->second.patch_id != -1)                    // (COULD BE REMOVED, unless we update patch area)
                    continue;                                      //
                nbr->second.patch_id = crt_patch;                  // assign neighbor to same patch
                todo.push(nbr_i);                                  // add neighbor to end of queue
            }
        }
    }

    // Collect hit vertices assigned to each patch.
    struct PatchRecord {
        std::vector<ChVector2<>> points;  // points in patch (projected on reference plane)
        double area;                      // patch area
        double perimeter;                 // patch perimeter
        double Kc_b;                      // approximate Bekker Kc/b value
    };
    std::vector<PatchRecord> patches(num_patches);
    for (auto& h : hits) {
        ChVector<> v = plane.TransformParentToLocal(vertices[h.first]);
        patches[h.second.patch_id].points.push_back(ChVector2<>(v.x(), v.z()));
    }

    // Calculate area and perimeter of each patch.
    // Calculate approximation to Beker term Kc/b.
    for (auto& p : patches) {
        if (Bekker_Kc == 0) {
            p.Kc_b = 0;
            continue;
        }

        utils::ChConvexHull2D ch(p.points);
        p.area = ch.GetArea();
        p.perimeter = ch.GetPerimeter();
        if (p.area < 1e-6) {
            p.Kc_b = 0;
        } else {
            double b = 2 * p.area / p.perimeter;
            p.Kc_b = Bekker_Kc / b;
        }
    }

    // Process only hit vertices
    for (auto& h : hits) {
        int i = h.first;
        ChContactable* contactable = h.second.contactable;
        const ChVector<>& abs_point = h.second.abs_point;
        int patch_id = h.second.patch_id;

        double p_hit_offset = 1e9;

        p_hit_level[i] = plane.TransformParentToLocal(abs_point).y();
        p_hit_offset = -p_hit_level[i] + p_level_initial[i];

        p_speeds[i] = contactable->GetContactPointSpeed(vertices[i]);

        ChVector<> T = -p_speeds[i];
        T = plane.TransformDirectionParentToLocal(T);
        double Vn = -T.y();
        T.y() = 0;
        T = plane.TransformDirectionLocalToParent(T);
        T.Normalize();

        // Compute i-th force:
        ChVector<> Fn;
        ChVector<> Ft;

        // Elastic try:
        p_sigma[i] = elastic_K * (p_hit_offset - p_sinkage_plastic[i]);

        // Handle unilaterality:
        if (p_sigma[i] < 0) {
            p_sigma[i] = 0;
        } else {
            // add compressive speed-proportional damping
            ////if (Vn < 0) {
            ////    p_sigma[i] += -Vn * this->damping_R;
            ////}

            p_sinkage[i] = p_hit_offset;
            p_level[i] = p_hit_level[i];

            // Accumulate shear for Janosi-Hanamoto
            p_kshear[i] += Vdot(p_speeds[i], -T) * GetSystem()->GetStep();

            // Plastic correction:
            if (p_sigma[i] > p_sigma_yeld[i]) {
                // Bekker formula
                p_sigma[i] = (patches[patch_id].Kc_b + Bekker_Kphi) * pow(p_sinkage[i], Bekker_n);
                p_sigma_yeld[i] = p_sigma[i];
                double old_sinkage_plastic = p_sinkage_plastic[i];
                p_sinkage_plastic[i] = p_sinkage[i] - p_sigma[i] / elastic_K;
                p_step_plastic_flow[i] = (p_sinkage_plastic[i] - old_sinkage_plastic) / GetSystem()->GetStep();
            }

            p_sinkage_elastic[i] = p_sinkage[i] - p_sinkage_plastic[i];

            // add compressive speed-proportional damping (not clamped by pressure yield)
            ////if (Vn < 0) {
            p_sigma[i] += -Vn * damping_R;
            ////}

            // Mohr-Coulomb
            double tau_max = Mohr_cohesion + p_sigma[i] * tan(Mohr_friction * CH_C_DEG_TO_RAD);

            // Janosi-Hanamoto
            p_tau[i] = tau_max * (1.0 - exp(-(p_kshear[i] / Janosi_shear)));

            Fn = N * p_area[i] * p_sigma[i];
            Ft = T * p_area[i] * p_tau[i];

            if (ChBody* rigidbody = dynamic_cast<ChBody*>(contactable)) {
                // [](){} Trick: no deletion for this shared ptr, since 'rigidbody' was not a new ChBody()
                // object, but an already used pointer because mrayhit_result.hitModel->GetPhysicsItem()
                // cannot return it as shared_ptr, as needed by the ChLoadBodyForce:
                std::shared_ptr<ChBody> srigidbody(rigidbody, [](ChBody*) {});
                std::shared_ptr<ChLoadBodyForce> mload(
                    new ChLoadBodyForce(srigidbody, Fn + Ft, false, vertices[i], false));
                this->Add(mload);

                // Accumulate contact force for this rigid body.
                // The resultant force is assumed to be applied at the body COM.
                // All components of the generalized terrain force are expressed in the global frame.
                auto itr = m_contact_forces.find(contactable);
                if (itr == m_contact_forces.end()) {
                    // Create new entry and initialize generalized force.
                    ChVector<> force = Fn + Ft;
                    TerrainForce frc;
                    frc.point = srigidbody->GetPos();
                    frc.force = force;
                    frc.moment = Vcross(Vsub(vertices[i], srigidbody->GetPos()), force);
                    m_contact_forces.insert(std::make_pair(contactable, frc));
                } else {
                    // Update generalized force.
                    ChVector<> force = Fn + Ft;
                    itr->second.force += force;
                    itr->second.moment += Vcross(Vsub(vertices[i], srigidbody->GetPos()), force);
                }
            } else if (ChLoadableUV* surf = dynamic_cast<ChLoadableUV*>(contactable)) {
                // [](){} Trick: no deletion for this shared ptr
                std::shared_ptr<ChLoadableUV> ssurf(surf, [](ChLoadableUV*) {});
                std::shared_ptr<ChLoad<ChLoaderForceOnSurface>> mload(new ChLoad<ChLoaderForceOnSurface>(ssurf));
                mload->loader.SetForce(Fn + Ft);
                mload->loader.SetApplication(0.5, 0.5);  //***TODO*** set UV, now just in middle
                this->Add(mload);

                // Accumulate contact forces for this surface.
                //// TODO
            }

            // Update mesh representation
            vertices[i] = p_vertices_initial[i] - N * p_sinkage[i];

        }  // end positive contact force

    } // end loop on ray hits

    m_timer_ray_casting.stop();

    //
    // Refine the mesh detail
    //

    m_num_marked_faces = 0;
    m_timer_refinement.start();

    if (do_refinement) {

        std::vector<std::vector<double>*> aux_data_double;
        aux_data_double.push_back(&p_level);
        aux_data_double.push_back(&p_level_initial);
        aux_data_double.push_back(&p_hit_level);
        aux_data_double.push_back(&p_sinkage);
        aux_data_double.push_back(&p_sinkage_plastic);
        aux_data_double.push_back(&p_sinkage_elastic);
        aux_data_double.push_back(&p_step_plastic_flow);
        aux_data_double.push_back(&p_kshear);
        aux_data_double.push_back(&p_area);
        aux_data_double.push_back(&p_sigma);
        aux_data_double.push_back(&p_sigma_yeld);
        aux_data_double.push_back(&p_tau);
        aux_data_double.push_back(&p_massremainder);
        std::vector<std::vector<int>*> aux_data_int;
        aux_data_int.push_back(&p_id_island);
        std::vector<std::vector<bool>*> aux_data_bool;
        aux_data_bool.push_back(&p_erosion);
        std::vector<std::vector<ChVector<>>*> aux_data_vect;
        aux_data_vect.push_back(&p_vertices_initial);
        aux_data_vect.push_back(&p_speeds);

        // loop on triangles to see which needs refinement
        std::vector<int> marked_tris;
        for (int it = 0; it < idx_vertices.size(); ++it) {
            // see if at least one of the vertexes are touching
            if (p_sigma[idx_vertices[it][0]] > 0 ||
                p_sigma[idx_vertices[it][1]] > 0 ||
                p_sigma[idx_vertices[it][2]] > 0) {
                marked_tris.push_back(it);
            }
        }
        m_num_marked_faces = marked_tris.size();

        // custom edge refinement criterion: do not use default edge length, 
        // length of the edge as projected on soil plane
        class MyRefinement : public geometry::ChTriangleMeshConnected::ChRefineEdgeCriterion {
        public:
            virtual double ComputeLength(const int vert_a, const int  vert_b, geometry::ChTriangleMeshConnected* mmesh) {
                ChVector<> d = A.MatrT_x_Vect(mmesh->m_vertices[vert_a] - mmesh->m_vertices[vert_b]);
                d.y() = 0;
                return d.Length();
            }
            ChMatrix33<> A;
        };

        MyRefinement refinement_criterion;
        refinement_criterion.A = ChMatrix33<>(this->plane.rot);

        // perform refinement using the LEPP  algorithm, also refining the soil-specific vertex attributes
        for (int i = 0; i < 1; ++i) {
            m_trimesh_shape->GetMesh()->RefineMeshEdges(
                marked_tris,
                refinement_resolution,
                &refinement_criterion,
                0, //&tri_map, // note, update triangle connectivity map incrementally
                aux_data_double,
                aux_data_int,
                aux_data_bool,
                aux_data_vect);
        }
        // TO DO adjust this incrementally

        connected_vertexes.clear();
        connected_vertexes.resize(vertices.size());
        for (unsigned int iface = 0; iface < idx_vertices.size(); ++iface) {
            connected_vertexes[idx_vertices[iface][0]].insert(idx_vertices[iface][1]);
            connected_vertexes[idx_vertices[iface][0]].insert(idx_vertices[iface][2]);
            connected_vertexes[idx_vertices[iface][1]].insert(idx_vertices[iface][0]);
            connected_vertexes[idx_vertices[iface][1]].insert(idx_vertices[iface][2]);
            connected_vertexes[idx_vertices[iface][2]].insert(idx_vertices[iface][0]);
            connected_vertexes[idx_vertices[iface][2]].insert(idx_vertices[iface][1]);
        }

        // Recompute areas (could be optimized)
        for (unsigned int iv = 0; iv < vertices.size(); ++iv) {
            p_area[iv] = 0;
        }
        for (unsigned int it = 0; it < idx_vertices.size(); ++it) {
            ChVector<> AB = vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]];
            ChVector<> AC = vertices[idx_vertices[it][2]] - vertices[idx_vertices[it][0]];
            AB = plane.TransformDirectionParentToLocal(AB);
            AC = plane.TransformDirectionParentToLocal(AC);
            AB.y() = 0;
            AC.y() = 0;
            double triangle_area = 0.5 * (Vcross(AB, AC)).Length();
            p_area[idx_normals[it][0]] += triangle_area / 3.0;
            p_area[idx_normals[it][1]] += triangle_area / 3.0;
            p_area[idx_normals[it][2]] += triangle_area / 3.0;
        }
    }

    m_timer_refinement.stop();

    //
    // Flow material to the side of rut, using heuristics
    // 

    m_timer_bulldozing.start();

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
            double tot_area_island = p_area[*fillseed];
            fill_front.insert(*fillseed);
            p_id_island[*fillseed] = id_island;
            touched_vertexes.erase(fillseed);
            while (fill_front.size() >0) {
                // fill next front
                std::set<int> fill_front_2;
                for (const auto& ifront : fill_front) {
                    for (const auto& ivconnect : connected_vertexes[ifront]) {
                        if ((p_sigma[ivconnect]>0) && (p_id_island[ivconnect]==0)) {
                            ++n_vert_island;
                            tot_step_flow_island += p_area[ivconnect] * p_step_plastic_flow[ivconnect] * this->GetSystem()->GetStep();
                            tot_Nforce_island += p_area[ivconnect] * p_sigma[ivconnect];
                            tot_area_island += p_area[ivconnect];
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
            ////GetLog() << " island " << id_island << " flow volume =" << tot_step_flow_island << " N force=" << tot_Nforce_island << "\n"; 

            // Raise the boundary because of material flow (it gives a sharp spike around the
            // island boundary, but later we'll use the erosion algorithm to smooth it out)

            for (const auto& ibv : boundary) {
                double d_y = bulldozing_flow_factor * ((p_area[ibv]/tot_area_boundary) *  (1/p_area[ibv]) * tot_step_flow_island);
                double clamped_d_y = d_y; // ChMin(d_y, ChMin(p_hit_level[ibv]-p_level[ibv], test_high_offset) );
                if (d_y > p_hit_level[ibv]-p_level[ibv]) {
                    p_massremainder[ibv] += d_y - (p_hit_level[ibv]-p_level[ibv]);
                    clamped_d_y = p_hit_level[ibv]-p_level[ibv];
                }
                p_level[ibv]            += clamped_d_y;
                p_level_initial[ibv]    += clamped_d_y;
                vertices[ibv]           += N * clamped_d_y;
                p_vertices_initial[ibv] += N * clamped_d_y;
            }

            domain_boundaries.insert(boundary.begin(), boundary.end());

        }  // end for islands

        //***TEST***
        // int mm = p_massremainder.size();
        // p_massremainder.clear();p_massremainder.resize(mm);

        // Erosion domain area select, by topologically dilation of all the
        // boundaries of the islands:
        std::set<int> domain_erosion= domain_boundaries;
        for (const auto& ie : domain_boundaries)
            p_erosion[ie] = true;
        std::set<int> front_erosion = domain_boundaries;
        for (int iloop = 0; iloop <10; ++iloop) {
            std::set<int> front_erosion2;
            for(const auto& is : front_erosion) {
                for (const auto& ivconnect : connected_vertexes[is]) {
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
            for (const auto& is : domain_erosion) {
                for (const auto& ivc : connected_vertexes[is]) {
                    ChVector<> vis = this->plane.TransformParentToLocal(vertices[is]);
                    // flow remainder material 
                    if (true) {
                        if (p_massremainder[is]>p_massremainder[ivc]) {
                            double clamped_d_y_i;
                            double clamped_d_y_c;
 
                            // if i higher than c: clamp c upward correction as it might invalidate 
                            // the ceiling constraint, if collision is nearby
                            double d_y_c = (p_massremainder[is]-p_massremainder[ivc])* (1/(double)connected_vertexes[is].size()) *  p_area[is]/(p_area[is]+p_area[ivc]);
                            clamped_d_y_c = d_y_c; 
                            if (d_y_c > p_hit_level[ivc]-p_level[ivc]) {
                                p_massremainder[ivc] += d_y_c - (p_hit_level[ivc]-p_level[ivc]);
                                clamped_d_y_c = p_hit_level[ivc]-p_level[ivc];
                            }
                            double d_y_i = - d_y_c * p_area[ivc]/p_area[is];
                            clamped_d_y_i = d_y_i;
                            if (p_massremainder[is] >  -d_y_i) {
                                p_massremainder[is] -= -d_y_i;
                                clamped_d_y_i = 0;
                            } else
                            if ((p_massremainder[is] < -d_y_i) && (p_massremainder[is] >0)) {
                                p_massremainder[is] = 0;
                                clamped_d_y_i = d_y_i + p_massremainder[is];
                            }
                            
                            // correct vertexes
                            p_level[ivc]            += clamped_d_y_c;
                            p_level_initial[ivc]    += clamped_d_y_c;
                            vertices[ivc]           += N * clamped_d_y_c;
                            p_vertices_initial[ivc] += N * clamped_d_y_c;

                            p_level[is]             += clamped_d_y_i;
                            p_level_initial[is]     += clamped_d_y_i;
                            vertices[is]            += N * clamped_d_y_i;
                            p_vertices_initial[is]  += N * clamped_d_y_i;      
                        }
                    }
                    // smooth
                    if (p_sigma[ivc] == 0) {
                        ChVector<> vic = this->plane.TransformParentToLocal(vertices[ivc]);
                        ChVector<> vdist = vic-vis;
                        vdist.y() = 0;
                        double ddist = vdist.Length();
                        double dy = p_level[is] + p_massremainder[is]  - p_level[ivc] - p_massremainder[ivc];
                        double dy_lim = ddist * tan(bulldozing_erosion_angle*CH_C_DEG_TO_RAD);
                        if (fabs(dy)>dy_lim) {
                            double clamped_d_y_i;
                            double clamped_d_y_c;
                            if (dy > 0) { 
                                // if i higher than c: clamp c upward correction as it might invalidate 
                                // the ceiling constraint, if collision is nearby
                                double d_y_c = (fabs(dy)-dy_lim)* (1/(double)connected_vertexes[is].size()) *  p_area[is]/(p_area[is]+p_area[ivc]);
                                clamped_d_y_c = d_y_c; //clamped_d_y_c = ChMin(d_y_c, p_hit_level[ivc]-p_level[ivc] );
                                if (d_y_c > p_hit_level[ivc]-p_level[ivc]) {
                                    p_massremainder[ivc] += d_y_c - (p_hit_level[ivc]-p_level[ivc]);
                                    clamped_d_y_c = p_hit_level[ivc]-p_level[ivc];
                                }
                                double d_y_i = - d_y_c * p_area[ivc]/p_area[is];
                                clamped_d_y_i = d_y_i;
                                if (p_massremainder[is] >  -d_y_i) {
                                    p_massremainder[is] -= -d_y_i;
                                    clamped_d_y_i = 0;
                                } else
                                if ((p_massremainder[is] < -d_y_i) && (p_massremainder[is] >0)) {
                                    p_massremainder[is] = 0;
                                    clamped_d_y_i = d_y_i + p_massremainder[is];
                                }
                            } else {
                                // if c higher than i: clamp i upward correction as it might invalidate 
                                // the ceiling constraint, if collision is nearby
                                double d_y_i = (fabs(dy)-dy_lim)* (1/(double)connected_vertexes[is].size()) *  p_area[is]/(p_area[is]+p_area[ivc]);
                                clamped_d_y_i = d_y_i; 
                                if (d_y_i > p_hit_level[is]-p_level[is]) {
                                    p_massremainder[is] += d_y_i - (p_hit_level[is]-p_level[is]);
                                    clamped_d_y_i = p_hit_level[is]-p_level[is];
                                }
                                double d_y_c = - d_y_i * p_area[is]/p_area[ivc];
                                clamped_d_y_c = d_y_c;
                                if (p_massremainder[ivc] >  -d_y_c) {
                                    p_massremainder[ivc] -= -d_y_c;
                                    clamped_d_y_c = 0;
                                } else
                                if ((p_massremainder[ivc] < -d_y_c) && (p_massremainder[ivc] >0)) {
                                    p_massremainder[ivc] = 0;
                                    clamped_d_y_c = d_y_c + p_massremainder[ivc];
                                }
                            }

                            // correct vertexes
                            p_level[ivc]            += clamped_d_y_c;
                            p_level_initial[ivc]    += clamped_d_y_c;
                            vertices[ivc]           += N * clamped_d_y_c;
                            p_vertices_initial[ivc] += N * clamped_d_y_c;

                            p_level[is]             += clamped_d_y_i;
                            p_level_initial[is]     += clamped_d_y_i;
                            vertices[is]            += N * clamped_d_y_i;
                            p_vertices_initial[is]  += N * clamped_d_y_i;      
                        }
                    }
                }
            }
        }

    } // end bulldozing flow 

    m_timer_bulldozing.stop();


    m_timer_visualization.start();

    //
    // Update the visualization colors
    // 

    if (plot_type != SCMDeformableTerrain::PLOT_NONE) {
        colors.resize(vertices.size());
        for (size_t iv = 0; iv< vertices.size(); ++iv) {
            ChColor mcolor;
            switch (plot_type) {
                case SCMDeformableTerrain::PLOT_LEVEL:
                    mcolor = ChColor::ComputeFalseColor(p_level[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_LEVEL_INITIAL:
                    mcolor = ChColor::ComputeFalseColor(p_level_initial[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_SINKAGE:
                    mcolor = ChColor::ComputeFalseColor(p_sinkage[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_SINKAGE_ELASTIC:
                    mcolor = ChColor::ComputeFalseColor(p_sinkage_elastic[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_SINKAGE_PLASTIC:
                    mcolor = ChColor::ComputeFalseColor(p_sinkage_plastic[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_STEP_PLASTIC_FLOW:
                    mcolor = ChColor::ComputeFalseColor(p_step_plastic_flow[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_K_JANOSI:
                    mcolor = ChColor::ComputeFalseColor(p_kshear[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_PRESSURE:
                    mcolor = ChColor::ComputeFalseColor(p_sigma[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_PRESSURE_YELD:
                    mcolor = ChColor::ComputeFalseColor(p_sigma_yeld[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_SHEAR:
                    mcolor = ChColor::ComputeFalseColor(p_tau[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_MASSREMAINDER:
                    mcolor = ChColor::ComputeFalseColor(p_massremainder[iv], plot_v_min, plot_v_max);
                    break;
                case SCMDeformableTerrain::PLOT_ISLAND_ID:
                    mcolor = ChColor(0,0,1);
                    if (p_erosion[iv] == true)
                        mcolor = ChColor(1,1,1);
                    if (p_id_island[iv] >0)
                        mcolor = ChColor::ComputeFalseColor(4 +(p_id_island[iv] % 8), 0, 12);
                    if (p_id_island[iv] <0)
                        mcolor = ChColor(0,0,0);
                    break;
                case SCMDeformableTerrain::PLOT_IS_TOUCHED:
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
        ChVector<> nrm = -Vcross(vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]],
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
    for (unsigned int in = 0; in < vertices.size(); ++in) {
        normals[in] /= (double)accumulators[in];
    }

    m_timer_visualization.stop();

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
