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
// Authors: Alessandro Tasora, Radu Serban, Jay Taves
// =============================================================================
//
// Deformable terrain based on SCM (Soil Contact Model) from DLR
// (Krenn & Hirzinger)
//
// =============================================================================

#include <cstdio>
#include <cmath>
#include <queue>
#include <limits>

#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/utils/ChConvexHull.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/stb/stb.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the SCMDeformableTerrain wrapper class
// -----------------------------------------------------------------------------
SCMDeformableTerrain::SCMDeformableTerrain(ChSystem* system, bool visualization_mesh) {
    m_ground = chrono_types::make_shared<SCMDeformableSoil>(system, visualization_mesh);
    system->Add(m_ground);
}

// Return the terrain height at the specified location
double SCMDeformableTerrain::GetHeight(const ChVector<>& loc) const {
    return m_ground->GetHeight(loc);
}

// Return the terrain normal at the specified location
ChVector<> SCMDeformableTerrain::GetNormal(const ChVector<>& loc) const {
    //// TODO
    return m_ground->plane.TransformDirectionLocalToParent(ChWorldFrame::Vertical());
}

// Return the terrain coefficient of friction at the specified location
float SCMDeformableTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : 0.8f;
}

// Set the color of the visualization assets
void SCMDeformableTerrain::SetColor(ChColor color) {
    if (m_ground->m_color)
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
void SCMDeformableTerrain::SetPlane(ChCoordsys<> mplane) {
    m_ground->plane = mplane;
}

// Get the plane reference.
const ChCoordsys<>& SCMDeformableTerrain::GetPlane() const {
    return m_ground->plane;
}

// Get the trimesh that defines the ground shape.
const std::shared_ptr<ChTriangleMeshShape> SCMDeformableTerrain::GetMesh() const {
    return m_ground->m_trimesh_shape;
}

// Enable bulldozing effect.
void SCMDeformableTerrain::SetBulldozingFlow(bool mb) {
    m_ground->do_bulldozing = mb;
}

bool SCMDeformableTerrain::GetBulldozingFlow() const {
    return m_ground->do_bulldozing;
}

// Set properties of the SCM soil model
void SCMDeformableTerrain::SetSoilParameters(
    double Bekker_Kphi,    // Kphi, frictional modulus in Bekker model
    double Bekker_Kc,      // Kc, cohesive modulus in Bekker model
    double Bekker_n,       // n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double Mohr_cohesion,  // Cohesion in, Pa, for shear failure
    double Mohr_friction,  // Friction angle (in degrees!), for shear failure
    double Janosi_shear,   // J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
    double elastic_K,      // elastic stiffness K (must be > Kphi; very high values gives the original SCM model)
    double
        damping_R  // vertical damping R, per unit area (vertical speed proportional, it is zero in original SCM model)
) {
    m_ground->m_Bekker_Kphi = Bekker_Kphi;
    m_ground->m_Bekker_Kc = Bekker_Kc;
    m_ground->m_Bekker_n = Bekker_n;
    m_ground->m_Mohr_cohesion = Mohr_cohesion;
    m_ground->m_Mohr_friction = Mohr_friction;
    m_ground->m_Janosi_shear = Janosi_shear;
    m_ground->m_elastic_K = ChMax(elastic_K, Bekker_Kphi);
    m_ground->m_damping_R = damping_R;
}

void SCMDeformableTerrain::SetBulldozingParameters(
    double mbulldozing_erosion_angle,       ///< angle of erosion of the displaced material (in degrees!)
    double mbulldozing_flow_factor,         ///< growth of lateral volume respect to pressed volume
    int mbulldozing_erosion_n_iterations,   ///< number of erosion refinements per timestep
    int mbulldozing_erosion_n_propagations  ///< number of concentric vertex selections subject to erosion
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
void SCMDeformableTerrain::AddMovingPatch(std::shared_ptr<ChBody> body,
                                          const ChVector<>& OOBB_center,
                                          const ChVector<>& OOBB_dims) {
    SCMDeformableSoil::MovingPatchInfo pinfo;
    pinfo.m_body = body;
    pinfo.m_center = OOBB_center;
    pinfo.m_hdims = OOBB_dims / 2;

    m_ground->m_patches.push_back(pinfo);

    // Moving patch monitoring is now enabled
    m_ground->m_moving_patch = true;
}

// Set user-supplied callback for evaluating location-dependent soil parameters
void SCMDeformableTerrain::RegisterSoilParametersCallback(std::shared_ptr<SoilParametersCallback> cb) {
    m_ground->m_soil_fun = cb;
}

// Initialize the terrain as a flat grid
void SCMDeformableTerrain::Initialize(double sizeX, double sizeY, double delta) {
    m_ground->Initialize(sizeX, sizeY, delta);
}

// Initialize the terrain from a specified height map.
void SCMDeformableTerrain::Initialize(const std::string& heightmap_file,
                                      const std::string& mesh_name,
                                      double sizeX,
                                      double sizeY,
                                      double hMin,
                                      double hMax,
                                      int divX,
                                      int divY) {
    m_ground->Initialize(heightmap_file, mesh_name, sizeX, sizeY, hMin, hMax, divX, divY);
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
    os << "   Number ray-casts:        " << m_ground->m_num_ray_casts << std::endl;
}

// -----------------------------------------------------------------------------
// Implementation of SCMDeformableSoil
// -----------------------------------------------------------------------------

// Constructor.
SCMDeformableSoil::SCMDeformableSoil(ChSystem* system, bool visualization_mesh) : m_soil_fun(nullptr) {
    this->SetSystem(system);

    // Create the default triangle mesh asset
    m_trimesh_shape = std::shared_ptr<ChTriangleMeshShape>(new ChTriangleMeshShape);

    if (visualization_mesh) {
        // Create the default mesh asset
        m_color = std::shared_ptr<ChColorAsset>(new ChColorAsset);
        m_color->SetColor(ChColor(0.3f, 0.3f, 0.3f));
        this->AddAsset(m_color);

        this->AddAsset(m_trimesh_shape);
        m_trimesh_shape->SetWireframe(true);
    }

    do_bulldozing = false;
    bulldozing_flow_factor = 1.2;
    bulldozing_erosion_angle = 40;
    bulldozing_erosion_n_iterations = 3;
    bulldozing_erosion_n_propagations = 10;

    do_refinement = false;
    refinement_resolution = 0.01;

    // Default soil parameters
    m_Bekker_Kphi = 2e6;
    m_Bekker_Kc = 0;
    m_Bekker_n = 1.1;
    m_Mohr_cohesion = 50;
    m_Mohr_friction = 20;
    m_Janosi_shear = 0.01;
    m_elastic_K = 50000000;
    m_damping_R = 0;

    Initialize(3, 3, 0.1);

    plot_type = SCMDeformableTerrain::PLOT_NONE;
    plot_v_min = 0;
    plot_v_max = 0.2;

    test_high_offset = 0.1;
    test_low_offset = 0.5;

    m_moving_patch = false;
}

// Initialize the terrain as a flat grid
void SCMDeformableSoil::Initialize(double sizeX, double sizeY, double delta) {
    m_trimesh_shape->GetMesh()->Clear();
    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();
    std::vector<ChVector<>>& uv_coords = trimesh->getCoordsUV();
    std::vector<ChVector<float>>& colors = trimesh->getCoordsColors();

    int nX = 2 * static_cast<int>(std::ceil((sizeX / 2) / delta));
    int nY = 2 * static_cast<int>(std::ceil((sizeY / 2) / delta));

    m_delta = sizeX / nX;
    m_area = std::pow(m_delta, 2);

    unsigned int nvx = nX + 1;
    unsigned int nvy = nY + 1;
    unsigned int n_verts = nvx * nvy;
    unsigned int n_faces = 2 * nX * nY;
    double x_scale = 1.0 / nX;
    double y_scale = 1.0 / nY;

    // Resize mesh arrays.
    vertices.resize(n_verts);
    normals.resize(n_verts);
    uv_coords.resize(n_verts);
    // colors.resize(n_verts);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    unsigned int iv = 0;
    for (int iy = nvy - 1; iy >= 0; --iy) {
        double y = 0.5 * sizeY - iy * m_delta;
        for (unsigned int ix = 0; ix < nvx; ++ix) {
            double x = ix * m_delta - 0.5 * sizeX;
            // Set vertex location
            vertices[iv] = plane * ChVector<>(x, y, 0);
            // Initialize vertex normal to Y up
            normals[iv] = plane.TransformDirectionLocalToParent(ChVector<>(0, 0, 1));
            // Assign color white to all vertices
            // colors[iv] = ChVector<float>(1, 1, 1);
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

    m_type = PatchType::BOX;
}

// Initialize the terrain from a specified height map.
void SCMDeformableSoil::Initialize(const std::string& heightmap_file,
                                   const std::string& mesh_name,
                                   double sizeX,
                                   double sizeY,
                                   double hMin,
                                   double hMax,
                                   int divX,
                                   int divY) {
    // Read the image file (request only 1 channel) and extract number of pixels.
    STB hmap;
    if (!hmap.ReadFromFile(heightmap_file, 1)) {
        throw ChException("Cannot open height map image file");
    }
    int nx_img = hmap.GetWidth();
    int ny_img = hmap.GetHeight();

    ////std::cout << "image size: " << nx_img << " x " << ny_img << std::endl;
    ////std::cout << "number channels: " << hmap.GetNumChannels() << std::endl;
    ////std::cout << "range: " << hmap.GetRange() << std::endl;

    double dx_img = 1.0 / (nx_img - 1.0);
    double dy_img = 1.0 / (ny_img - 1.0);

    int nx = (divX > 0) ? divX + 1 : nx_img;
    int ny = (divY > 0) ? divY + 1 : ny_img;

    double dx = 1.0 / (nx - 1.0);
    double dy = 1.0 / (ny - 1.0);

    // Resample image and calculate interpolated gray levels
    ChMatrixDynamic<> G(nx, ny);
    for (int ix = 0; ix < nx; ix++) {
        double x = ix * dx;                       // Vertex x location (in [0,1])
        int jx1 = (int)std::floor(x / dx_img);    // Left pixel
        int jx2 = (int)std::ceil(x / dx_img);     // Right pixel
        double ax = (x - jx1 * dx_img) / dx_img;  // Scaled offset from left pixel

        assert(ax < 1.0);
        assert(jx1 < nx_img);
        assert(jx2 < nx_img);
        assert(jx1 <= jx2);

        for (int iy = 0; iy < ny; iy++) {
            double y = iy * dy;                       // Vertex y location (in [0,1])
            int jy1 = (int)std::floor(y / dy_img);    // Up pixel
            int jy2 = (int)std::ceil(y / dy_img);     // Down pixel
            double ay = (y - jy1 * dy_img) / dy_img;  // Scaled offset from down pixel

            assert(ay < 1.0);
            assert(jy1 < ny_img);
            assert(jy2 < ny_img);
            assert(jy1 <= jy2);

            // Gray levels at left-up, left-down, right-up, and right-down pixels
            double g11 = hmap.Gray(jx1, jy1);
            double g12 = hmap.Gray(jx1, jy2);
            double g21 = hmap.Gray(jx2, jy1);
            double g22 = hmap.Gray(jx2, jy2);

            // Bilinear interpolation
            G(ix, iy) = (1 - ax) * (1 - ay) * g11 + (1 - ax) * ay * g12 + ax * (1 - ay) * g21 + ax * ay * g22;
        }
    }

    // Construct a triangular mesh of sizeX x sizeY.
    // Usually, each pixel in the image represents a vertex. Otherwise, use interpolation.
    // The gray level of a pixel is mapped to the height range, with black corresponding
    // to hMin and white corresponding to hMax.
    // UV coordinates are mapped in [0,1] x [0,1].
    // We use smoothed vertex normals.

    auto trimesh = m_trimesh_shape->GetMesh();
    trimesh->Clear();

    // Readability aliases
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<>>& coordsUV = trimesh->getCoordsUV();
    std::vector<ChVector<float>>& colors = trimesh->getCoordsColors();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();

    // Resize mesh arrays.
    unsigned int n_verts = nx * ny;
    unsigned int n_faces = 2 * (nx - 1) * (ny - 1);

    vertices.resize(n_verts);
    normals.resize(n_verts);
    coordsUV.resize(n_verts);
    colors.resize(n_verts);

    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Load mesh vertices.
    // Note that pixels in the image start at top-left corner.
    // We order the vertices starting at the bottom-left corner, row after row.
    // The bottom-left corner corresponds to the point (-sizeX/2, -sizeY/2).
    double h_scale = (hMax - hMin) / hmap.GetRange();

    unsigned int iv = 0;
    for (int iy = ny - 1; iy >= 0; --iy) {                     //
        double y = (0.5 - iy * dy) * sizeY;                    // Vertex y location
        for (int ix = 0; ix < nx; ++ix) {                      //
            double x = (ix * dx - 0.5) * sizeX;                // Vertex x location
            double z = hMin + G(ix, iy) * h_scale;             // Map gray level to vertex height
            vertices[iv] = plane * ChVector<>(x, y, z);        // Set vertex location
            normals[iv] = ChVector<>(0, 0, 0);                 // Initialize vertex normal to (0, 0, 0)
            colors[iv] = ChVector<float>(1, 1, 1);             // Assign color white to all vertices
            coordsUV[iv] = ChVector<>(ix * dx, iy * dy, 0.0);  // Set UV coordinates in [0,1] x [0,1]
            ++iv;                                              //
        }
    }

    // Specify triangular faces (two at a time).
    // Specify the face vertices counter-clockwise.
    // Set the normal indices same as the vertex indices.
    unsigned int it = 0;
    for (int iy = ny - 2; iy >= 0; --iy) {
        for (int ix = 0; ix < nx - 1; ++ix) {
            int v0 = ix + nx * iy;
            idx_vertices[it] = ChVector<int>(v0, v0 + nx + 1, v0 + nx);
            idx_normals[it] = ChVector<int>(v0, v0 + nx + 1, v0 + nx);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nx + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nx + 1);
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

    m_type = PatchType::HEIGHT_MAP;

    ////std::vector<geometry::ChTriangleMeshConnected> meshes = {*trimesh};
    ////trimesh->WriteWavefront("foo.obj", meshes);
}

void SCMDeformableSoil::SetupInitial() {
    // If no user-specified moving patches, create one that will encompass all collision shapes in the system
    if (!m_moving_patch) {
        SCMDeformableSoil::MovingPatchInfo pinfo;
        pinfo.m_body = nullptr;
        m_patches.push_back(pinfo);
    }
}

// Return the terrain height at the specified grid vertex
double SCMDeformableSoil::GetHeight(const ChVector2<int>& loc) {
    // First query the hash-map
    auto p = m_grid_map.find(loc);
    if (p != m_grid_map.end())
        return p->second.p_level;

    // Else return undeformed height
    switch (m_type) {
        case PatchType::BOX:
            return 0;
        case PatchType::HEIGHT_MAP:
            //// TODO
            return 0;
        default:
            return 0;
    }
}

// Return the terrain height at the specified location
double SCMDeformableSoil::GetHeight(const ChVector<>& loc) const {
    //// TODO
    return 0;
}

// Set up auxiliary data structures.
void SCMDeformableSoil::SetupAuxData() {
    // ----------------------
    // OLD
    // Don't think we need this except for the m_trimesh initialization if we stick with that for viz
    // ----------------------

    // better readability:
    std::vector<ChVector<int>>& idx_vertices = m_trimesh_shape->GetMesh()->getIndicesVertexes();
    std::vector<ChVector<>>& vertices = m_trimesh_shape->GetMesh()->getCoordsVertices();

    // Reset and initialize computation data:
    //
    p_vertices_initial = vertices;
    p_speeds.resize(vertices.size());
    p_step_plastic_flow.resize(vertices.size());
    p_level.resize(vertices.size());
    p_level_initial.resize(vertices.size());
    p_hit_level.resize(vertices.size());
    p_sinkage.resize(vertices.size());
    p_sinkage_plastic.resize(vertices.size());
    p_sinkage_elastic.resize(vertices.size());
    p_kshear.resize(vertices.size());
    p_area.resize(vertices.size());
    p_sigma.resize(vertices.size());
    p_sigma_yeld.resize(vertices.size());
    p_tau.resize(vertices.size());
    p_massremainder.resize(vertices.size());
    p_id_island.resize(vertices.size());
    p_erosion.resize(vertices.size());

    for (int i = 0; i < vertices.size(); ++i) {
        p_level[i] = plane.TransformParentToLocal(vertices[i]).z();
        p_level_initial[i] = p_level[i];
    }

    connected_vertexes.resize(vertices.size());
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

// Synchronize information for a moving patch
void SCMDeformableSoil::UpdateMovingPatch(MovingPatchInfo& p) {
    ChVector2<> p_min(+std::numeric_limits<double>::max());
    ChVector2<> p_max(-std::numeric_limits<double>::max());

    // Loop over all corners of the OOBB
    for (int j = 0; j < 8; j++) {
        int ix = j % 2;
        int iy = (j / 2) % 2;
        int iz = (j / 4);

        // OOBB corner in body frame
        ChVector<> c_body = p.m_center + p.m_hdims * ChVector<>(2.0 * ix - 1, 2.0 * iy - 1, 2.0 * iz - 1);
        // OOBB corner in absolute frame
        ChVector<> c_abs = p.m_body->GetFrame_REF_to_abs().TransformPointLocalToParent(c_body);
        // OOBB corner expressed in SCM frame
        ChVector<> c_scm = plane.TransformPointParentToLocal(c_abs);

        // Update AABB of patch projection onto SCM plane
        p_min.x() = std::min(p_min.x(), c_scm.x());
        p_min.y() = std::min(p_min.y(), c_scm.y());
        p_max.x() = std::max(p_max.x(), c_scm.x());
        p_max.y() = std::max(p_max.y(), c_scm.y());
    }

    // Find index ranges for grid vertices contained in the patch projection AABB
    p.m_bl.x() = static_cast<int>(std::ceil(p_min.x() / m_delta));
    p.m_bl.y() = static_cast<int>(std::ceil(p_min.y() / m_delta));
    p.m_tr.x() = static_cast<int>(std::floor(p_max.x() / m_delta));
    p.m_tr.y() = static_cast<int>(std::floor(p_max.y() / m_delta));
}

// Synchronize information for fixed patch
void SCMDeformableSoil::UpdateFixedPatch(MovingPatchInfo& p) {
    ChVector2<> p_min(+std::numeric_limits<double>::max());
    ChVector2<> p_max(-std::numeric_limits<double>::max());

    // Get current bounding box (AABB) of all collision shapes
    ChVector<> aabb_min;
    ChVector<> aabb_max;
    GetSystem()->GetCollisionSystem()->GetBoundingBox(aabb_min, aabb_max);

    // Loop over all corners of the AABB
    for (int j = 0; j < 8; j++) {
        int ix = j % 2;
        int iy = (j / 2) % 2;
        int iz = (j / 4);

        // AABB corner in absolute frame
        ChVector<> c_abs = aabb_max * ChVector<>(ix, iy, iz) + aabb_min * ChVector<>(1.0 - ix, 1.0 - iy, 1.0 - iz);
        // AABB corner in SCM frame
        ChVector<> c_scm = plane.TransformPointParentToLocal(c_abs);

        // Update AABB of patch projection onto SCM plane
        p_min.x() = std::min(p_min.x(), c_scm.x());
        p_min.y() = std::min(p_min.y(), c_scm.y());
        p_max.x() = std::max(p_max.x(), c_scm.x());
        p_max.y() = std::max(p_max.y(), c_scm.y());
    }

    // Find index ranges for grid vertices contained in the patch projection AABB
    p.m_bl.x() = static_cast<int>(std::ceil(p_min.x() / m_delta));
    p.m_bl.y() = static_cast<int>(std::ceil(p_min.y() / m_delta));
    p.m_tr.x() = static_cast<int>(std::floor(p_max.x() / m_delta));
    p.m_tr.y() = static_cast<int>(std::floor(p_max.y() / m_delta));
}

// Offsets for the 8 neighbors of a grid vertex
static const std::vector<ChVector2<int>> neighbors{
    ChVector2<int>(-1, -1),  // SW
    ChVector2<int>(0, -1),   // S
    ChVector2<int>(1, -1),   // SE
    ChVector2<int>(-1, 0),   // W
    ChVector2<int>(1, 0),    // E
    ChVector2<int>(-1, 1),   // NW
    ChVector2<int>(0, 1),    // N
    ChVector2<int>(1, 1)     // NE
};

// Reset the list of forces, and fills it with forces from a soil contact model.
void SCMDeformableSoil::ComputeInternalForces() {
    m_timer_ray_casting.reset();
    m_timer_refinement.reset();
    m_timer_bulldozing.reset();
    m_timer_visualization.reset();

    // TODO : We shouldn't need these anymore, but they'll throw compile errors until I'm done with the whole function
    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<float>>& colors = trimesh->getCoordsColors();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();

    // Reset the load list and map of contact forces
    this->GetLoadList().clear();
    m_contact_forces.clear();

    ChVector<> N = plane.TransformDirectionLocalToParent(ChVector<>(0, 0, 1));

    // Perform ray casting test to detect the contact point sinkage
    m_timer_ray_casting.start();
    m_num_ray_casts = 0;

    // Update moving patch information
    if (m_moving_patch) {
        for (auto& p : m_patches)
            UpdateMovingPatch(p);
    } else {
        assert(m_patches.size() == 1);
        UpdateFixedPatch(m_patches[0]);
    }

    struct HitRecord {
        ChContactable* contactable;  // pointer to hit object
        ChVector<> abs_point;        // hit point, expressed in global frame
        int patch_id;                // index of associated patch id
    };
    std::unordered_map<ChVector2<int>, HitRecord, CoordHash> hits;

    int num_hits = 0;

    // Loop through all moving patches (user-defined or default one)
    for (auto& p : m_patches) {
        // Loop through all vertices in this range
        for (int i = p.m_bl.x(); i <= p.m_tr.x(); i++) {
            for (int j = p.m_bl.y(); j <= p.m_tr.y(); j++) {
                ChVector2<int> ij(i, j);

                // Move from (i, j) to (x, y, z) representation in the world frame
                double x = i * m_delta;
                double y = j * m_delta;
                double z = GetHeight(ij);
                ChVector<> vertex_abs = plane.TransformPointLocalToParent(ChVector<>(x, y, z));

                // Raycast to see if we need to work on this point later
                collision::ChCollisionSystem::ChRayhitResult mrayhit_result;
                ChVector<> to = vertex_abs + N * test_high_offset;
                ChVector<> from = to - N * test_low_offset;
                GetSystem()->GetCollisionSystem()->RayHit(from, to, mrayhit_result);
                m_num_ray_casts++;

                if (mrayhit_result.hit) {
                    // If this point has never been hit before, initialize it
                    if (m_grid_map.find(ij) == m_grid_map.end()) {
                        m_grid_map.insert(std::make_pair(ij, VertexRecord(z)));
                    }

                    // Add to our map of hits to process
                    HitRecord record = {mrayhit_result.hitModel->GetContactable(), mrayhit_result.abs_hitPoint, -1};
                    hits.insert(std::make_pair(ij, record));
                    num_hits++;
                }
            }
        }
    }

    // Collect hit vertices assigned to each contact patch.
    struct ContactPatchRecord {
        std::vector<ChVector2<>> points;  // points in contact patch (projected on reference plane)
        double area;                      // contact patch area
        double perimeter;                 // contact patch perimeter
        double oob;                       // approximate value of 1/b
    };
    std::vector<ContactPatchRecord> contact_patches;

    // Profile this flood-fill once the whole thing is complete - there could be some ways to make it faster:
    // - automatically traverse east and west quickly
    // - only ij-coords and patch_id are needed, may not need the whole hit_record
    // - a helpful metric is the number of times each vertex is visited
    // - could be cost-effective to remove elements when they are tagged as neighbors - but then we have to add them
    // somewhere else so that they aren't forgotten, this copying may make it not worth it

    // Loop through all hit vertices and determine to which contact patch they belong.
    // Use a queue-based flood-filling algorithm based on the 8 neighbors of each hit vertex.
    int num_patches = 0;
    for (auto& h : hits) {
        if (h.second.patch_id != -1)
            continue;

        ChVector2<int> ij = h.first;

        // Make a new contact patch and add this hit vertex to it
        h.second.patch_id = num_patches++;
        ContactPatchRecord patch;
        patch.points.push_back(ChVector2<>(m_delta * ij.x(), m_delta * ij.y()));

        // Add current vertex to the work queue
        std::queue<ChVector2<int>> todo;
        todo.push(ij);

        while (!todo.empty()) {
            auto crt = hits.at(todo.front());  // Current hit vertex is first element in queue
            todo.pop();                        // Remove first element from queue

            int crt_patch = crt.patch_id;

            // Loop through the 8 neighbors of this hit vertex
            for (int k = 0; k < 8; k++) {
                ChVector2<int> nbr_ij = ij + neighbors[k];
                // If neighbor is not a hit vertex, move on
                auto nbr = hits.find(nbr_ij);
                if (nbr == hits.end())
                    continue;
                if (nbr->second.patch_id != -1)
                    continue;
                // Assign neighbor to the same contact patch
                nbr->second.patch_id = crt_patch;
                // Add neighbor to same contact patch and to the queue
                patch.points.push_back(ChVector2<>(m_delta * nbr_ij.x(), m_delta * nbr_ij.y()));
                // Add neighbor to end of work queue
                todo.push(nbr_ij);
            }
        }
        contact_patches.push_back(patch);
    }

    // Calculate area and perimeter of each contact patch.
    // Calculate approximation to Beker term 1/b.
    for (auto& p : contact_patches) {
        utils::ChConvexHull2D ch(p.points);
        p.area = ch.GetArea();
        p.perimeter = ch.GetPerimeter();
        if (p.area < 1e-6) {
            p.oob = 0;
        } else {
            p.oob = p.perimeter / (2 * p.area);
        }
    }

    // Initialize local values for the soil parameters
    double Bekker_Kphi = m_Bekker_Kphi;
    double Bekker_Kc = m_Bekker_Kc;
    double Bekker_n = m_Bekker_n;
    double Mohr_cohesion = m_Mohr_cohesion;
    double Mohr_friction = m_Mohr_friction;
    double Janosi_shear = m_Janosi_shear;
    double elastic_K = m_elastic_K;
    double damping_R = m_damping_R;

    // Process only hit vertices
    for (auto& h : hits) {
        ChVector2<> ij = h.first;

        auto v = m_grid_map.at(ij);

        ChContactable* contactable = h.second.contactable;
        const ChVector<>& abs_point = h.second.abs_point;
        int patch_id = h.second.patch_id;

        auto loc_point = plane.TransformParentToLocal(abs_point);

        if (m_soil_fun) {
            m_soil_fun->Set(loc_point.x(), loc_point.y());

            Bekker_Kphi = m_soil_fun->m_Bekker_Kphi;
            Bekker_Kc = m_soil_fun->m_Bekker_Kc;
            Bekker_n = m_soil_fun->m_Bekker_n;
            Mohr_cohesion = m_soil_fun->m_Mohr_cohesion;
            Mohr_friction = m_soil_fun->m_Mohr_friction;
            Janosi_shear = m_soil_fun->m_Janosi_shear;
            elastic_K = m_soil_fun->m_elastic_K;
            damping_R = m_soil_fun->m_damping_R;
        }

        v.p_hit_level = loc_point.z();
        double p_hit_offset = -v.p_hit_level + v.p_level_initial;

        ChVector<> vertex =
            plane.TransformPointLocalToParent(ChVector<>(ij.x() * m_delta, ij.y() * m_delta, v.p_level));

        v.p_speeds = contactable->GetContactPointSpeed(vertex);

        ChVector<> T = -v.p_speeds;
        T = plane.TransformDirectionParentToLocal(T);
        double Vn = -T.z();
        T.z() = 0;
        T = plane.TransformDirectionLocalToParent(T);
        T.Normalize();

        // Compute i-th force:
        ChVector<> Fn;
        ChVector<> Ft;

        // Elastic try:
        v.p_sigma = elastic_K * (p_hit_offset - v.p_sinkage_plastic);

        // Handle unilaterality:
        if (v.p_sigma < 0) {
            v.p_sigma = 0;
        } else {
            // add compressive speed-proportional damping
            ////if (Vn < 0) {
            ////    v.p_sigma += -Vn * this->damping_R;
            ////}

            v.p_sinkage = p_hit_offset;
            v.p_level = v.p_hit_level;

            // Accumulate shear for Janosi-Hanamoto
            v.p_kshear += Vdot(v.p_speeds, -T) * GetSystem()->GetStep();

            // Plastic correction:
            if (v.p_sigma > v.p_sigma_yield) {
                // Bekker formula
                v.p_sigma = (contact_patches[patch_id].oob * Bekker_Kc + Bekker_Kphi) * pow(v.p_sinkage, Bekker_n);
                v.p_sigma_yield = v.p_sigma;
                double old_sinkage_plastic = v.p_sinkage_plastic;
                v.p_sinkage_plastic = v.p_sinkage - v.p_sigma / elastic_K;
                v.p_step_plastic_flow = (v.p_sinkage_plastic - old_sinkage_plastic) / GetSystem()->GetStep();
            }

            v.p_sinkage_elastic = v.p_sinkage - v.p_sinkage_plastic;

            // add compressive speed-proportional damping (not clamped by pressure yield)
            ////if (Vn < 0) {
            v.p_sigma += -Vn * damping_R;
            ////}

            // Mohr-Coulomb
            double tau_max = Mohr_cohesion + v.p_sigma * tan(Mohr_friction * CH_C_DEG_TO_RAD);

            // Janosi-Hanamoto
            v.p_tau = tau_max * (1.0 - exp(-(v.p_kshear / Janosi_shear)));

            Fn = N * m_area * v.p_sigma;
            Ft = T * m_area * v.p_tau;

            if (ChBody* rigidbody = dynamic_cast<ChBody*>(contactable)) {
                // [](){} Trick: no deletion for this shared ptr, since 'rigidbody' was not a new ChBody()
                // object, but an already used pointer because mrayhit_result.hitModel->GetPhysicsItem()
                // cannot return it as shared_ptr, as needed by the ChLoadBodyForce:
                std::shared_ptr<ChBody> srigidbody(rigidbody, [](ChBody*) {});
                std::shared_ptr<ChLoadBodyForce> mload(new ChLoadBodyForce(srigidbody, Fn + Ft, false, vertex, false));
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
                    frc.moment = Vcross(Vsub(vertex, srigidbody->GetPos()), force);
                    m_contact_forces.insert(std::make_pair(contactable, frc));
                } else {
                    // Update generalized force.
                    ChVector<> force = Fn + Ft;
                    itr->second.force += force;
                    itr->second.moment += Vcross(Vsub(vertex, srigidbody->GetPos()), force);
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

            // This might be the correction needed for p_vertices_initial - need to think more to confirm it's in the
            // right frame and test ChVector<> p_vertices_initial(ij.x() * m_delta, ij.y() * m_delta,
            // v.p_level_initial); v.p_level = plane.TransformPointParentToLocal(p_vertices_initial - N *
            // v.p_sinkage).z();

            // Oops, v.p_vertices_initial has not been initialized here...
            v.p_level = plane.TransformPointParentToLocal(v.p_vertices_initial - N * v.p_sinkage).z();

        }  // end positive contact force

    }  // end loop on ray hits

    m_timer_ray_casting.stop();

    //
    // Flow material to the side of rut, using heuristics
    //

    m_timer_bulldozing.start();

    // if (do_bulldozing) {
    //     std::set<int> touched_vertexes;
    //     for (int iv = 0; iv < vertices.size(); ++iv) {
    //         p_id_island[iv] = 0;
    //         if (p_sigma[iv] > 0)
    //             touched_vertexes.insert(iv);
    //     }

    //     std::set<int> domain_boundaries;

    //     // Compute contact islands (and their displaced material) by flood-filling the mesh
    //     int id_island = 0;
    //     for (auto fillseed = touched_vertexes.begin(); fillseed != touched_vertexes.end();
    //          fillseed = touched_vertexes.begin()) {
    //         // new island:
    //         ++id_island;
    //         std::set<int> fill_front;

    //         std::set<int> boundary;
    //         int n_vert_boundary = 0;
    //         double tot_area_boundary = 0;

    //         int n_vert_island = 1;
    //         double tot_step_flow_island =
    //             p_area[*fillseed] * p_step_plastic_flow[*fillseed] * this->GetSystem()->GetStep();
    //         double tot_Nforce_island = p_area[*fillseed] * p_sigma[*fillseed];
    //         double tot_area_island = p_area[*fillseed];
    //         fill_front.insert(*fillseed);
    //         p_id_island[*fillseed] = id_island;
    //         touched_vertexes.erase(fillseed);
    //         while (fill_front.size() > 0) {
    //             // fill next front
    //             std::set<int> fill_front_2;
    //             for (const auto& ifront : fill_front) {
    //                 for (const auto& ivconnect : connected_vertexes[ifront]) {
    //                     if ((p_sigma[ivconnect] > 0) && (p_id_island[ivconnect] == 0)) {
    //                         ++n_vert_island;
    //                         tot_step_flow_island +=
    //                             p_area[ivconnect] * p_step_plastic_flow[ivconnect] * this->GetSystem()->GetStep();
    //                         tot_Nforce_island += p_area[ivconnect] * p_sigma[ivconnect];
    //                         tot_area_island += p_area[ivconnect];
    //                         fill_front_2.insert(ivconnect);
    //                         p_id_island[ivconnect] = id_island;
    //                         touched_vertexes.erase(ivconnect);
    //                     } else if ((p_sigma[ivconnect] == 0) && (p_id_island[ivconnect] <= 0) &&
    //                                (p_id_island[ivconnect] != -id_island)) {
    //                         ++n_vert_boundary;
    //                         tot_area_boundary += p_area[ivconnect];
    //                         p_id_island[ivconnect] = -id_island;  // negative to mark as boundary
    //                         boundary.insert(ivconnect);
    //                     }
    //                 }
    //             }
    //             // advance to next front
    //             fill_front = fill_front_2;
    //         }
    //         ////GetLog() << " island " << id_island << " flow volume =" << tot_step_flow_island << " N force=" <<
    //         /// tot_Nforce_island << "\n";

    //         // Raise the boundary because of material flow (it gives a sharp spike around the
    //         // island boundary, but later we'll use the erosion algorithm to smooth it out)

    //         for (const auto& ibv : boundary) {
    //             double d_y = bulldozing_flow_factor *
    //                          ((p_area[ibv] / tot_area_boundary) * (1 / p_area[ibv]) * tot_step_flow_island);
    //             double clamped_d_y = d_y;  // ChMin(d_y, ChMin(p_hit_level[ibv]-p_level[ibv], test_high_offset) );
    //             if (d_y > p_hit_level[ibv] - p_level[ibv]) {
    //                 p_massremainder[ibv] += d_y - (p_hit_level[ibv] - p_level[ibv]);
    //                 clamped_d_y = p_hit_level[ibv] - p_level[ibv];
    //             }
    //             p_level[ibv] += clamped_d_y;
    //             p_level_initial[ibv] += clamped_d_y;
    //             vertices[ibv] += N * clamped_d_y;
    //             p_vertices_initial[ibv] += N * clamped_d_y;
    //         }

    //         domain_boundaries.insert(boundary.begin(), boundary.end());

    //     }  // end for islands

    //     //***TEST***
    //     // int mm = p_massremainder.size();
    //     // p_massremainder.clear();p_massremainder.resize(mm);

    //     // Erosion domain area select, by topologically dilation of all the
    //     // boundaries of the islands:
    //     std::set<int> domain_erosion = domain_boundaries;
    //     for (const auto& ie : domain_boundaries)
    //         p_erosion[ie] = true;
    //     std::set<int> front_erosion = domain_boundaries;
    //     for (int iloop = 0; iloop < 10; ++iloop) {
    //         std::set<int> front_erosion2;
    //         for (const auto& is : front_erosion) {
    //             for (const auto& ivconnect : connected_vertexes[is]) {
    //                 if ((p_id_island[ivconnect] == 0) && (p_erosion[ivconnect] == 0)) {
    //                     front_erosion2.insert(ivconnect);
    //                     p_erosion[ivconnect] = true;
    //                 }
    //             }
    //         }
    //         domain_erosion.insert(front_erosion2.begin(), front_erosion2.end());
    //         front_erosion = front_erosion2;
    //     }
    //     // Erosion smoothing algorithm on domain
    //     for (int ismo = 0; ismo < 3; ++ismo) {
    //         for (const auto& is : domain_erosion) {
    //             for (const auto& ivc : connected_vertexes[is]) {
    //                 ChVector<> vis = this->plane.TransformParentToLocal(vertices[is]);
    //                 // flow remainder material
    //                 if (true) {
    //                     if (p_massremainder[is] > p_massremainder[ivc]) {
    //                         double clamped_d_y_i;
    //                         double clamped_d_y_c;

    //                         // if i higher than c: clamp c upward correction as it might invalidate
    //                         // the ceiling constraint, if collision is nearby
    //                         double d_y_c = (p_massremainder[is] - p_massremainder[ivc]) *
    //                                        (1 / (double)connected_vertexes[is].size()) * p_area[is] /
    //                                        (p_area[is] + p_area[ivc]);
    //                         clamped_d_y_c = d_y_c;
    //                         if (d_y_c > p_hit_level[ivc] - p_level[ivc]) {
    //                             p_massremainder[ivc] += d_y_c - (p_hit_level[ivc] - p_level[ivc]);
    //                             clamped_d_y_c = p_hit_level[ivc] - p_level[ivc];
    //                         }
    //                         double d_y_i = -d_y_c * p_area[ivc] / p_area[is];
    //                         clamped_d_y_i = d_y_i;
    //                         if (p_massremainder[is] > -d_y_i) {
    //                             p_massremainder[is] -= -d_y_i;
    //                             clamped_d_y_i = 0;
    //                         } else if ((p_massremainder[is] < -d_y_i) && (p_massremainder[is] > 0)) {
    //                             p_massremainder[is] = 0;
    //                             clamped_d_y_i = d_y_i + p_massremainder[is];
    //                         }

    //                         // correct vertexes
    //                         p_level[ivc] += clamped_d_y_c;
    //                         p_level_initial[ivc] += clamped_d_y_c;
    //                         vertices[ivc] += N * clamped_d_y_c;
    //                         p_vertices_initial[ivc] += N * clamped_d_y_c;

    //                         p_level[is] += clamped_d_y_i;
    //                         p_level_initial[is] += clamped_d_y_i;
    //                         vertices[is] += N * clamped_d_y_i;
    //                         p_vertices_initial[is] += N * clamped_d_y_i;
    //                     }
    //                 }
    //                 // smooth
    //                 if (p_sigma[ivc] == 0) {
    //                     ChVector<> vic = this->plane.TransformParentToLocal(vertices[ivc]);
    //                     ChVector<> vdist = vic - vis;
    //                     vdist.z() = 0;
    //                     double ddist = vdist.Length();
    //                     double dy = p_level[is] + p_massremainder[is] - p_level[ivc] - p_massremainder[ivc];
    //                     double dy_lim = ddist * tan(bulldozing_erosion_angle * CH_C_DEG_TO_RAD);
    //                     if (fabs(dy) > dy_lim) {
    //                         double clamped_d_y_i;
    //                         double clamped_d_y_c;
    //                         if (dy > 0) {
    //                             // if i higher than c: clamp c upward correction as it might invalidate
    //                             // the ceiling constraint, if collision is nearby
    //                             double d_y_c = (fabs(dy) - dy_lim) * (1 / (double)connected_vertexes[is].size()) *
    //                                            p_area[is] / (p_area[is] + p_area[ivc]);
    //                             clamped_d_y_c = d_y_c;  // clamped_d_y_c = ChMin(d_y_c, p_hit_level[ivc]-p_level[ivc]
    //                             ); if (d_y_c > p_hit_level[ivc] - p_level[ivc]) {
    //                                 p_massremainder[ivc] += d_y_c - (p_hit_level[ivc] - p_level[ivc]);
    //                                 clamped_d_y_c = p_hit_level[ivc] - p_level[ivc];
    //                             }
    //                             double d_y_i = -d_y_c * p_area[ivc] / p_area[is];
    //                             clamped_d_y_i = d_y_i;
    //                             if (p_massremainder[is] > -d_y_i) {
    //                                 p_massremainder[is] -= -d_y_i;
    //                                 clamped_d_y_i = 0;
    //                             } else if ((p_massremainder[is] < -d_y_i) && (p_massremainder[is] > 0)) {
    //                                 p_massremainder[is] = 0;
    //                                 clamped_d_y_i = d_y_i + p_massremainder[is];
    //                             }
    //                         } else {
    //                             // if c higher than i: clamp i upward correction as it might invalidate
    //                             // the ceiling constraint, if collision is nearby
    //                             double d_y_i = (fabs(dy) - dy_lim) * (1 / (double)connected_vertexes[is].size()) *
    //                                            p_area[is] / (p_area[is] + p_area[ivc]);
    //                             clamped_d_y_i = d_y_i;
    //                             if (d_y_i > p_hit_level[is] - p_level[is]) {
    //                                 p_massremainder[is] += d_y_i - (p_hit_level[is] - p_level[is]);
    //                                 clamped_d_y_i = p_hit_level[is] - p_level[is];
    //                             }
    //                             double d_y_c = -d_y_i * p_area[is] / p_area[ivc];
    //                             clamped_d_y_c = d_y_c;
    //                             if (p_massremainder[ivc] > -d_y_c) {
    //                                 p_massremainder[ivc] -= -d_y_c;
    //                                 clamped_d_y_c = 0;
    //                             } else if ((p_massremainder[ivc] < -d_y_c) && (p_massremainder[ivc] > 0)) {
    //                                 p_massremainder[ivc] = 0;
    //                                 clamped_d_y_c = d_y_c + p_massremainder[ivc];
    //                             }
    //                         }

    //                         // correct vertexes
    //                         p_level[ivc] += clamped_d_y_c;
    //                         p_level_initial[ivc] += clamped_d_y_c;
    //                         vertices[ivc] += N * clamped_d_y_c;
    //                         p_vertices_initial[ivc] += N * clamped_d_y_c;

    //                         p_level[is] += clamped_d_y_i;
    //                         p_level_initial[is] += clamped_d_y_i;
    //                         vertices[is] += N * clamped_d_y_i;
    //                         p_vertices_initial[is] += N * clamped_d_y_i;
    //                     }
    //                 }
    //             }
    //         }
    //     }

    // }  // end bulldozing flow

    m_timer_bulldozing.stop();

    m_timer_visualization.start();

    //
    // Update the visualization colors
    //

    if (plot_type != SCMDeformableTerrain::PLOT_NONE) {
        colors.resize(vertices.size());
        for (size_t iv = 0; iv < vertices.size(); ++iv) {
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
                    mcolor = ChColor(0, 0, 1);
                    if (p_erosion[iv] == true)
                        mcolor = ChColor(1, 1, 1);
                    if (p_id_island[iv] > 0)
                        mcolor = ChColor::ComputeFalseColor(4.0 + (p_id_island[iv] % 8), 0.0, 12.0);
                    if (p_id_island[iv] < 0)
                        mcolor = ChColor(0, 0, 0);
                    break;
                case SCMDeformableTerrain::PLOT_IS_TOUCHED:
                    if (p_sigma[iv] > 0)
                        mcolor = ChColor(1, 0, 0);
                    else
                        mcolor = ChColor(0, 0, 1);
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
    for (unsigned int in = 0; in < vertices.size(); ++in) {
        normals[in] /= (double)accumulators[in];
    }

    m_timer_visualization.stop();

    //
    // Compute the forces
    //

    // Use the SCM soil contact model as described in the paper:
    // "Parameter Identification of a Planetary Rover Wheel�Soil
    // Contact Model via a Bayesian Approach", A.Gallina, R. Krenn et al.

    //
    // Update visual asset
    //

    // Not needed because Update() will happen anyway
    //  ChPhysicsItem::Update(0, true);
}

}  // end namespace vehicle
}  // end namespace chrono
