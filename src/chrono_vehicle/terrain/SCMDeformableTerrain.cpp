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
    return m_ground->m_plane.TransformDirectionLocalToParent(ChWorldFrame::Vertical());
}

// Return the terrain coefficient of friction at the specified location
float SCMDeformableTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : 0.8f;
}

// Set the color of the visualization assets
void SCMDeformableTerrain::SetColor(const ChColor& color) {
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
void SCMDeformableTerrain::SetPlane(const ChCoordsys<>& plane) {
    m_ground->m_plane = plane;
}

// Get the plane reference.
const ChCoordsys<>& SCMDeformableTerrain::GetPlane() const {
    return m_ground->m_plane;
}

// Get the trimesh that defines the ground shape.
std::shared_ptr<ChTriangleMeshShape> SCMDeformableTerrain::GetMesh() const {
    return m_ground->m_trimesh_shape;
}

// Save the visualization mesh as a Wavefront OBJ file.
void SCMDeformableTerrain::WriteMesh(const std::string& filename) const {
    if (!m_ground->m_trimesh_shape) {
        std::cout << "SCMDeformableTerrain::WriteMesh  -- visualization mesh not created.";
        return;
    }
    auto trimesh = m_ground->m_trimesh_shape->GetMesh();
    std::vector<geometry::ChTriangleMeshConnected> meshes = {*trimesh};
    trimesh->WriteWavefront(filename, meshes);
}

// Enable bulldozing effect.
void SCMDeformableTerrain::SetBulldozingFlow(bool mb) {
    m_ground->do_bulldozing = mb;
}

bool SCMDeformableTerrain::BulldozingFlow() const {
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

void SCMDeformableTerrain::SetTestHeight(double offset) {
    m_ground->m_test_offset_up = offset;
}

double SCMDeformableTerrain::GetTestHeight() const {
    return m_ground->m_test_offset_up;
}

// Set the color plot type.
void SCMDeformableTerrain::SetPlotType(DataPlotType plot_type, double min_val, double max_val) {
    m_ground->m_plot_type = plot_type;
    m_ground->m_plot_v_min = min_val;
    m_ground->m_plot_v_max = max_val;
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
                                      double sizeX,
                                      double sizeY,
                                      double hMin,
                                      double hMax,
                                      double delta) {
    m_ground->Initialize(heightmap_file, sizeX, sizeY, hMin, hMax, delta);
}

// Get the grid vertices and their heights that were modified over last step.
std::vector<SCMDeformableTerrain::VertexLevel> SCMDeformableTerrain::GetModifiedVertices() const {
    return m_ground->GetModifiedVertices();
}

// Modify the level of vertices in the underlying grid map from the given list.
void SCMDeformableTerrain::SetModifiedVertices(const std::vector<VertexLevel>& vertices) {
    m_ground->SetModifiedVertices(vertices);
}

// Return the current cumulative contact force on the specified body (due to interaction with the SCM terrain).
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

// Print timing and counter information for last step.
void SCMDeformableTerrain::PrintStepStatistics(std::ostream& os) const {
    os << " Timers:" << std::endl;
    os << "   Ray casting:             " << m_ground->m_timer_ray_casting() << std::endl;
    os << "   Contact patches:         " << m_ground->m_timer_contact_patches() << std::endl;
    os << "   Contact forces:          " << m_ground->m_timer_contact_forces() << std::endl;
    os << "   Bulldozing:              " << m_ground->m_timer_bulldozing() << std::endl;
    os << "   Visualization:           " << m_ground->m_timer_visualization() << std::endl;

    os << " Counters:" << std::endl;
    os << "   Number ray casts:        " << m_ground->m_num_ray_casts << std::endl;
    os << "   Number ray hits:         " << m_ground->m_num_ray_hits << std::endl;
    os << "   Number contact patches:  " << m_ground->m_num_contact_patches << std::endl;
}

// -----------------------------------------------------------------------------
// Implementation of SCMDeformableSoil
// -----------------------------------------------------------------------------

// Constructor.
SCMDeformableSoil::SCMDeformableSoil(ChSystem* system, bool visualization_mesh) : m_soil_fun(nullptr) {
    this->SetSystem(system);

    if (visualization_mesh) {
        // Create the visualization mesh and asset
        m_trimesh_shape = std::shared_ptr<ChTriangleMeshShape>(new ChTriangleMeshShape);
        m_trimesh_shape->SetWireframe(true);
        m_trimesh_shape->SetFixedConnectivity();
        this->AddAsset(m_trimesh_shape);

        // Create the default mesh asset
        m_color = std::shared_ptr<ChColorAsset>(new ChColorAsset);
        m_color->SetColor(ChColor(0.3f, 0.3f, 0.3f));
        this->AddAsset(m_color);
    }

    do_bulldozing = false;
    bulldozing_flow_factor = 1.2;
    bulldozing_erosion_angle = 40;
    bulldozing_erosion_n_iterations = 3;
    bulldozing_erosion_n_propagations = 10;

    // Default soil parameters
    m_Bekker_Kphi = 2e6;
    m_Bekker_Kc = 0;
    m_Bekker_n = 1.1;
    m_Mohr_cohesion = 50;
    m_Mohr_friction = 20;
    m_Janosi_shear = 0.01;
    m_elastic_K = 50000000;
    m_damping_R = 0;

    m_plot_type = SCMDeformableTerrain::PLOT_NONE;
    m_plot_v_min = 0;
    m_plot_v_max = 0.2;

    m_test_offset_up = 0.1;
    m_test_offset_down = 0.5;

    m_moving_patch = false;
}

// Initialize the terrain as a flat grid
void SCMDeformableSoil::Initialize(double sizeX, double sizeY, double delta) {
    m_type = PatchType::BOX;

    m_nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    m_ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction

    m_delta = sizeX / (2 * m_nx);   // grid spacing
    m_area = std::pow(m_delta, 2);  // area of a cell

    int nvx = 2 * m_nx + 1;                     // number of grid vertices in X direction
    int nvy = 2 * m_ny + 1;                     // number of grid vertices in Y direction
    int n_verts = nvx * nvy;                    // total number of vertices for initial visualization trimesh
    int n_faces = 2 * (2 * m_nx) * (2 * m_ny);  // total number of faces for initial visualization trimesh
    double x_scale = 0.5 / m_nx;                // scale for texture coordinates (U direction)
    double y_scale = 0.5 / m_ny;                // scale for texture coordinates (V direction)

    // Return now if no visualization
    if (!m_trimesh_shape)
        return;

    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    trimesh->Clear();
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();
    std::vector<ChVector<>>& uv_coords = trimesh->getCoordsUV();
    std::vector<ChVector<float>>& colors = trimesh->getCoordsColors();

    // Resize mesh arrays
    vertices.resize(n_verts);
    normals.resize(n_verts);
    uv_coords.resize(n_verts);
    colors.resize(n_verts);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    // Load mesh vertices.
    // We order the vertices starting at the bottom-left corner, row after row.
    // The bottom-left corner corresponds to the point (-sizeX/2, -sizeY/2).
    // UV coordinates are mapped in [0,1] x [0,1].
    int iv = 0;
    for (int iy = 0; iy < nvy; iy++) {
        double y = iy * m_delta - 0.5 * sizeY;
        for (int ix = 0; ix < nvx; ix++) {
            double x = ix * m_delta - 0.5 * sizeX;
            // Set vertex location
            vertices[iv] = m_plane * ChVector<>(x, y, 0);
            // Initialize vertex normal to Y up
            normals[iv] = m_plane.TransformDirectionLocalToParent(ChVector<>(0, 0, 1));
            // Assign color white to all vertices
            colors[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            uv_coords[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time).
    // Specify the face vertices counter-clockwise.
    // Set the normal indices same as the vertex indices.
    int it = 0;
    for (int iy = 0; iy < nvy - 1; iy++) {
        for (int ix = 0; ix < nvx - 1; ix++) {
            int v0 = ix + nvx * iy;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            idx_normals[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            ++it;
        }
    }
}

// Initialize the terrain from a specified height map.
void SCMDeformableSoil::Initialize(const std::string& heightmap_file,
                                   double sizeX,
                                   double sizeY,
                                   double hMin,
                                   double hMax,
                                   double delta) {
    m_type = PatchType::HEIGHT_MAP;

    // Read the image file (request only 1 channel) and extract number of pixels.
    STB hmap;
    if (!hmap.ReadFromFile(heightmap_file, 1)) {
        throw ChException("Cannot open height map image file");
    }
    int nx_img = hmap.GetWidth();
    int ny_img = hmap.GetHeight();

    double dx_img = 1.0 / (nx_img - 1.0);
    double dy_img = 1.0 / (ny_img - 1.0);

    m_nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    m_ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction

    m_delta = sizeX / (2.0 * m_nx);  // grid spacing
    m_area = std::pow(m_delta, 2);   // area of a cell

    double dx_grid = 0.5 / m_nx;
    double dy_grid = 0.5 / m_ny;

    int nvx = 2 * m_nx + 1;                     // number of grid vertices in X direction
    int nvy = 2 * m_ny + 1;                     // number of grid vertices in Y direction
    int n_verts = nvx * nvy;                    // total number of vertices for initial visualization trimesh
    int n_faces = 2 * (2 * m_nx) * (2 * m_ny);  // total number of faces for initial visualization trimesh
    double x_scale = 0.5 / m_nx;                // scale for texture coordinates (U direction)
    double y_scale = 0.5 / m_ny;                // scale for texture coordinates (V direction)

    // Resample image and calculate interpolated gray levels and then map it to the height range, with black
    // corresponding to hMin and white corresponding to hMax. Entry (0,0) corresponds to bottom-left grid vertex.
    // Note that pixels in the image start at top-left corner.
    double h_scale = (hMax - hMin) / hmap.GetRange();
    m_heights = ChMatrixDynamic<>(nvx, nvy);
    for (int ix = 0; ix < nvx; ix++) {
        double x = ix * dx_grid;                  // x location in image (in [0,1], 0 at left)
        int jx1 = (int)std::floor(x / dx_img);    // Left pixel
        int jx2 = (int)std::ceil(x / dx_img);     // Right pixel
        double ax = (x - jx1 * dx_img) / dx_img;  // Scaled offset from left pixel

        assert(ax < 1.0);
        assert(jx1 < nx_img);
        assert(jx2 < nx_img);
        assert(jx1 <= jx2);

        for (int iy = 0; iy < nvy; iy++) {
            double y = (2 * m_ny - iy) * dy_grid;     // y location in image (in [0,1], 0 at top)
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

            // Bilinear interpolation (gray level)
            m_heights(ix, iy) = (1 - ax) * (1 - ay) * g11 + (1 - ax) * ay * g12 + ax * (1 - ay) * g21 + ax * ay * g22;
            // Map into height range
            m_heights(ix, iy) = hMin + m_heights(ix, iy) * h_scale;
        }
    }

    // Return now if no visualization
    if (!m_trimesh_shape)
        return;

    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    trimesh->Clear();
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();
    std::vector<ChVector<>>& uv_coords = trimesh->getCoordsUV();
    std::vector<ChVector<float>>& colors = trimesh->getCoordsColors();

    // Resize mesh arrays.
    vertices.resize(n_verts);
    normals.resize(n_verts);
    uv_coords.resize(n_verts);
    colors.resize(n_verts);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    // Load mesh vertices.
    // We order the vertices starting at the bottom-left corner, row after row.
    // The bottom-left corner corresponds to the point (-sizeX/2, -sizeY/2).
    // UV coordinates are mapped in [0,1] x [0,1]. Use smoothed vertex normals.
    int iv = 0;
    for (int iy = 0; iy < nvy; iy++) {
        double y = iy * m_delta - 0.5 * sizeY;
        for (int ix = 0; ix < nvx; ix++) {
            double x = ix * m_delta - 0.5 * sizeX;
            // Set vertex location
            vertices[iv] = m_plane * ChVector<>(x, y, m_heights(ix, iy));
            // Initialize vertex normal to Y up
            normals[iv] = m_plane.TransformDirectionLocalToParent(ChVector<>(0, 0, 1));
            // Assign color white to all vertices
            colors[iv] = ChVector<float>(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            uv_coords[iv] = ChVector<>(ix * x_scale, iy * y_scale, 0.0);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time).
    // Specify the face vertices counter-clockwise.
    // Set the normal indices same as the vertex indices.
    int it = 0;
    for (int iy = 0; iy < nvy - 1; iy++) {
        for (int ix = 0; ix < nvx - 1; ix++) {
            int v0 = ix + nvx * iy;
            idx_vertices[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            idx_normals[it] = ChVector<int>(v0, v0 + 1, v0 + nvx + 1);
            ++it;
            idx_vertices[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            idx_normals[it] = ChVector<int>(v0, v0 + nvx + 1, v0 + nvx);
            ++it;
        }
    }

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Calculate normals and then average the normals from all adjacent faces.
    for (int it = 0; it < n_faces; it++) {
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
    for (int in = 0; in < n_verts; in++) {
        normals[in] /= (double)accumulators[in];
    }
}

void SCMDeformableSoil::SetupInitial() {
    // If no user-specified moving patches, create one that will encompass all collision shapes in the system
    if (!m_moving_patch) {
        SCMDeformableSoil::MovingPatchInfo pinfo;
        pinfo.m_body = nullptr;
        m_patches.push_back(pinfo);
    }
}

// Get index of trimesh vertex corresponding to the specified grid vertex.
int SCMDeformableSoil::GetMeshVertexIndex(const ChVector2<int>& loc) {
    assert(loc.x() >= -m_nx);
    assert(loc.x() <= +m_nx);
    assert(loc.y() >= -m_ny);
    assert(loc.y() <= +m_ny);
    return (loc.x() + m_nx) + (2 * m_nx + 1) * (loc.y() + m_ny);
}

// Get indices of trimesh faces incident to the specified grid vertex.
std::vector<int> SCMDeformableSoil::GetMeshFaceIndices(const ChVector2<int>& loc) {
    int i = loc.x();
    int j = loc.y();

    // Ignore boundary vertices
    if (i == -m_nx || i == m_nx || j == -m_ny || j == m_ny)
        return std::vector<int>();

    // Load indices of 6 adjacent faces
    i += m_nx;
    j += m_ny;
    int nx = 2 * m_nx;
    std::vector<int> faces(6);
    faces[0] = 2 * ((i - 1) + nx * (j - 1));
    faces[1] = 2 * ((i - 1) + nx * (j - 1)) + 1;
    faces[2] = 2 * ((i - 1) + nx * (j - 0));
    faces[3] = 2 * ((i - 0) + nx * (j - 0));
    faces[4] = 2 * ((i - 0) + nx * (j - 0)) + 1;
    faces[5] = 2 * ((i - 0) + nx * (j - 1)) + 1;

    return faces;
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
            assert(loc.x() >= -m_nx && loc.x() <= m_nx);
            assert(loc.y() >= -m_ny && loc.y() <= m_ny);
            return m_heights(loc.x() + m_nx, loc.y() + m_ny);
        default:
            return 0;
    }
}

// Return the terrain height at the specified location
double SCMDeformableSoil::GetHeight(const ChVector<>& loc) const {
    //// TODO
    return 0;
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
        ChVector<> c_scm = m_plane.TransformPointParentToLocal(c_abs);

        // Update AABB of patch projection onto SCM plane
        p_min.x() = std::min(p_min.x(), c_scm.x());
        p_min.y() = std::min(p_min.y(), c_scm.y());
        p_max.x() = std::max(p_max.x(), c_scm.x());
        p_max.y() = std::max(p_max.y(), c_scm.y());
    }

    // Find index ranges for grid vertices contained in the patch projection AABB
    p.m_bl.x() = ChClamp(static_cast<int>(std::ceil(p_min.x() / m_delta)), -m_nx, +m_nx);
    p.m_bl.y() = ChClamp(static_cast<int>(std::ceil(p_min.y() / m_delta)), -m_ny, +m_ny);
    p.m_tr.x() = ChClamp(static_cast<int>(std::floor(p_max.x() / m_delta)), -m_nx, +m_nx);
    p.m_tr.y() = ChClamp(static_cast<int>(std::floor(p_max.y() / m_delta)), -m_ny, +m_ny);
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
        ChVector<> c_scm = m_plane.TransformPointParentToLocal(c_abs);

        // Update AABB of patch projection onto SCM plane
        p_min.x() = std::min(p_min.x(), c_scm.x());
        p_min.y() = std::min(p_min.y(), c_scm.y());
        p_max.x() = std::max(p_max.x(), c_scm.x());
        p_max.y() = std::max(p_max.y(), c_scm.y());
    }

    // Find index ranges for grid vertices contained in the patch projection AABB
    p.m_bl.x() = ChClamp(static_cast<int>(std::ceil(p_min.x() / m_delta)), -m_nx, +m_nx);
    p.m_bl.y() = ChClamp(static_cast<int>(std::ceil(p_min.y() / m_delta)), -m_ny, +m_ny);
    p.m_tr.x() = ChClamp(static_cast<int>(std::floor(p_max.x() / m_delta)), -m_nx, +m_nx);
    p.m_tr.y() = ChClamp(static_cast<int>(std::floor(p_max.y() / m_delta)), -m_ny, +m_ny);
}

// Offsets for the 8 neighbors of a grid vertex
static const std::vector<ChVector2<int>> neighbors8{
    ChVector2<int>(-1, -1),  // SW
    ChVector2<int>(0, -1),   // S
    ChVector2<int>(1, -1),   // SE
    ChVector2<int>(-1, 0),   // W
    ChVector2<int>(1, 0),    // E
    ChVector2<int>(-1, 1),   // NW
    ChVector2<int>(0, 1),    // N
    ChVector2<int>(1, 1)     // NE
};

static const std::vector<ChVector2<int>> neighbors4{
    ChVector2<int>(0, -1),  // S
    ChVector2<int>(-1, 0),  // W
    ChVector2<int>(1, 0),   // E
    ChVector2<int>(0, 1)    // N
};

// Reset the list of forces, and fills it with forces from a soil contact model.
void SCMDeformableSoil::ComputeInternalForces() {
    m_timer_ray_casting.reset();
    m_timer_contact_patches.reset();
    m_timer_contact_forces.reset();
    m_timer_bulldozing.reset();
    m_timer_visualization.reset();

    // Reset the load list and map of contact forces
    this->GetLoadList().clear();
    m_contact_forces.clear();

    // Express SCM plane normal in absolute frame
    ChVector<> N = m_plane.TransformDirectionLocalToParent(ChVector<>(0, 0, 1));

    // -------------------------
    // Perform ray casting tests
    // -------------------------

    m_timer_ray_casting.start();
    m_num_ray_casts = 0;
    m_num_ray_hits = 0;
    m_hits.clear();

    // Update moving patch information
    if (m_moving_patch) {
        for (auto& p : m_patches)
            UpdateMovingPatch(p);
    } else {
        assert(m_patches.size() == 1);
        UpdateFixedPatch(m_patches[0]);
    }

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
                ChVector<> vertex_abs = m_plane.TransformPointLocalToParent(ChVector<>(x, y, z));

                // Raycast to see if we need to work on this point later
                collision::ChCollisionSystem::ChRayhitResult mrayhit_result;
                ChVector<> to = vertex_abs + N * m_test_offset_up;
                ChVector<> from = to - N * m_test_offset_down;
                GetSystem()->GetCollisionSystem()->RayHit(from, to, mrayhit_result);
                m_num_ray_casts++;

                if (mrayhit_result.hit) {
                    // If this point has never been hit before, initialize it
                    if (m_grid_map.find(ij) == m_grid_map.end()) {
                        m_grid_map.insert(std::make_pair(ij, VertexRecord(z, z)));
                    }

                    // Add to our map of hits to process
                    HitRecord record = {mrayhit_result.hitModel->GetContactable(), mrayhit_result.abs_hitPoint, -1};
                    m_hits.insert(std::make_pair(ij, record));
                    m_num_ray_hits++;
                }
            }
        }
    }

    m_timer_ray_casting.stop();

    // --------------------
    // Find contact patches
    // --------------------

    m_timer_contact_patches.start();

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
    m_num_contact_patches = 0;
    for (auto& h : m_hits) {
        if (h.second.patch_id != -1)
            continue;

        ChVector2<int> ij = h.first;

        // Make a new contact patch and add this hit vertex to it
        h.second.patch_id = m_num_contact_patches++;
        ContactPatchRecord patch;
        patch.points.push_back(ChVector2<>(m_delta * ij.x(), m_delta * ij.y()));

        // Add current vertex to the work queue
        std::queue<ChVector2<int>> todo;
        todo.push(ij);

        while (!todo.empty()) {
            auto crt = m_hits.find(todo.front());  // Current hit vertex is first element in queue
            todo.pop();                            // Remove first element from queue

            ChVector2<int> crt_ij = crt->first;
            int crt_patch = crt->second.patch_id;

            // Loop through the neighbors of the current hit vertex
            for (int k = 0; k < 4; k++) {
                ChVector2<int> nbr_ij = crt_ij + neighbors4[k];
                // If neighbor is not a hit vertex, move on
                auto nbr = m_hits.find(nbr_ij);
                if (nbr == m_hits.end())
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

    m_timer_contact_patches.stop();

    // ----------------------
    // Compute contact forces
    // ----------------------

    m_timer_contact_forces.start();

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
    for (auto& h : m_hits) {
        ChVector2<> ij = h.first;

        auto& v = m_grid_map.at(ij);

        ChContactable* contactable = h.second.contactable;
        const ChVector<>& abs_point = h.second.abs_point;
        int patch_id = h.second.patch_id;

        auto loc_point = m_plane.TransformPointParentToLocal(abs_point);

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
            m_plane.TransformPointLocalToParent(ChVector<>(ij.x() * m_delta, ij.y() * m_delta, v.p_level));

        v.p_speeds = contactable->GetContactPointSpeed(vertex);

        ChVector<> T = -v.p_speeds;
        T = m_plane.TransformDirectionParentToLocal(T);
        double Vn = -T.z();
        T.z() = 0;
        T = m_plane.TransformDirectionLocalToParent(T);
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

            // Update grid vertex height (in local SCM frame)
            v.p_level = v.p_level_initial - v.p_sinkage;

        }  // end positive contact force

    }  // end loop on ray hits

    m_timer_contact_forces.stop();

    // --------------------------------------------------
    // Flow material to the side of rut, using heuristics
    // --------------------------------------------------

    m_timer_bulldozing.start();

    if (do_bulldozing) {
        //// TODO
    }

    m_timer_bulldozing.stop();

    // --------------------
    // Update visualization
    // --------------------

    m_timer_visualization.start();

    if (m_trimesh_shape) {
        // Indices of modified vertices (initialize with any externally modified)
        std::vector<int> modified_vertices = m_external_modified_vertices;

        // Loop over list of hits and adjust corresponding mesh vertices
        for (const auto& h : m_hits) {
            auto ij = h.first;                       // grid location
            auto v = m_grid_map.at(ij);              // grid vertex record
            int iv = GetMeshVertexIndex(ij);         // mesh vertex index
            UpdateMeshVertexCoordinates(ij, iv, v);  // update vertex coordinates and color
            modified_vertices.push_back(iv);
        }

        // Update the visualization normals for modified vertices
        if (!m_trimesh_shape->IsWireframe()) {
            for (const auto& h : m_hits) {
                auto ij = h.first;                // grid location
                int iv = GetMeshVertexIndex(ij);  // mesh vertex index
                UpdateMeshVertexNormal(ij, iv);   // update vertex normal
            }
        }

        m_trimesh_shape->SetModifiedVertices(modified_vertices);
        m_external_modified_vertices.clear();
    }

    m_timer_visualization.stop();
}

// Update vertex position and color in visualization mesh
void SCMDeformableSoil::UpdateMeshVertexCoordinates(const ChVector2<int> ij, int iv, const VertexRecord& v) {
    auto& trimesh = *m_trimesh_shape->GetMesh();
    std::vector<ChVector<>>& vertices = trimesh.getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh.getCoordsNormals();
    std::vector<ChVector<float>>& colors = trimesh.getCoordsColors();

    // Update visualization mesh vertex position
    vertices[iv] = m_plane.TransformPointLocalToParent(ChVector<>(ij.x() * m_delta, ij.y() * m_delta, v.p_level));

    // Update visualization mesh vertex color
    if (m_plot_type != SCMDeformableTerrain::PLOT_NONE) {
        ChColor mcolor;
        switch (m_plot_type) {
            case SCMDeformableTerrain::PLOT_LEVEL:
                mcolor = ChColor::ComputeFalseColor(v.p_level, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_LEVEL_INITIAL:
                mcolor = ChColor::ComputeFalseColor(v.p_level_initial, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SINKAGE:
                mcolor = ChColor::ComputeFalseColor(v.p_sinkage, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SINKAGE_ELASTIC:
                mcolor = ChColor::ComputeFalseColor(v.p_sinkage_elastic, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SINKAGE_PLASTIC:
                mcolor = ChColor::ComputeFalseColor(v.p_sinkage_plastic, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_STEP_PLASTIC_FLOW:
                mcolor = ChColor::ComputeFalseColor(v.p_step_plastic_flow, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_K_JANOSI:
                mcolor = ChColor::ComputeFalseColor(v.p_kshear, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_PRESSURE:
                mcolor = ChColor::ComputeFalseColor(v.p_sigma, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_PRESSURE_YELD:
                mcolor = ChColor::ComputeFalseColor(v.p_sigma_yield, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SHEAR:
                mcolor = ChColor::ComputeFalseColor(v.p_tau, m_plot_v_min, m_plot_v_max);
                break;
            ////case SCMDeformableTerrain::PLOT_MASSREMAINDER:
            ////    mcolor = ChColor::ComputeFalseColor(v.p_massremainder, m_plot_v_min, m_plot_v_max);
            ////    break;
            ////case SCMDeformableTerrain::PLOT_ISLAND_ID:
            ////    mcolor = ChColor(0, 0, 1);
            ////    if (v.p_erosion == true)
            ////        mcolor = ChColor(1, 1, 1);
            ////    if (v.p_id_island > 0)
            ////        mcolor = ChColor::ComputeFalseColor(4.0 + (v.p_id_island % 8), 0.0, 12.0);
            ////    if (v.p_id_island < 0)
            ////        mcolor = ChColor(0, 0, 0);
            ////    break;
            case SCMDeformableTerrain::PLOT_IS_TOUCHED:
                if (v.p_sigma > 0)
                    mcolor = ChColor(1, 0, 0);
                else
                    mcolor = ChColor(0, 0, 1);
                break;
        }
        colors[iv] = {mcolor.R, mcolor.G, mcolor.B};
    }
}

// Update vertex normal in visualization mesh
void SCMDeformableSoil::UpdateMeshVertexNormal(const ChVector2<int> ij, int iv) {
    auto& trimesh = *m_trimesh_shape->GetMesh();
    std::vector<ChVector<>>& vertices = trimesh.getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh.getCoordsNormals();
    std::vector<ChVector<int>>& idx_normals = trimesh.getIndicesNormals();

    // Average normals from adjacent faces
    normals[iv] = ChVector<>(0, 0, 0);
    auto faces = GetMeshFaceIndices(ij);
    for (auto f : faces) {
        ChVector<> nrm = Vcross(vertices[idx_normals[f][1]] - vertices[idx_normals[f][0]],
                                vertices[idx_normals[f][2]] - vertices[idx_normals[f][0]]);
        nrm.Normalize();
        normals[iv] += nrm;
    }
    normals[iv] /= (double)faces.size();
}

// Get the grid vertices and their heights that were modified over last step.
std::vector<SCMDeformableTerrain::VertexLevel> SCMDeformableSoil::GetModifiedVertices() const {
    std::vector<SCMDeformableTerrain::VertexLevel> vertices;
    for (const auto& h : m_hits) {
        auto p = m_grid_map.find(h.first);
        assert(p != m_grid_map.end());
        vertices.push_back(std::make_pair(h.first, p->second.p_level));
    }
    return vertices;
}

// Modify the level of vertices in the underlying grid map from the given list.
void SCMDeformableSoil::SetModifiedVertices(const std::vector<SCMDeformableTerrain::VertexLevel>& vertices) {
    for (const auto& v : vertices) {
        auto ij = v.first;
        // Modify existing entry in grid map or insert new one
        double init_level = GetHeight(ij);
        m_grid_map[ij] = SCMDeformableSoil::VertexRecord(init_level, v.second);
    }

    // Update visualization
    if (m_trimesh_shape) {
        for (const auto& v : vertices) {
            auto ij = v.first;                        // grid location
            auto vr = m_grid_map.at(ij);              // grid vertex record
            int iv = GetMeshVertexIndex(ij);          // mesh vertex index
            UpdateMeshVertexCoordinates(ij, iv, vr);  // update vertex coordinates and color
            m_external_modified_vertices.push_back(iv);
        }
        if (!m_trimesh_shape->IsWireframe()) {
            for (const auto& v : vertices) {
                auto ij = v.first;                // grid location
                int iv = GetMeshVertexIndex(ij);  // mesh vertex index
                UpdateMeshVertexNormal(ij, iv);   // update vertex normal
            }
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
