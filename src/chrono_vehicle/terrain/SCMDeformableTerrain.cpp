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
#include <unordered_set>
#include <limits>

#ifdef _OPENMP
    #include <omp.h>
#endif

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

// Get the initial terrain height below the specified location.
double SCMDeformableTerrain::GetInitHeight(const ChVector<>& loc) const {
    return m_ground->GetInitHeight(loc);
}

// Get the initial terrain normal at the point below the specified location.
ChVector<> SCMDeformableTerrain::GetInitNormal(const ChVector<>& loc) const {
    return m_ground->GetInitNormal(loc);
}

// Get the terrain height below the specified location.
double SCMDeformableTerrain::GetHeight(const ChVector<>& loc) const {
    return m_ground->GetHeight(loc);
}

// Get the terrain normal at the point below the specified location.
ChVector<> SCMDeformableTerrain::GetNormal(const ChVector<>& loc) const {
    return m_ground->GetNormal(loc);
}

// Return the terrain coefficient of friction at the specified location.
float SCMDeformableTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return m_friction_fun ? (*m_friction_fun)(loc) : 0.8f;
}

// Get SCM information at the node closest to the specified location.
SCMDeformableTerrain::NodeInfo SCMDeformableTerrain::GetNodeInfo(const ChVector<>& loc) const {
    return m_ground->GetNodeInfo(loc);
}

// Set the color of the visualization assets.
void SCMDeformableTerrain::SetColor(const ChColor& color) {
    if (m_ground->GetVisualModel()) {
        m_ground->GetVisualShape(0)->SetColor(color);
    }
}

// Set the texture and texture scaling.
void SCMDeformableTerrain::SetTexture(const std::string tex_file, float scale_x, float scale_y) {
    if (m_ground->GetVisualModel()) {
        m_ground->GetVisualShape(0)->SetTexture(tex_file, scale_x, scale_y);
    }
}

// Set the SCM reference plane.
void SCMDeformableTerrain::SetPlane(const ChCoordsys<>& plane) {
    m_ground->m_plane = plane;
    m_ground->m_Z = plane.rot.GetZaxis();
}

// Get the SCM reference plane.
const ChCoordsys<>& SCMDeformableTerrain::GetPlane() const {
    return m_ground->m_plane;
}

// Set the visualization mesh as wireframe or as solid.
void SCMDeformableTerrain::SetMeshWireframe(bool val) {
    if (m_ground->m_trimesh_shape)
        m_ground->m_trimesh_shape->SetWireframe(val);
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

// Set properties of the SCM soil model.
void SCMDeformableTerrain::SetSoilParameters(
    double Bekker_Kphi,    // Kphi, frictional modulus in Bekker model
    double Bekker_Kc,      // Kc, cohesive modulus in Bekker model
    double Bekker_n,       // n, exponent of sinkage in Bekker model (usually 0.6...1.8)
    double Mohr_cohesion,  // Cohesion for shear failure [Pa]
    double Mohr_friction,  // Friction angle for shear failure [degree]
    double Janosi_shear,   // Shear parameter in Janosi-Hanamoto formula [m]
    double elastic_K,      // elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
    double damping_R       // vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)
) {
    m_ground->m_Bekker_Kphi = Bekker_Kphi;
    m_ground->m_Bekker_Kc = Bekker_Kc;
    m_ground->m_Bekker_n = Bekker_n;
    m_ground->m_Mohr_cohesion = Mohr_cohesion;
    m_ground->m_Mohr_mu = std::tan(Mohr_friction * CH_C_DEG_TO_RAD);
    m_ground->m_Janosi_shear = Janosi_shear;
    m_ground->m_elastic_K = ChMax(elastic_K, Bekker_Kphi);
    m_ground->m_damping_R = damping_R;
}

// Enable/disable bulldozing effect.
void SCMDeformableTerrain::EnableBulldozing(bool val) {
    m_ground->m_bulldozing = val;
}

// Set parameters controlling the creation of side ruts (bulldozing effects).
void SCMDeformableTerrain::SetBulldozingParameters(
    double erosion_angle,     // angle of erosion of the displaced material [degrees]
    double flow_factor,       // growth of lateral volume relative to pressed volume
    int erosion_iterations,   // number of erosion refinements per timestep
    int erosion_propagations  // number of concentric vertex selections subject to erosion
) {
    m_ground->m_flow_factor = flow_factor;
    m_ground->m_erosion_slope = std::tan(erosion_angle * CH_C_DEG_TO_RAD);
    m_ground->m_erosion_iterations = erosion_iterations;
    m_ground->m_erosion_propagations = erosion_propagations;
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

// Enable moving patch.
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

// Set user-supplied callback for evaluating location-dependent soil parameters.
void SCMDeformableTerrain::RegisterSoilParametersCallback(std::shared_ptr<SoilParametersCallback> cb) {
    m_ground->m_soil_fun = cb;
}

// Initialize the terrain as a flat grid.
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

// Initialize the terrain from a specified OBJ mesh file.
void SCMDeformableTerrain::Initialize(const std::string& mesh_file, double delta) {
    m_ground->Initialize(mesh_file, delta);
}

// Get the heights of modified grid nodes.
std::vector<SCMDeformableTerrain::NodeLevel> SCMDeformableTerrain::GetModifiedNodes(bool all_nodes) const {
    return m_ground->GetModifiedNodes(all_nodes);
}

// Modify the level of grid nodes from the given list.
void SCMDeformableTerrain::SetModifiedNodes(const std::vector<NodeLevel>& nodes) {
    m_ground->SetModifiedNodes(nodes);
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

// Return the number of rays cast at last step.
int SCMDeformableTerrain::GetNumRayCasts() const {
    return m_ground->m_num_ray_casts;
}

// Return the number of ray hits at last step.
int SCMDeformableTerrain::GetNumRayHits() const {
    return m_ground->m_num_ray_hits;
}

// Return the number of contact patches at last step.
int SCMDeformableTerrain::GetNumContactPatches() const {
    return m_ground->m_num_contact_patches;
}

// Return the number of nodes in the erosion domain at last step (bulldosing effects).
int SCMDeformableTerrain::GetNumErosionNodes() const {
    return m_ground->m_num_erosion_nodes;
}

// Timer information
double SCMDeformableTerrain::GetTimerMovingPatches() const {
    return 1e3 * m_ground->m_timer_moving_patches();
}
double SCMDeformableTerrain::GetTimerRayTesting() const {
    return 1e3 * m_ground->m_timer_ray_testing();
}
double SCMDeformableTerrain::GetTimerRayCasting() const {
    return 1e3 * m_ground->m_timer_ray_casting();
}
double SCMDeformableTerrain::GetTimerContactPatches() const {
    return 1e3 * m_ground->m_timer_contact_patches();
}
double SCMDeformableTerrain::GetTimerContactForces() const {
    return 1e3 * m_ground->m_timer_contact_forces();
}
double SCMDeformableTerrain::GetTimerBulldozing() const {
    return 1e3 * m_ground->m_timer_bulldozing();
}
double SCMDeformableTerrain::GetTimerVisUpdate() const {
    return 1e3 * m_ground->m_timer_visualization();
}

// Print timing and counter information for last step.
void SCMDeformableTerrain::PrintStepStatistics(std::ostream& os) const {
    os << " Timers (ms):" << std::endl;
    os << "   Moving patches:          " << 1e3 * m_ground->m_timer_moving_patches() << std::endl;
    os << "   Ray testing:             " << 1e3 * m_ground->m_timer_ray_testing() << std::endl;
    os << "   Ray casting:             " << 1e3 * m_ground->m_timer_ray_casting() << std::endl;
    os << "   Contact patches:         " << 1e3 * m_ground->m_timer_contact_patches() << std::endl;
    os << "   Contact forces:          " << 1e3 * m_ground->m_timer_contact_forces() << std::endl;
    os << "   Bulldozing:              " << 1e3 * m_ground->m_timer_bulldozing() << std::endl;
    os << "      Raise boundary:       " << 1e3 * m_ground->m_timer_bulldozing_boundary() << std::endl;
    os << "      Compute domain:       " << 1e3 * m_ground->m_timer_bulldozing_domain() << std::endl;
    os << "      Apply erosion:        " << 1e3 * m_ground->m_timer_bulldozing_erosion() << std::endl;
    os << "   Visualization:           " << 1e3 * m_ground->m_timer_visualization() << std::endl;

    os << " Counters:" << std::endl;
    os << "   Number ray casts:        " << m_ground->m_num_ray_casts << std::endl;
    os << "   Number ray hits:         " << m_ground->m_num_ray_hits << std::endl;
    os << "   Number contact patches:  " << m_ground->m_num_contact_patches << std::endl;
    os << "   Number erosion nodes:    " << m_ground->m_num_erosion_nodes << std::endl;
}

// -----------------------------------------------------------------------------
// Contactable user-data (contactable-soil parameters)
// -----------------------------------------------------------------------------

SCMContactableData::SCMContactableData(double area_ratio,
                                       double Mohr_cohesion,
                                       double Mohr_friction,
                                       double Janosi_shear)
    : area_ratio(area_ratio),
      Mohr_cohesion(Mohr_cohesion),
      Mohr_mu(std::tan(Mohr_friction * CH_C_DEG_TO_RAD)),
      Janosi_shear(Janosi_shear) {}

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
    }

    // Default SCM plane and plane normal
    m_plane = ChCoordsys<>(VNULL, QUNIT);
    m_Z = m_plane.rot.GetZaxis();

    // Bulldozing effects
    m_bulldozing = false;
    m_flow_factor = 1.2;
    m_erosion_slope = std::tan(40.0 * CH_C_DEG_TO_RAD);
    m_erosion_iterations = 3;
    m_erosion_propagations = 10;

    // Default soil parameters
    m_Bekker_Kphi = 2e6;
    m_Bekker_Kc = 0;
    m_Bekker_n = 1.1;
    m_Mohr_cohesion = 50;
    m_Mohr_mu = std::tan(20.0 * CH_C_DEG_TO_RAD);
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
    m_type = PatchType::FLAT;

    m_nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    m_ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction

    m_delta = sizeX / (2 * m_nx);   // grid spacing
    m_area = std::pow(m_delta, 2);  // area of a cell

    // Return now if no visualization
    if (!m_trimesh_shape)
        return;

    CreateVisualizationMesh(sizeX, sizeY);
    this->AddVisualShape(m_trimesh_shape);
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
        std::cout << "STB error in reading height map file " << heightmap_file << std::endl;
        throw ChException("Cannot read height map image file");
    }
    int nx_img = hmap.GetWidth();
    int ny_img = hmap.GetHeight();

    double dx_img = 1.0 / (nx_img - 1.0);
    double dy_img = 1.0 / (ny_img - 1.0);

    m_nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    m_ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction
    int nvx = 2 * m_nx + 1;                                   // number of grid vertices in X direction
    int nvy = 2 * m_ny + 1;                                   // number of grid vertices in Y direction
    m_delta = sizeX / (2.0 * m_nx);                           // grid spacing
    m_area = std::pow(m_delta, 2);                            // area of a cell

    double dx_grid = 0.5 / m_nx;
    double dy_grid = 0.5 / m_ny;

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

    CreateVisualizationMesh(sizeX, sizeY);
    this->AddVisualShape(m_trimesh_shape);
}

// Initialize the terrain from a specified OBJ mesh file.
bool calcBarycentricCoordinates(const ChVector<>& v1,
                                const ChVector<>& v2,
                                const ChVector<>& v3,
                                const ChVector<>& v,
                                double& a1,
                                double& a2,
                                double& a3) {
    double denom = (v2.y() - v3.y()) * (v1.x() - v3.x()) + (v3.x() - v2.x()) * (v1.y() - v3.y());
    a1 = ((v2.y() - v3.y()) * (v.x() - v3.x()) + (v3.x() - v2.x()) * (v.y() - v3.y())) / denom;
    a2 = ((v3.y() - v1.y()) * (v.x() - v3.x()) + (v1.x() - v3.x()) * (v.y() - v3.y())) / denom;
    a3 = 1 - a1 - a2;

    return (0 <= a1) && (a1 <= 1) && (0 <= a2) && (a2 <= 1) && (0 <= a3) && (a3 <= 1);
}

void SCMDeformableSoil::Initialize(const std::string& mesh_file, double delta) {
    m_type = PatchType::TRI_MESH;

    // Load triangular mesh
    auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_file, true, true);
    const auto& vertices = trimesh->getCoordsVertices();
    const auto& faces = trimesh->getIndicesVertexes();

    // Find x, y, and z ranges of vertex data
    auto minmaxX = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector<>& v1, const ChVector<>& v2) { return v1.x() < v2.x(); });
    auto minmaxY = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector<>& v1, const ChVector<>& v2) { return v1.y() < v2.y(); });
    auto minmaxZ = std::minmax_element(begin(vertices), end(vertices),
                                       [](const ChVector<>& v1, const ChVector<>& v2) { return v1.z() < v2.z(); });
    auto minX = minmaxX.first->x() + delta;
    auto maxX = minmaxX.second->x() - delta;
    auto minY = minmaxY.first->y() + delta;
    auto maxY = minmaxY.second->y() - delta;
    auto minZ = minmaxZ.first->z();
    auto maxZ = minmaxZ.second->z();

    auto sizeX = (maxX - minX);
    auto sizeY = (maxY - minY);
    ChVector<> center((maxX + minX) / 2, (maxY + minY) / 2, 0);

    // Initial grid extent
    m_nx = static_cast<int>(std::ceil((sizeX / 2) / delta));  // half number of divisions in X direction
    m_ny = static_cast<int>(std::ceil((sizeY / 2) / delta));  // number of divisions in Y direction
    m_delta = sizeX / (2.0 * m_nx);                           // grid spacing
    m_area = std::pow(m_delta, 2);                            // area of a cell
    int nvx = 2 * m_nx + 1;                                   // number of grid vertices in X direction
    int nvy = 2 * m_ny + 1;                                   // number of grid vertices in Y direction

    // Loop over all mesh faces, project onto the x-y plane and set the height for all covered grid nodes.
    m_heights = ChMatrixDynamic<>::Zero(nvx, nvy);

    int num_h_set = 0;
    double a1, a2, a3;
    for (const auto& f : faces) {
        // Find bounds of (shifted) face projection
        const auto& v1 = vertices[f[0]] - center;
        const auto& v2 = vertices[f[1]] - center;
        const auto& v3 = vertices[f[2]] - center;
        auto x_min = ChMin(ChMin(v1.x(), v2.x()), v3.x());
        auto x_max = ChMax(ChMax(v1.x(), v2.x()), v3.x());
        auto y_min = ChMin(ChMin(v1.y(), v2.y()), v3.y());
        auto y_max = ChMax(ChMax(v1.y(), v2.y()), v3.y());
        int i_min = static_cast<int>(std::floor(x_min / m_delta));
        int j_min = static_cast<int>(std::floor(y_min / m_delta));
        int i_max = static_cast<int>(std::ceil(x_max / m_delta));
        int j_max = static_cast<int>(std::ceil(y_max / m_delta));
        ChClampValue(i_min, -m_nx, +m_nx);
        ChClampValue(i_max, -m_nx, +m_nx);
        ChClampValue(j_min, -m_ny, +m_ny);
        ChClampValue(j_max, -m_ny, +m_ny);
        // Loop over all grid nodes within bounds
        for (int i = i_min; i <= i_max; i++) {
            for (int j = j_min; j <= j_max; j++) {
                ChVector<> v(i * m_delta, j * m_delta, 0);
                if (calcBarycentricCoordinates(v1, v2, v3, v, a1, a2, a3)) {
                    m_heights(m_nx + i, m_ny + j) = minZ + a1 * v1.z() + a2 * v2.z() + a3 * v3.z();
                    num_h_set++;
                }
            }
        }
    }

    // Return now if no visualization
    if (!m_trimesh_shape)
        return;

    CreateVisualizationMesh(sizeX, sizeY);
    this->AddVisualShape(m_trimesh_shape);
}

void SCMDeformableSoil::CreateVisualizationMesh(double sizeX, double sizeY) {
    int nvx = 2 * m_nx + 1;                     // number of grid vertices in X direction
    int nvy = 2 * m_ny + 1;                     // number of grid vertices in Y direction
    int n_verts = nvx * nvy;                    // total number of vertices for initial visualization trimesh
    int n_faces = 2 * (2 * m_nx) * (2 * m_ny);  // total number of faces for initial visualization trimesh
    double x_scale = 0.5 / m_nx;                // scale for texture coordinates (U direction)
    double y_scale = 0.5 / m_ny;                // scale for texture coordinates (V direction)

    // Readability aliases
    auto trimesh = m_trimesh_shape->GetMesh();
    trimesh->Clear();
    std::vector<ChVector<>>& vertices = trimesh->getCoordsVertices();
    std::vector<ChVector<>>& normals = trimesh->getCoordsNormals();
    std::vector<ChVector<int>>& idx_vertices = trimesh->getIndicesVertexes();
    std::vector<ChVector<int>>& idx_normals = trimesh->getIndicesNormals();
    std::vector<ChVector2<>>& uv_coords = trimesh->getCoordsUV();
    std::vector<ChColor>& colors = trimesh->getCoordsColors();

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
            if (m_type == PatchType::FLAT) {
                // Set vertex location
                vertices[iv] = m_plane * ChVector<>(x, y, 0);
                // Initialize vertex normal to Z up
                normals[iv] = m_plane.TransformDirectionLocalToParent(ChVector<>(0, 0, 1));
            } else {
                // Set vertex location
                vertices[iv] = m_plane * ChVector<>(x, y, m_heights(ix, iy));
                // Initialize vertex normal to zero (will be set later)
                normals[iv] = ChVector<>(0, 0, 0);
            }
            // Assign color white to all vertices
            colors[iv] = ChColor(1, 1, 1);
            // Set UV coordinates in [0,1] x [0,1]
            uv_coords[iv] = ChVector2<>(ix * x_scale, iy * y_scale);
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

    if (m_type == PatchType::FLAT)
        return;

    // Initialize the array of accumulators (number of adjacent faces to a vertex)
    std::vector<int> accumulators(n_verts, 0);

    // Calculate normals and then average the normals from all adjacent faces.
    for (it = 0; it < n_faces; it++) {
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

bool SCMDeformableSoil::CheckMeshBounds(const ChVector2<int>& loc) const {
    return loc.x() >= -m_nx && loc.x() <= m_nx && loc.y() >= -m_ny && loc.y() <= m_ny;
}

SCMDeformableTerrain::NodeInfo SCMDeformableSoil::GetNodeInfo(const ChVector<>& loc) const {
    SCMDeformableTerrain::NodeInfo ni;

    // Express location in the SCM frame
    ChVector<> loc_loc = m_plane.TransformPointParentToLocal(loc);

    // Find closest grid vertex (approximation)
    int i = static_cast<int>(std::round(loc_loc.x() / m_delta));
    int j = static_cast<int>(std::round(loc_loc.y() / m_delta));
    ChVector2<int> ij(i, j);

    // First query the hash-map
    auto p = m_grid_map.find(ij);
    if (p != m_grid_map.end()) {
        ni.sinkage = p->second.sinkage;
        ni.sinkage_plastic = p->second.sinkage_plastic;
        ni.sinkage_elastic = p->second.sinkage_elastic;
        ni.sigma = p->second.sigma;
        ni.sigma_yield = p->second.sigma_yield;
        ni.kshear = p->second.kshear;
        ni.tau = p->second.tau;
        return ni;
    }

    // Return a default node record
    ni.sinkage = 0;
    ni.sinkage_plastic = 0;
    ni.sinkage_elastic = 0;
    ni.sigma = 0;
    ni.sigma_yield = 0;
    ni.kshear = 0;
    ni.tau = 0;
    return ni;
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

// Get the initial undeformed terrain height (relative to the SCM plane) at the specified grid vertex.
double SCMDeformableSoil::GetInitHeight(const ChVector2<int>& loc) const {
    switch (m_type) {
        case PatchType::FLAT:
            return 0;
        case PatchType::HEIGHT_MAP:
        case PatchType::TRI_MESH: {
            auto x = ChClamp(loc.x(), -m_nx, +m_nx);
            auto y = ChClamp(loc.y(), -m_ny, +m_ny);
            return m_heights(x + m_nx, y + m_ny);
        }
        default:
            return 0;
    }
}

// Get the initial undeformed terrain normal (relative to the SCM plane) at the specified grid node.
ChVector<> SCMDeformableSoil::GetInitNormal(const ChVector2<int>& loc) const {
    switch (m_type) {
        case PatchType::HEIGHT_MAP:
        case PatchType::TRI_MESH: {
            // Average normals of 4 triangular faces incident to given grid node
            auto hE = GetInitHeight(loc + ChVector2<int>(1, 0));  // east
            auto hW = GetInitHeight(loc - ChVector2<int>(1, 0));  // west
            auto hN = GetInitHeight(loc + ChVector2<int>(0, 1));  // north
            auto hS = GetInitHeight(loc - ChVector2<int>(0, 1));  // south
            return ChVector<>(hW - hE, hS - hN, 2 * m_delta).GetNormalized();
        }
        case PatchType::FLAT:
        default:
            return ChVector<>(0, 0, 1);
    }
}

// Get the terrain height (relative to the SCM plane) at the specified grid vertex.
double SCMDeformableSoil::GetHeight(const ChVector2<int>& loc) const {
    // First query the hash-map
    auto p = m_grid_map.find(loc);
    if (p != m_grid_map.end())
        return p->second.level;

    // Else return undeformed height
    return GetInitHeight(loc);
}

// Get the terrain normal (relative to the SCM plane) at the specified grid vertex.
ChVector<> SCMDeformableSoil::GetNormal(const ChVector2<>& loc) const {
    switch (m_type) {
        case PatchType::HEIGHT_MAP:
        case PatchType::TRI_MESH: {
            // Average normals of 4 triangular faces incident to given grid node
            auto hE = GetHeight(loc + ChVector2<int>(1, 0));  // east
            auto hW = GetHeight(loc - ChVector2<int>(1, 0));  // west
            auto hN = GetHeight(loc + ChVector2<int>(0, 1));  // north
            auto hS = GetHeight(loc - ChVector2<int>(0, 1));  // south
            return ChVector<>(hW - hE, hS - hN, 2 * m_delta).GetNormalized();
        }
        case PatchType::FLAT:
        default:
            return ChVector<>(0, 0, 1);
    }
}

// Get the initial terrain height below the specified location.
double SCMDeformableSoil::GetInitHeight(const ChVector<>& loc) const {
    // Express location in the SCM frame
    ChVector<> loc_loc = m_plane.TransformPointParentToLocal(loc);

    // Get height (relative to SCM plane) at closest grid vertex (approximation)
    int i = static_cast<int>(std::round(loc_loc.x() / m_delta));
    int j = static_cast<int>(std::round(loc_loc.y() / m_delta));
    loc_loc.z() = GetInitHeight(ChVector2<int>(i, j));

    // Express in global frame
    ChVector<> loc_abs = m_plane.TransformPointLocalToParent(loc_loc);
    return ChWorldFrame::Height(loc_abs);
}

// Get the initial terrain normal at the point below the specified location.
ChVector<> SCMDeformableSoil::GetInitNormal(const ChVector<>& loc) const {
    // Express location in the SCM frame
    ChVector<> loc_loc = m_plane.TransformPointParentToLocal(loc);

    // Get height (relative to SCM plane) at closest grid vertex (approximation)
    int i = static_cast<int>(std::round(loc_loc.x() / m_delta));
    int j = static_cast<int>(std::round(loc_loc.y() / m_delta));
    auto nrm_loc = GetInitNormal(ChVector2<int>(i, j));

    // Express in global frame
    auto nrm_abs = m_plane.TransformDirectionLocalToParent(nrm_loc);
    return ChWorldFrame::FromISO(nrm_abs);
}

// Get the terrain height below the specified location.
double SCMDeformableSoil::GetHeight(const ChVector<>& loc) const {
    // Express location in the SCM frame
    ChVector<> loc_loc = m_plane.TransformPointParentToLocal(loc);

    // Get height (relative to SCM plane) at closest grid vertex (approximation)
    int i = static_cast<int>(std::round(loc_loc.x() / m_delta));
    int j = static_cast<int>(std::round(loc_loc.y() / m_delta));
    loc_loc.z() = GetHeight(ChVector2<int>(i, j));

    // Express in global frame
    ChVector<> loc_abs = m_plane.TransformPointLocalToParent(loc_loc);
    return ChWorldFrame::Height(loc_abs);
}

// Get the terrain normal at the point below the specified location.
ChVector<> SCMDeformableSoil::GetNormal(const ChVector<>& loc) const {
    // Express location in the SCM frame
    ChVector<> loc_loc = m_plane.TransformPointParentToLocal(loc);

    // Get height (relative to SCM plane) at closest grid vertex (approximation)
    int i = static_cast<int>(std::round(loc_loc.x() / m_delta));
    int j = static_cast<int>(std::round(loc_loc.y() / m_delta));
    auto nrm_loc = GetNormal(ChVector2<int>(i, j));

    // Express in global frame
    auto nrm_abs = m_plane.TransformDirectionLocalToParent(nrm_loc);
    return ChWorldFrame::FromISO(nrm_abs);
}

// Synchronize information for a moving patch
void SCMDeformableSoil::UpdateMovingPatch(MovingPatchInfo& p, const ChVector<>& Z) {
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
    int x_min = static_cast<int>(std::ceil(p_min.x() / m_delta));
    int y_min = static_cast<int>(std::ceil(p_min.y() / m_delta));
    int x_max = static_cast<int>(std::floor(p_max.x() / m_delta));
    int y_max = static_cast<int>(std::floor(p_max.y() / m_delta));
    int n_x = x_max - x_min + 1;
    int n_y = y_max - y_min + 1;

    p.m_range.resize(n_x * n_y);
    for (int i = 0; i < n_x; i++) {
        for (int j = 0; j < n_y; j++) {
            p.m_range[j * n_x + i] = ChVector2<int>(i + x_min, j + y_min);
        }
    }

    // Calculate inverse of SCM normal expressed in body frame (for optimization of ray-OBB test)
    ChVector<> dir = p.m_body->TransformDirectionParentToLocal(Z);
    p.m_ooN.x() = (dir.x() == 0) ? 1e10 : 1.0 / dir.x();
    p.m_ooN.y() = (dir.y() == 0) ? 1e10 : 1.0 / dir.y();
    p.m_ooN.z() = (dir.z() == 0) ? 1e10 : 1.0 / dir.z();
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
    int x_min = static_cast<int>(std::ceil(p_min.x() / m_delta));
    int y_min = static_cast<int>(std::ceil(p_min.y() / m_delta));
    int x_max = static_cast<int>(std::floor(p_max.x() / m_delta));
    int y_max = static_cast<int>(std::floor(p_max.y() / m_delta));
    int n_x = x_max - x_min + 1;
    int n_y = y_max - y_min + 1;

    p.m_range.resize(n_x * n_y);
    for (int i = 0; i < n_x; i++) {
        for (int j = 0; j < n_y; j++) {
            p.m_range[j * n_x + i] = ChVector2<int>(i + x_min, j + y_min);
        }
    }
}

// Ray-OBB intersection test
bool SCMDeformableSoil::RayOBBtest(const MovingPatchInfo& p, const ChVector<>& from, const ChVector<>& Z) {
    // Express ray origin in OBB frame
    ChVector<> orig = p.m_body->TransformPointParentToLocal(from) - p.m_center;

    // Perform ray-AABB test (slab tests)
    double t1 = (-p.m_hdims.x() - orig.x()) * p.m_ooN.x();
    double t2 = (+p.m_hdims.x() - orig.x()) * p.m_ooN.x();
    double t3 = (-p.m_hdims.y() - orig.y()) * p.m_ooN.y();
    double t4 = (+p.m_hdims.y() - orig.y()) * p.m_ooN.y();
    double t5 = (-p.m_hdims.z() - orig.z()) * p.m_ooN.z();
    double t6 = (+p.m_hdims.z() - orig.z()) * p.m_ooN.z();

    double tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
    double tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

    if (tmax < 0)
        return false;
    if (tmin > tmax)
        return false;
    return true;
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

// Default implementation uses Map-Reduce for collecting ray intersection hits.
// The alternative is to simultaenously load the global map of hits while ray casting (using a critical section).
////#define RAY_CASTING_WITH_CRITICAL_SECTION

// Reset the list of forces, and fills it with forces from a soil contact model.
void SCMDeformableSoil::ComputeInternalForces() {
    // Initialize list of modified visualization mesh vertices (use any externally modified vertices)
    std::vector<int> modified_vertices = m_external_modified_vertices;
    m_external_modified_vertices.clear();

    // Reset quantities at grid nodes modified over previous step
    // (required for bulldozing effects and for proper visualization coloring)
    for (const auto& ij : m_modified_nodes) {
        auto& nr = m_grid_map.at(ij);
        nr.sigma = 0;
        nr.sinkage_elastic = 0;
        nr.step_plastic_flow = 0;
        nr.erosion = false;
        nr.hit_level = 1e9;

        // Update visualization (only color changes relevant here)
        if (m_trimesh_shape && CheckMeshBounds(ij)) {
            int iv = GetMeshVertexIndex(ij);          // mesh vertex index
            UpdateMeshVertexCoordinates(ij, iv, nr);  // update vertex coordinates and color
            modified_vertices.push_back(iv);
        }
    }

    m_modified_nodes.clear();

    // Reset timers
    m_timer_moving_patches.reset();
    m_timer_ray_testing.reset();
    m_timer_ray_casting.reset();
    m_timer_contact_patches.reset();
    m_timer_contact_forces.reset();
    m_timer_bulldozing.reset();
    m_timer_bulldozing_boundary.reset();
    m_timer_bulldozing_domain.reset();
    m_timer_bulldozing_erosion.reset();
    m_timer_visualization.reset();

    // Reset the load list and map of contact forces
    this->GetLoadList().clear();
    m_contact_forces.clear();

    // ---------------------
    // Update moving patches
    // ---------------------

    m_timer_moving_patches.start();

    // Update patch information (find range of grid indices)
    if (m_moving_patch) {
        for (auto& p : m_patches)
            UpdateMovingPatch(p, m_Z);
    } else {
        assert(m_patches.size() == 1);
        UpdateFixedPatch(m_patches[0]);
    }

    m_timer_moving_patches.stop();

    // -------------------------
    // Perform ray casting tests
    // -------------------------

    // Information of vertices with ray-cast hits
    struct HitRecord {
        ChContactable* contactable;  // pointer to hit object
        ChVector<> abs_point;        // hit point, expressed in global frame
        int patch_id;                // index of associated patch id
    };

    // Hash-map for vertices with ray-cast hits
    std::unordered_map<ChVector2<int>, HitRecord, CoordHash> hits;

    m_num_ray_casts = 0;
    m_num_ray_hits = 0;

    m_timer_ray_casting.start();

#ifdef RAY_CASTING_WITH_CRITICAL_SECTION

    int nthreads = GetSystem()->GetNumThreadsChrono();

    // Loop through all moving patches (user-defined or default one)
    for (auto& p : m_patches) {
        // Loop through all vertices in the patch range
        int num_ray_casts = 0;
    #pragma omp parallel for num_threads(nthreads) reduction(+ : num_ray_casts)
        for (int k = 0; k < p.m_range.size(); k++) {
            ChVector2<int> ij = p.m_range[k];

            // Move from (i, j) to (x, y, z) representation in the world frame
            double x = ij.x() * m_delta;
            double y = ij.y() * m_delta;
            double z;
    #pragma omp critical(SCM_ray_casting)
            z = GetHeight(ij);

            ChVector<> vertex_abs = m_plane.TransformPointLocalToParent(ChVector<>(x, y, z));

            // Create ray at current grid location
            collision::ChCollisionSystem::ChRayhitResult mrayhit_result;
            ChVector<> to = vertex_abs + m_Z * m_test_offset_up;
            ChVector<> from = to - m_Z * m_test_offset_down;

            // Ray-OBB test (quick rejection)
            if (m_moving_patch && !RayOBBtest(p, from, m_Z))
                continue;

            // Cast ray into collision system
            GetSystem()->GetCollisionSystem()->RayHit(from, to, mrayhit_result);
            num_ray_casts++;

            if (mrayhit_result.hit) {
    #pragma omp critical(SCM_ray_casting)
                {
                    // If this is the first hit from this node, initialize the node record
                    if (m_grid_map.find(ij) == m_grid_map.end()) {
                        m_grid_map.insert(std::make_pair(ij, NodeRecord(z, z, GetInitNormal(ij))));
                    }

                    // Add to our map of hits to process
                    HitRecord record = {mrayhit_result.hitModel->GetContactable(), mrayhit_result.abs_hitPoint, -1};
                    hits.insert(std::make_pair(ij, record));
                    m_num_ray_hits++;
                }
            }
        }
        m_num_ray_casts += num_ray_casts;
    }

#else

    // Map-reduce approach (to eliminate critical section)

    const int nthreads = GetSystem()->GetNumThreadsChrono();
    std::vector<std::unordered_map<ChVector2<int>, HitRecord, CoordHash>> t_hits(nthreads);

    // Loop through all moving patches (user-defined or default one)
    for (auto& p : m_patches) {
        m_timer_ray_testing.start();

        // Loop through all vertices in the patch range
        int num_ray_casts = 0;
    #pragma omp parallel for num_threads(nthreads) reduction(+ : num_ray_casts)
        for (int k = 0; k < p.m_range.size(); k++) {
            int t_num = ChOMP::GetThreadNum();
            ChVector2<int> ij = p.m_range[k];

            // Move from (i, j) to (x, y, z) representation in the world frame
            double x = ij.x() * m_delta;
            double y = ij.y() * m_delta;
            double z = GetHeight(ij);

            ChVector<> vertex_abs = m_plane.TransformPointLocalToParent(ChVector<>(x, y, z));

            // Create ray at current grid location
            collision::ChCollisionSystem::ChRayhitResult mrayhit_result;
            ChVector<> to = vertex_abs + m_Z * m_test_offset_up;
            ChVector<> from = to - m_Z * m_test_offset_down;

            // Ray-OBB test (quick rejection)
            if (m_moving_patch && !RayOBBtest(p, from, m_Z))
                continue;

            // Cast ray into collision system
            GetSystem()->GetCollisionSystem()->RayHit(from, to, mrayhit_result);
            num_ray_casts++;

            if (mrayhit_result.hit) {
                // Add to our map of hits to process
                HitRecord record = {mrayhit_result.hitModel->GetContactable(), mrayhit_result.abs_hitPoint, -1};
                t_hits[t_num].insert(std::make_pair(ij, record));
            }
        }

        m_timer_ray_testing.stop();

        m_num_ray_casts += num_ray_casts;

        // Sequential insertion in global hits
        for (int t_num = 0; t_num < nthreads; t_num++) {
            for (auto& h : t_hits[t_num]) {
                // If this is the first hit from this node, initialize the node record
                if (m_grid_map.find(h.first) == m_grid_map.end()) {
                    double z = GetInitHeight(h.first);
                    m_grid_map.insert(std::make_pair(h.first, NodeRecord(z, z, GetInitNormal(h.first))));
                }
                ////hits.insert(h);
            }

            hits.insert(t_hits[t_num].begin(), t_hits[t_num].end());
            t_hits[t_num].clear();
        }
        m_num_ray_hits = (int)hits.size();
    }

#endif

    m_timer_ray_casting.stop();

    // --------------------
    // Find contact patches
    // --------------------

    m_timer_contact_patches.start();

    // Collect hit vertices assigned to each contact patch.
    struct ContactPatchRecord {
        std::vector<ChVector2<>> points;    // points in contact patch (in reference plane)
        std::vector<ChVector2<int>> nodes;  // grid nodes in the contact patch
        double area;                        // contact patch area
        double perimeter;                   // contact patch perimeter
        double oob;                         // approximate value of 1/b
    };
    std::vector<ContactPatchRecord> contact_patches;

    // Loop through all hit nodes and determine to which contact patch they belong.
    // Use a queue-based flood-filling algorithm based on the neighbors of each hit node.
    m_num_contact_patches = 0;
    for (auto& h : hits) {
        if (h.second.patch_id != -1)
            continue;

        ChVector2<int> ij = h.first;

        // Make a new contact patch and add this hit node to it
        h.second.patch_id = m_num_contact_patches++;
        ContactPatchRecord patch;
        patch.nodes.push_back(ij);
        patch.points.push_back(ChVector2<>(m_delta * ij.x(), m_delta * ij.y()));

        // Add current node to the work queue
        std::queue<ChVector2<int>> todo;
        todo.push(ij);

        while (!todo.empty()) {
            auto crt = hits.find(todo.front());  // Current hit node is first element in queue
            todo.pop();                          // Remove first element from queue

            ChVector2<int> crt_ij = crt->first;
            int crt_patch = crt->second.patch_id;

            // Loop through the neighbors of the current hit node
            for (int k = 0; k < 4; k++) {
                ChVector2<int> nbr_ij = crt_ij + neighbors4[k];
                // If neighbor is not a hit node, move on
                auto nbr = hits.find(nbr_ij);
                if (nbr == hits.end())
                    continue;
                // If neighbor already assigned to a contact patch, move on
                if (nbr->second.patch_id != -1)
                    continue;
                // Assign neighbor to the same contact patch
                nbr->second.patch_id = crt_patch;
                // Add neighbor point to patch lists
                patch.nodes.push_back(nbr_ij);
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
    double Mohr_mu = m_Mohr_mu;
    double Janosi_shear = m_Janosi_shear;
    double elastic_K = m_elastic_K;
    double damping_R = m_damping_R;

    // Process only hit nodes
    for (auto& h : hits) {
        ChVector2<> ij = h.first;

        auto& nr = m_grid_map.at(ij);      // node record
        const double& ca = nr.normal.z();  // cosine of angle between local normal and SCM plane vertical

        ChContactable* contactable = h.second.contactable;
        const ChVector<>& hit_point_abs = h.second.abs_point;
        int patch_id = h.second.patch_id;

        auto hit_point_loc = m_plane.TransformPointParentToLocal(hit_point_abs);

        if (m_soil_fun) {
            double Mohr_friction;
            m_soil_fun->Set(hit_point_loc, Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_cohesion, Mohr_friction, Janosi_shear,
                            elastic_K, damping_R);
            Mohr_mu = std::tan(Mohr_friction * CH_C_DEG_TO_RAD);
        }

        nr.hit_level = hit_point_loc.z();                              // along SCM z axis
        double p_hit_offset = ca * (nr.level_initial - nr.hit_level);  // along local normal direction

        // Elastic try (along local normal direction)
        nr.sigma = elastic_K * (p_hit_offset - nr.sinkage_plastic);

        // Handle unilaterality
        if (nr.sigma < 0) {
            nr.sigma = 0;
            continue;
        }

        // Mark current node as modified
        m_modified_nodes.push_back(ij);

        // Calculate velocity at touched grid node
        ChVector<> point_local(ij.x() * m_delta, ij.y() * m_delta, nr.level);
        ChVector<> point_abs = m_plane.TransformPointLocalToParent(point_local);
        ChVector<> speed_abs = contactable->GetContactPointSpeed(point_abs);

        // Calculate normal and tangent directions (expressed in absolute frame)
        ChVector<> N = m_plane.TransformDirectionLocalToParent(nr.normal);
        double Vn = Vdot(speed_abs, N);
        ChVector<> T = -(speed_abs - Vn * N);
        T.Normalize();

        // Update total sinkage and current level for this hit node
        nr.sinkage = p_hit_offset;
        nr.level = nr.hit_level;

        // Accumulate shear for Janosi-Hanamoto (along local tangent direction)
        nr.kshear += Vdot(speed_abs, -T) * GetSystem()->GetStep();

        // Plastic correction (along local normal direction)
        if (nr.sigma > nr.sigma_yield) {
            // Bekker formula
            nr.sigma = (contact_patches[patch_id].oob * Bekker_Kc + Bekker_Kphi) * pow(nr.sinkage, Bekker_n);
            nr.sigma_yield = nr.sigma;
            double old_sinkage_plastic = nr.sinkage_plastic;
            nr.sinkage_plastic = nr.sinkage - nr.sigma / elastic_K;
            nr.step_plastic_flow = (nr.sinkage_plastic - old_sinkage_plastic) / GetSystem()->GetStep();
        }

        // Elastic sinkage (along local normal direction)
        nr.sinkage_elastic = nr.sinkage - nr.sinkage_plastic;

        // Add compressive speed-proportional damping (not clamped by pressure yield)
        ////if (Vn < 0) {
        nr.sigma += -Vn * damping_R;
        ////}

        // Mohr-Coulomb
        double tau_max = Mohr_cohesion + nr.sigma * Mohr_mu;

        // Janosi-Hanamoto (along local tangent direction)
        nr.tau = tau_max * (1.0 - exp(-(nr.kshear / Janosi_shear)));

        // Calculate normal and tangential forces (in local node directions).
        // If specified, combine properties for soil-contactable interaction and soil-soil interaction.
        ChVector<> Fn = N * m_area * nr.sigma;
        ChVector<> Ft;

        //// TODO:  take into account "tread height" (add to SCMContactableData)?

        if (auto cprops = contactable->GetUserData<vehicle::SCMContactableData>()) {
            // Use weighted sum of soil-contactable and soil-soil parameters
            double c_tau_max = cprops->Mohr_cohesion + nr.sigma * cprops->Mohr_mu;
            double c_tau = c_tau_max * (1.0 - exp(-(nr.kshear / cprops->Janosi_shear)));
            double ratio = cprops->area_ratio;
            Ft = T * m_area * ((1 - ratio) * nr.tau + ratio * c_tau);
        } else {
            // Use only soil-soil parameters
            Ft = T * m_area * nr.tau;
        }

        if (ChBody* rigidbody = dynamic_cast<ChBody*>(contactable)) {
            // [](){} Trick: no deletion for this shared ptr, since 'rigidbody' was not a new ChBody()
            // object, but an already used pointer because mrayhit_result.hitModel->GetPhysicsItem()
            // cannot return it as shared_ptr, as needed by the ChLoadBodyForce:
            std::shared_ptr<ChBody> srigidbody(rigidbody, [](ChBody*) {});
            std::shared_ptr<ChLoadBodyForce> mload(new ChLoadBodyForce(srigidbody, Fn + Ft, false, point_abs, false));
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
                frc.moment = Vcross(Vsub(point_abs, srigidbody->GetPos()), force);
                m_contact_forces.insert(std::make_pair(contactable, frc));
            } else {
                // Update generalized force.
                ChVector<> force = Fn + Ft;
                itr->second.force += force;
                itr->second.moment += Vcross(Vsub(point_abs, srigidbody->GetPos()), force);
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

        // Update grid node height (in local SCM frame, along SCM z axis)
        nr.level = nr.level_initial - nr.sinkage / ca;

    }  // end loop on ray hits

    m_timer_contact_forces.stop();

    // --------------------------------------------------
    // Flow material to the side of rut, using heuristics
    // --------------------------------------------------

    m_timer_bulldozing.start();

    m_num_erosion_nodes = 0;

    if (m_bulldozing) {
        typedef std::unordered_set<ChVector2<int>, CoordHash> NodeSet;

        // Maximum level change between neighboring nodes (smoothing phase)
        double dy_lim = m_delta * m_erosion_slope;

        // (1) Raise boundaries of each contact patch
        m_timer_bulldozing_boundary.start();

        NodeSet boundary;  // union of contact patch boundaries
        for (auto p : contact_patches) {
            NodeSet p_boundary;  // boundary of effective contact patch

            // Calculate the displaced material from all touched nodes and identify boundary
            double tot_step_flow = 0;
            for (const auto& ij : p.nodes) {                     // for each node in contact patch
                const auto& nr = m_grid_map.at(ij);              //   get node record
                if (nr.sigma <= 0)                               //   if node not touched
                    continue;                                    //     skip (not in effective patch)
                tot_step_flow += nr.step_plastic_flow;           //   accumulate displaced material
                for (int k = 0; k < 4; k++) {                    //   check each node neighbor
                    ChVector2<int> nbr_ij = ij + neighbors4[k];  //     neighbor node coordinates
                    ////if (!CheckMeshBounds(nbr_ij))                     //     if neighbor out of bounds
                    ////    continue;                                     //       skip neighbor
                    if (m_grid_map.find(nbr_ij) == m_grid_map.end())  //     if neighbor not yet recorded
                        p_boundary.insert(nbr_ij);                    //       set neighbor as boundary
                    else if (m_grid_map.at(nbr_ij).sigma <= 0)        //     if neighbor not touched
                        p_boundary.insert(nbr_ij);                    //       set neighbor as boundary
                }
            }
            tot_step_flow *= GetSystem()->GetStep();

            // Target raise amount for each boundary node (unless clamped)
            double diff = m_flow_factor * tot_step_flow / p_boundary.size();

            // Raise boundary (create a sharp spike which will be later smoothed out with erosion)
            for (const auto& ij : p_boundary) {                                  // for each node in bndry
                m_modified_nodes.push_back(ij);                                  //   mark as modified
                if (m_grid_map.find(ij) == m_grid_map.end()) {                   //   if not yet recorded
                    double z = GetInitHeight(ij);                                //     undeformed height
                    const ChVector<>& n = GetInitNormal(ij);                     //     terrain normal
                    m_grid_map.insert(std::make_pair(ij, NodeRecord(z, z, n)));  //     add new node record
                    m_modified_nodes.push_back(ij);                              //     mark as modified
                }                                                                //
                auto& nr = m_grid_map.at(ij);                                    //   node record
                nr.erosion = true;                                               //   add to erosion domain
                AddMaterialToNode(diff, nr);                                     //   add raise amount
            }

            // Accumulate boundary
            boundary.insert(p_boundary.begin(), p_boundary.end());

        }  // end for contact_patches

        m_timer_bulldozing_boundary.stop();

        // (2) Calculate erosion domain (dilate boundary)
        m_timer_bulldozing_domain.start();

        NodeSet erosion_domain = boundary;
        NodeSet erosion_front = boundary;  // initialize erosion front to boundary nodes
        for (int i = 0; i < m_erosion_propagations; i++) {
            NodeSet front;                                       // new erosion front
            for (const auto& ij : erosion_front) {               // for each node in current erosion front
                for (int k = 0; k < 4; k++) {                    // check each of its neighbors
                    ChVector2<int> nbr_ij = ij + neighbors4[k];  //   neighbor node coordinates
                    ////if (!CheckMeshBounds(nbr_ij))                       //   if out of bounds
                    ////    continue;                                       //     ignore neighbor
                    if (m_grid_map.find(nbr_ij) == m_grid_map.end()) {  //   if neighbor not yet recorded
                        double z = GetInitHeight(nbr_ij);               //     undeformed height at neighbor location
                        const ChVector<>& n = GetInitNormal(nbr_ij);    //     terrain normal at neighbor location
                        NodeRecord nr(z, z, n);                         //     create new record
                        nr.erosion = true;                              //     include in erosion domain
                        m_grid_map.insert(std::make_pair(nbr_ij, nr));  //     add new node record
                        front.insert(nbr_ij);                           //     add neighbor to new front
                        m_modified_nodes.push_back(nbr_ij);             //     mark as modified
                    } else {                                            //   if neighbor previously recorded
                        NodeRecord& nr = m_grid_map.at(nbr_ij);         //     get existing record
                        if (!nr.erosion && nr.sigma <= 0) {             //     if neighbor not touched
                            nr.erosion = true;                          //       include in erosion domain
                            front.insert(nbr_ij);                       //       add neighbor to new front
                            m_modified_nodes.push_back(nbr_ij);         //       mark as modified
                        }
                    }
                }
            }
            erosion_domain.insert(front.begin(), front.end());  // add current front to erosion domain
            erosion_front = front;                              // advance erosion front
        }

        m_num_erosion_nodes = static_cast<int>(erosion_domain.size());
        m_timer_bulldozing_domain.stop();

        // (3) Erosion algorithm on domain
        m_timer_bulldozing_erosion.start();

        for (int iter = 0; iter < m_erosion_iterations; iter++) {
            for (const auto& ij : erosion_domain) {
                auto& nr = m_grid_map.at(ij);
                for (int k = 0; k < 4; k++) {
                    ChVector2<int> nbr_ij = ij + neighbors4[k];
                    auto rec = m_grid_map.find(nbr_ij);
                    if (rec == m_grid_map.end())
                        continue;
                    auto& nbr_nr = rec->second;

                    // (3.1) Flow remaining material to neighbor
                    double diff = 0.5 * (nr.massremainder - nbr_nr.massremainder) / 4;  //// TODO: rethink this!
                    if (diff > 0) {
                        RemoveMaterialFromNode(diff, nr);
                        AddMaterialToNode(diff, nbr_nr);
                    }

                    // (3.2) Smoothing
                    if (nbr_nr.sigma == 0) {
                        double dy = (nr.level + nr.massremainder) - (nbr_nr.level + nbr_nr.massremainder);
                        diff = 0.5 * (std::abs(dy) - dy_lim) / 4;  //// TODO: rethink this!
                        if (diff > 0) {
                            if (dy > 0) {
                                RemoveMaterialFromNode(diff, nr);
                                AddMaterialToNode(diff, nbr_nr);
                            } else {
                                RemoveMaterialFromNode(diff, nbr_nr);
                                AddMaterialToNode(diff, nr);
                            }
                        }
                    }
                }
            }
        }

        m_timer_bulldozing_erosion.stop();

    }  // end do_bulldozing

    m_timer_bulldozing.stop();

    // --------------------
    // Update visualization
    // --------------------

    m_timer_visualization.start();

    if (m_trimesh_shape) {
        // Loop over list of modified nodes and adjust corresponding mesh vertices.
        // If not rendering a wireframe mesh, also update normals.
        for (const auto& ij : m_modified_nodes) {
            if (!CheckMeshBounds(ij))                 // if node outside mesh
                continue;                             //   do nothing
            const auto& nr = m_grid_map.at(ij);       // grid node record
            int iv = GetMeshVertexIndex(ij);          // mesh vertex index
            UpdateMeshVertexCoordinates(ij, iv, nr);  // update vertex coordinates and color
            modified_vertices.push_back(iv);          // cache in list of modified mesh vertices
            if (!m_trimesh_shape->IsWireframe())      // if not wireframe
                UpdateMeshVertexNormal(ij, iv);       // update vertex normal
        }

        m_trimesh_shape->SetModifiedVertices(modified_vertices);
    }

    m_timer_visualization.stop();
}

void SCMDeformableSoil::AddMaterialToNode(double amount, NodeRecord& nr) {
    if (amount > nr.hit_level - nr.level) {                      //   if not possible to assign all mass
        nr.massremainder += amount - (nr.hit_level - nr.level);  //     material to be further propagated
        amount = nr.hit_level - nr.level;                        //     clamp raise amount
    }                                                            //
    nr.level += amount;                                          //   modify node level
    nr.level_initial += amount;                                  //   reset node initial level
}

void SCMDeformableSoil::RemoveMaterialFromNode(double amount, NodeRecord& nr) {
    if (nr.massremainder > amount) {                                 // if too much remainder material
        nr.massremainder -= amount;                                  //   decrease remainder material
        /*amount = 0;*/                                              //   ???
    } else if (nr.massremainder < amount && nr.massremainder > 0) {  // if not enough remainder material
        amount -= nr.massremainder;                                  //   clamp removed amount
        nr.massremainder = 0;                                        //   remainder material exhausted
    }                                                                //
    nr.level -= amount;                                              //   modify node level
    nr.level_initial -= amount;                                      //   reset node initial level
}

// Update vertex position and color in visualization mesh
void SCMDeformableSoil::UpdateMeshVertexCoordinates(const ChVector2<int> ij, int iv, const NodeRecord& nr) {
    auto& trimesh = *m_trimesh_shape->GetMesh();
    std::vector<ChVector<>>& vertices = trimesh.getCoordsVertices();
    std::vector<ChColor>& colors = trimesh.getCoordsColors();

    // Update visualization mesh vertex position
    vertices[iv] = m_plane.TransformPointLocalToParent(ChVector<>(ij.x() * m_delta, ij.y() * m_delta, nr.level));

    // Update visualization mesh vertex color
    if (m_plot_type != SCMDeformableTerrain::PLOT_NONE) {
        ChColor mcolor;
        switch (m_plot_type) {
            case SCMDeformableTerrain::PLOT_LEVEL:
                mcolor = ChColor::ComputeFalseColor(nr.level, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_LEVEL_INITIAL:
                mcolor = ChColor::ComputeFalseColor(nr.level_initial, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SINKAGE:
                mcolor = ChColor::ComputeFalseColor(nr.sinkage, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SINKAGE_ELASTIC:
                mcolor = ChColor::ComputeFalseColor(nr.sinkage_elastic, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SINKAGE_PLASTIC:
                mcolor = ChColor::ComputeFalseColor(nr.sinkage_plastic, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_STEP_PLASTIC_FLOW:
                mcolor = ChColor::ComputeFalseColor(nr.step_plastic_flow, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_K_JANOSI:
                mcolor = ChColor::ComputeFalseColor(nr.kshear, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_PRESSURE:
                mcolor = ChColor::ComputeFalseColor(nr.sigma, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_PRESSURE_YELD:
                mcolor = ChColor::ComputeFalseColor(nr.sigma_yield, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_SHEAR:
                mcolor = ChColor::ComputeFalseColor(nr.tau, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_MASSREMAINDER:
                mcolor = ChColor::ComputeFalseColor(nr.massremainder, m_plot_v_min, m_plot_v_max);
                break;
            case SCMDeformableTerrain::PLOT_ISLAND_ID:
                if (nr.erosion)
                    mcolor = ChColor(0, 0, 0);
                if (nr.sigma > 0)
                    mcolor = ChColor(1, 0, 0);
                break;
            case SCMDeformableTerrain::PLOT_IS_TOUCHED:
                if (nr.sigma > 0)
                    mcolor = ChColor(1, 0, 0);
                else
                    mcolor = ChColor(0, 0, 1);
                break;
            case SCMDeformableTerrain::PLOT_NONE:
                break;
        }
        colors[iv] = mcolor;
    }
}

// Update vertex normal in visualization mesh.
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

// Get the heights of modified grid nodes.
std::vector<SCMDeformableTerrain::NodeLevel> SCMDeformableSoil::GetModifiedNodes(bool all_nodes) const {
    std::vector<SCMDeformableTerrain::NodeLevel> nodes;
    if (all_nodes) {
        for (const auto& nr : m_grid_map) {
            nodes.push_back(std::make_pair(nr.first, nr.second.level));
        }
    } else {
        for (const auto& ij : m_modified_nodes) {
            auto rec = m_grid_map.find(ij);
            assert(rec != m_grid_map.end());
            nodes.push_back(std::make_pair(ij, rec->second.level));
        }
    }
    return nodes;
}

// Modify the level of grid nodes from the given list.
// NOTE: We set only the level of the specified nodes and none of the other soil properties.
//       As such, some plot types may be incorrect at these nodes.
void SCMDeformableSoil::SetModifiedNodes(const std::vector<SCMDeformableTerrain::NodeLevel>& nodes) {
    for (const auto& n : nodes) {
        // Modify existing entry in grid map or insert new one
        m_grid_map[n.first] = SCMDeformableSoil::NodeRecord(n.second, n.second, GetInitNormal(n.first));
    }

    // Update visualization
    if (m_trimesh_shape) {
        for (const auto& n : nodes) {
            auto ij = n.first;                           // grid location
            if (!CheckMeshBounds(ij))                    // if outside mesh
                continue;                                //   do nothing
            const auto& nr = m_grid_map.at(ij);          // grid node record
            int iv = GetMeshVertexIndex(ij);             // mesh vertex index
            UpdateMeshVertexCoordinates(ij, iv, nr);     // update vertex coordinates and color
            if (!m_trimesh_shape->IsWireframe())         // if not in wireframe mode
                UpdateMeshVertexNormal(ij, iv);          //   update vertex normal
            m_external_modified_vertices.push_back(iv);  // cache in list
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
