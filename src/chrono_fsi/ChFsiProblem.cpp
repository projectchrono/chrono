// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility class to set up a Chrono::FSI problem.
//
// =============================================================================

#include <fstream>
#include <iostream>
#include <sstream>
#include <queue>

#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono_fsi/ChFsiProblem.h"

#include "chrono_thirdparty/stb/stb.h"
#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::endl;

namespace chrono {
namespace fsi {

// ----------------------------------------------------------------------------

ChFsiProblem::ChFsiProblem(ChSystem& sys, double spacing)
    : m_sys(sys), m_spacing(spacing), m_initialized(false), m_offset_sph(VNULL), m_offset_bce(VNULL), m_verbose(false) {
    // Create ground body
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetFixed(true);
    sys.AddBody(m_ground);

    // Associate MBS system with underlying FSI system
    m_sysFSI.SetVerbose(m_verbose);
    m_sysFSI.AttachSystem(&sys);
    m_sysFSI.SetInitialSpacing(spacing);
    m_sysFSI.SetKernelLength(spacing);
}

ChFsiProblemCartesian::ChFsiProblemCartesian(ChSystem& sys, double spacing) : ChFsiProblem(sys, spacing) {}

ChFsiProblemCylindrical::ChFsiProblemCylindrical(ChSystem& sys, double spacing) : ChFsiProblem(sys, spacing) {}

// ----------------------------------------------------------------------------

void ChFsiProblem::SetVerbose(bool verbose) {
    m_sysFSI.SetVerbose(verbose);
    m_verbose = verbose;
}

size_t ChFsiProblem::AddRigidBody(std::shared_ptr<ChBody> body,
                                  const utils::ChBodyGeometry& geometry,
                                  bool check_embedded,
                                  bool use_grid) {
    if (m_verbose) {
        cout << "Add FSI rigid body " << body->GetName() << endl;
    }

    RigidBody b;
    b.body = body;
    b.geometry = geometry;
    b.check_embedded = check_embedded;

    //// TODO: eliminate duplicate BCE markers (from multiple volumes).
    ////       easiest if BCE created on a grid!

    // Create the BCE markers for each shape in the collision geometry
    for (const auto& sphere : geometry.coll_spheres) {
        std::vector<ChVector3d> points;
        m_sysFSI.CreateBCE_sphere(sphere.radius, true, !use_grid, points);
        for (auto& p : points)
            p += sphere.pos;
        b.bce.insert(b.bce.end(), points.begin(), points.end());
    }
    for (const auto& box : geometry.coll_boxes) {
        std::vector<ChVector3d> points;
        m_sysFSI.CreateBCE_box(box.dims, true, points);
        for (auto& p : points)
            p = box.pos + box.rot.Rotate(p);
        b.bce.insert(b.bce.end(), points.begin(), points.end());
    }
    for (const auto& cyl : geometry.coll_cylinders) {
        std::vector<ChVector3d> points;
        m_sysFSI.CreateBCE_cylinder(cyl.radius, cyl.length, true, true, !use_grid, points);
        for (auto& p : points)
            p = cyl.pos + cyl.rot.Rotate(p);
        b.bce.insert(b.bce.end(), points.begin(), points.end());
    }
    for (const auto& mesh : geometry.coll_meshes) {
        std::vector<ChVector3d> points;
        m_sysFSI.CreateMeshPoints(*mesh.trimesh, m_spacing, points);
        for (auto& p : points)
            p += mesh.pos;
        b.bce.insert(b.bce.end(), points.begin(), points.end());
    }

    m_bodies.push_back(b);

    if (m_verbose) {
        cout << "  Cummulative num. BCE markers: " << b.bce.size() << endl;
    }

    return b.bce.size();
}

size_t ChFsiProblem::AddRigidBodySphere(std::shared_ptr<ChBody> body,
                                        const ChVector3d& pos,
                                        double radius,
                                        bool use_grid) {
    utils::ChBodyGeometry geometry;
    geometry.coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius));
    return AddRigidBody(body, geometry, true, use_grid);
}

size_t ChFsiProblem::AddRigidBodyBox(std::shared_ptr<ChBody> body, const ChFramed& frame, const ChVector3d& size) {
    utils::ChBodyGeometry geometry;
    geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), size));
    return AddRigidBody(body, geometry, true, true);
}

size_t ChFsiProblem::AddRigidBodyCylinderX(std::shared_ptr<ChBody> body,
                                           const ChFramed& frame,
                                           double radius,
                                           double length,
                                           bool use_grid) {
    utils::ChBodyGeometry geometry;
    geometry.coll_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRotMat().GetAxisX(), radius, length));
    return AddRigidBody(body, geometry, true, use_grid);
}

size_t ChFsiProblem::AddRigidBodyMesh(std::shared_ptr<ChBody> body,
                                      const ChVector3d& pos,
                                      const std::string& obj_filename,
                                      const ChVector3d& interior_point,
                                      double scale) {
    utils::ChBodyGeometry geometry;
    geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(pos, obj_filename, interior_point, scale));
    return AddRigidBody(body, geometry, true, true);
}

// ----------------------------------------------------------------------------

void ChFsiProblem::Initialize() {
    // Prune SPH particles at grid locations that overlap with obstacles
    if (!m_bodies.empty()) {
        if (m_verbose)
            cout << "Remove SPH particles inside obstacle volumes" << endl;

        for (auto& b : m_bodies)
            if (b.check_embedded)
                ProcessBody(b);

        if (m_verbose)
            cout << "  Num. SPH particles: " << m_sph.size() << endl;
    }

    // Convert SPH and boundary BCE grid points to real coordinates and apply patch transformation
    ChAABB aabb;
    RealPoints sph_points;
    sph_points.reserve(m_sph.size());
    for (const auto& p : m_sph) {
        ChVector3d point = Grid2Point(p);
        point += m_offset_sph;
        sph_points.push_back(point);
        aabb.min = Vmin(aabb.min, point);
        aabb.max = Vmax(aabb.max, point);
    }

    m_sph_aabb = aabb;

    // Include boundary BCE markers in AABB
    RealPoints bce_points;
    bce_points.reserve(m_bce.size());
    for (const auto& p : m_bce) {
        ChVector3d point = Grid2Point(p);
        point += m_offset_bce;
        bce_points.push_back(point);
        aabb.min = Vmin(aabb.min, point);
        aabb.max = Vmax(aabb.max, point);
    }

    // Include body BCE markers in AABB
    for (auto& b : m_bodies) {
        for (const auto& p : b.bce) {
            auto point = b.body->TransformPointLocalToParent(p);
            aabb.min = Vmin(aabb.min, point);
            aabb.max = Vmax(aabb.max, point);
        }
    }

    if (m_verbose) {
        cout << "AABB of SPH particles:" << endl;
        cout << "  min: " << m_sph_aabb.min << endl;
        cout << "  max: " << m_sph_aabb.max << endl;
        cout << "AABB of SPH particles + BCE markers" << endl;
        cout << "  min: " << aabb.min << endl;
        cout << "  max: " << aabb.max << endl;
    }

    // Set computational domain
    if (!m_domain_aabb.IsInverted()) {
        // Use provided computational domain
        m_sysFSI.SetBoundaries(m_domain_aabb.min, m_domain_aabb.max);
    } else {
        // Set computational domain based on actual AABB of all markers
        int bce_layers = m_sysFSI.GetNumBCELayers();
        m_domain_aabb = ChAABB(aabb.min - bce_layers * m_spacing, aabb.max + bce_layers * m_spacing);
        m_sysFSI.SetBoundaries(m_domain_aabb.min, m_domain_aabb.max);
    }

    // Callback for setting initial particle properties
    if (!m_props_cb)
        m_props_cb = chrono_types::make_shared<ParticlePropertiesCallback>(m_sysFSI);

    // Create SPH particles
    switch (m_sysFSI.GetPhysicsProblem()) {
        case ChSystemFsi::PhysicsProblem::CFD: {
            for (const auto& pos : sph_points) {
                m_props_cb->set(pos);
                m_sysFSI.AddSPHParticle(pos, m_props_cb->rho0, m_props_cb->p0, m_props_cb->mu0, m_props_cb->v0);
            }
            break;
        }
        case ChSystemFsi::PhysicsProblem::CRM: {
            ChVector3d tau_offdiag(0);
            for (const auto& pos : sph_points) {
                m_props_cb->set(pos);
                ChVector3d tau_diag(-m_props_cb->p0);
                m_sysFSI.AddSPHParticle(pos, m_props_cb->rho0, m_props_cb->p0, m_props_cb->mu0, m_props_cb->v0,  //
                                        tau_diag, tau_offdiag);
            }
            break;
        }
    }

    // Create boundary BCE markers
    // (ATTENTION: BCE markers must be created after the SPH particles!)
    m_sysFSI.AddPointsBCE(m_ground, bce_points, ChFrame<>(), false);

    // Create the body BCE markers
    // (ATTENTION: BCE markers for moving objects must be created after the fixed BCE markers!)
    for (const auto& b : m_bodies) {
        auto body_index = m_sysFSI.AddFsiBody(b.body);
        m_sysFSI.AddPointsBCE(b.body, b.bce, ChFrame<>(), true);
        m_fsi_bodies[b.body] = body_index;
    }

    // Initialize the underlying FSI system
    m_sysFSI.Initialize();
    m_initialized = true;
}

//// TODO: use some additional "fuzz"?
// Check if specified point is inside a primitive shape of the given geometry.
bool InsidePoint(const utils::ChBodyGeometry& geometry, const ChVector3d& p) {
    for (const auto& sphere : geometry.coll_spheres) {
        if ((p - sphere.pos).Length2() <= sphere.radius * sphere.radius)
            return true;
    }
    for (const auto& box : geometry.coll_boxes) {
        auto pp = box.rot.RotateBack(p - box.pos);
        if (std::abs(pp.x()) <= box.dims.x() / 2 ||  //
            std::abs(pp.y()) <= box.dims.y() / 2 ||  //
            std::abs(pp.z()) <= box.dims.z() / 2)
            return true;
    }
    for (const auto& cyl : geometry.coll_cylinders) {
        auto pp = cyl.rot.RotateBack(p - cyl.pos);
        if (pp.x() * pp.x() + pp.y() * pp.y() <= cyl.radius * cyl.radius || std::abs(pp.z()) <= cyl.length / 2)
            return true;
    }
    return false;
}

// Prune SPH particles inside a body volume.
void ChFsiProblem::ProcessBody(RigidBody& b) {
    int num_removed = 0;

    // Traverse all body BCEs (with potential duplicates), transform into the ChFsiProblem frame, express in grid
    // coordinates and calculate the (integer) body AABB.
    ChVector3i aabb_min(+std::numeric_limits<int>::max());
    ChVector3i aabb_max(-std::numeric_limits<int>::max());
    for (auto& p : b.bce) {
        auto p_abs = b.body->TransformPointLocalToParent(p);  // BCE point in absolute frame
        auto p_sph = p_abs - m_offset_sph;                    // BCE point in ChFsiProblem frame
        auto p_grd = Snap2Grid(p_sph);                        // BCE point in integer grid coordinates
        aabb_min = Vmin(aabb_min, p_grd);
        aabb_max = Vmax(aabb_max, p_grd);
    }

    //// TODO: possible performance optimization: also check if grid point inside SPH AABB

    // Collect all grid points inside the volume of a body primitive shape
    GridPoints interior;
    for (int ix = aabb_min.x(); ix <= aabb_max.x(); ix++) {
        for (int iy = aabb_min.y(); iy <= aabb_max.y(); iy++) {
            for (int iz = aabb_min.z(); iz <= aabb_max.z(); iz++) {
                // Convert to body local frame
                ChVector3d p_sph = Grid2Point({ix, iy, iz});
                ChVector3d p_abs = p_sph + m_offset_sph;
                ChVector3d p_loc = b.body->TransformPointParentToLocal(p_abs);
                // Check if inside a primitive shape
                if (InsidePoint(b.geometry, p_loc))
                    interior.insert(ChVector3i(ix, iy, iz));
            }
        }
    }

    // Remove any SPH particles at an interior location
    for (const auto& p : interior) {
        auto iter = m_sph.find(p);
        if (iter != m_sph.end()) {
            m_sph.erase(iter);
            num_removed++;
        }
    }

    // Treat mesh shapes spearately
    for (const auto& mesh : b.geometry.coll_meshes) {
        auto num_removed_mesh = ProcessBodyMesh(b, *mesh.trimesh, mesh.int_point);
        num_removed += num_removed_mesh;
    }

    if (m_verbose) {
        cout << "  Body name: " << b.body->GetName() << endl;
        cout << "    Num. SPH particles removed: " << num_removed << endl;
    }
}

// Offsets for the 6 neighbors of an integer grid node.
static const std::vector<ChVector3i> nbr3D{
    ChVector3i(-1, 0, 0),  //
    ChVector3i(+1, 0, 0),  //
    ChVector3i(0, -1, 0),  //
    ChVector3i(0, +1, 0),  //
    ChVector3i(0, 0, -1),  //
    ChVector3i(0, 0, +1)   //
};

// Prune SPH particles inside a body mesh volume.
int ChFsiProblem::ProcessBodyMesh(RigidBody& b, ChTriangleMeshConnected trimesh, const ChVector3d& interior_point) {
    // Transform mesh in ChFsiProblem frame
    // (to address any roundoff issues that may result in a set of BCE markers that are not watertight)
    for (auto& v : trimesh.GetCoordsVertices()) {
        auto v_abs = b.body->TransformPointLocalToParent(v);  // vertex in absolute frame
        v = v_abs - m_offset_sph;                             // vertex in FSIProblem frame
    }

    // BCE marker locations (in FSIProblem frame)
    std::vector<ChVector3d> bce;
    m_sysFSI.CreateMeshPoints(trimesh, m_spacing, bce);

    // BCE marker locations in integer grid coordinates
    GridPoints gbce;
    for (auto& p : bce) {
        gbce.insert(Snap2Grid(p));
    }

    // Express the provided interior point in ChFsiProblem grid coordinates
    auto c_abs = b.body->TransformPointLocalToParent(interior_point);  // interior point in absolute frame
    auto c_sph = c_abs - m_offset_sph;                                 // interior point in ChFsiProblem frame
    auto c = Snap2Grid(c_sph);                                         // interior point in integer grid coordinates

    // Calculate the (integer) mesh AABB
    ChVector3i aabb_min(+std::numeric_limits<int>::max());
    ChVector3i aabb_max(-std::numeric_limits<int>::max());
    for (const auto& p : gbce) {
        aabb_min = Vmin(aabb_min, p);
        aabb_max = Vmax(aabb_max, p);
    }

    // Collect all grid points contained in the BCE volume in a set (initialized with the mesh BCEs)
    GridPoints list = gbce;

    // Use a queue-based flood-filling algorithm to find all points interior to the mesh volume
    std::queue<ChVector3i> todo;

    // Add the provided interior point to the work queue then iterate until the queue is empty
    todo.push({c.x(), c.y(), c.z()});
    while (!todo.empty()) {
        // Get first element in queue, add it to the set, then remove from queue
        auto crt = todo.front();
        list.insert(crt);
        todo.pop();

        // Safeguard -- stop as soon as we spill out of the mesh AABB
        if (!(crt > aabb_min && crt < aabb_max)) {
            std::cout << "Obstacle BCE set is NOT watertight!" << std::endl;
            throw std::invalid_argument("Obstacle BCE set is NOT watertight!");
        }

        // Loop through all 6 neighbors of the current node and add them to the end of the work queue
        // if not already in the set
        for (int k = 0; k < 6; k++) {
            auto nbr = crt + nbr3D[k];
            if (list.find(nbr) == list.end())
                todo.push(nbr);
        }
    }

    // Loop through the set of nodes and remove any SPH particle at one of these locations
    size_t num_removed = 0;
    for (const auto& p : list) {
        auto iter = m_sph.find(p);
        if (iter != m_sph.end()) {
            m_sph.erase(iter);
            num_removed++;
        }
    }

    return num_removed;
}

void ChFsiProblem::SaveInitialMarkers(const std::string& out_dir) const {
    // SPH particle grid locations
    std::ofstream sph_grid(out_dir + "/sph_grid.txt", std::ios_base::out);
    for (const auto& p : m_sph)
        sph_grid << p << std::endl;

    // Fixed BCE marker grid locations
    std::ofstream bce_grid(out_dir + "/bce_grid.txt", std::ios_base::out);
    for (const auto& p : m_bce)
        bce_grid << p << std::endl;

    // Body BCE marker locations
    std::ofstream obs_bce(out_dir + "/body_bce.txt", std::ios_base::out);
    for (const auto& b : m_bodies) {
        for (const auto& p : b.bce)
            obs_bce << p << std::endl;
    }
}

const ChVector3d& ChFsiProblem::GetFsiBodyForce(std::shared_ptr<ChBody> body) const {
    auto index = m_fsi_bodies.at(body);
    return m_sysFSI.GetFsiBodyForce(index);
}

const ChVector3d& ChFsiProblem::GetFsiBodyTorque(std::shared_ptr<ChBody> body) const {
    auto index = m_fsi_bodies.at(body);
    return m_sysFSI.GetFsiBodyTorque(index);
}

// ----------------------------------------------------------------------------

void ChFsiProblemCartesian::Construct(const std::string& sph_file, const std::string& bce_file, const ChVector3d& pos) {
    if (m_verbose) {
        cout << "Construct ChFsiProblem from data files" << endl;
    }

    std::string line;
    int x, y, z;

    std::ifstream sph(sph_file, std::ios_base::in);
    while (std::getline(sph, line)) {
        std::istringstream iss(line, std::ios_base::in);
        iss >> x >> y >> z;
        m_sph.insert(ChVector3i(x, y, z));
    }

    std::ifstream bce(bce_file, std::ios_base::in);
    while (std::getline(bce, line)) {
        std::istringstream iss(line, std::ios_base::in);
        iss >> x >> y >> z;
        m_bce.insert(ChVector3i(x, y, z));
    }

    if (m_verbose) {
        cout << "  SPH particles filename: " << sph_file << "  [" << m_sph.size() << "]" << endl;
        cout << "  BCE markers filename: " << bce_file << "  [" << m_bce.size() << "]" << endl;
    }

    m_offset_sph = pos;
    m_offset_bce = m_offset_sph;
}

void ChFsiProblemCartesian::Construct(const ChVector3d& box_size,
                                      const ChVector3d& pos,
                                      bool bottom_wall,
                                      bool side_walls) {
    if (m_verbose) {
        cout << "Construct box ChFsiProblem" << endl;
    }

    // Number of BCE layers
    int bce_layers = m_sysFSI.GetNumBCELayers();

    // Number of points in each direction
    int Nx = std::round(box_size.x() / m_spacing) + 1;
    int Ny = std::round(box_size.y() / m_spacing) + 1;
    int Nz = std::round(box_size.z() / m_spacing) + 1;

    // Reserve space for containers
    int num_sph = Nx * Ny * Nz;
    int num_bce = 0;
    if (bottom_wall)
        num_bce += Nx * Ny * bce_layers;
    if (side_walls)
        num_bce += (Nx + Ny + 2 * bce_layers) * 2 * bce_layers * Nz;

    std::vector<ChVector3i> sph;
    std::vector<ChVector3i> bce;
    sph.reserve(num_sph);
    bce.reserve(num_bce);

    // Generate SPH and bottom BCE points
    for (int Ix = 0; Ix < Nx; Ix++) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Iz = 0; Iz < Nz; Iz++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above 0
            }
            if (bottom_wall) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, -Iz));  // BCE markers below 0
                }
            }
        }
    }

    // Generate side BCE points
    if (side_walls) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                    bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, Iz));
                }
            }
        }
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                    bce.push_back(ChVector3i(Nx - 1 - Ix, Iy, Iz));
                }
            }
        }
    }

    // Insert in cached sets
    for (auto& p : sph) {
        m_sph.insert(p);
    }
    for (auto& p : bce) {
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "  Particle grid size:      " << Nx << " " << Ny << " " << Nz << endl;
        cout << "  Num. SPH particles:      " << m_sph.size() << " (" << sph.size() << ")" << endl;
        cout << "  Num. bndry. BCE markers: " << m_bce.size() << " (" << bce.size() << ")" << endl;
    }

    m_offset_sph = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);
    if (bottom_wall || side_walls)
        m_offset_bce = m_offset_sph;
}

void ChFsiProblemCartesian::Construct(const std::string& heightmap_file,
                                      double length,
                                      double width,
                                      const ChVector2d& height_range,
                                      double depth,
                                      bool uniform_depth,
                                      const ChVector3d& pos,
                                      bool bottom_wall,
                                      bool side_walls) {
    if (m_verbose) {
        cout << "Construct ChFsiProblem from heightmap file" << endl;
    }

    // Read the image file (request only 1 channel) and extract number of pixels
    STB cmap;
    if (!cmap.ReadFromFile(heightmap_file, 1)) {
        throw std::invalid_argument("Cannot open height map image file");
    }

    int nx = cmap.GetWidth();   // number of pixels in x direction
    int ny = cmap.GetHeight();  // number of pixels in y direction

    double dx = length / (nx - 1);
    double dy = width / (ny - 1);

    // Create a matrix with gray levels (g = 0x0000 is black, g = 0xffff is white)
    unsigned short g_min = +std::numeric_limits<unsigned short>::max();
    unsigned short g_max = -std::numeric_limits<unsigned short>::max();
    ChMatrixDynamic<unsigned short> gmap(nx + 1, ny + 1);
    for (int ix = 0; ix <= nx; ix++) {
        for (int iy = 0; iy <= ny; iy++) {
            auto gray = cmap.Gray(ix < nx ? ix : nx - 1, iy < ny ? iy : ny - 1);
            gmap(ix, iy) = gray;
            g_min = std::min(g_min, gray);
            g_max = std::max(g_max, gray);
        }
    }

    // Calculate height scaling (black->hMin, white->hMax)
    double h_scale = (height_range[1] - height_range[0]) / cmap.GetRange();

    // Calculate min and max heights and corresponding grid levels
    auto z_min = height_range[0] + g_min * h_scale;
    auto z_max = height_range[0] + g_max * h_scale;
    int Iz_min = (int)std::round(z_min / m_spacing);
    int Iz_max = (int)std::round(z_max / m_spacing);

    ////std::cout << "g_min = " << g_min << std::endl;
    ////std::cout << "g_max = " << g_max << std::endl;

    // Number of particles in each direction
    int Nx = (int)std::floor(length / m_spacing);
    int Ny = (int)std::floor(width / m_spacing);
    double Dx = length / (Nx - 1);
    double Dy = width / (Ny - 1);

    // Number of particles in Z direction over specified depth
    int Nz = (int)std::floor(depth / m_spacing);

    // Number of BCE layers
    int bce_layers = m_sysFSI.GetNumBCELayers();

    // Reserve space for containers
    std::vector<ChVector3i> sph;
    std::vector<ChVector3i> bce;
    sph.reserve(Nx * Ny * Nz);            // underestimate if not uniform depth
    bce.reserve(bce_layers * (Nx * Ny));  // underestimate if side_walls

    // Generate SPH and bottom BCE points
    for (int Ix = 0; Ix < Nx; Ix++) {
        double x = Ix * Dx;
        int ix = (int)std::round(x / dx);       // x location between pixels ix and ix+1
        double wx1 = ((ix + 1) * dx - x) / dx;  // weight for bi-linear interpolation
        double wx2 = (x - ix * dx) / dx;        // weight for bi-linear interpolation
        for (int Iy = 0; Iy < Ny; Iy++) {
            double y = Iy * Dy;
            int iy = (int)std::round(y / dy);       // y location between pixels iy and iy+1
            double wy1 = ((iy + 1) * dy - y) / dy;  // weight for bi-linear interpolation
            double wy2 = (y - iy * dy) / dy;        // weight for bi-linear interpolation

            // Calculate surface height at current location (bi-linear interpolation)
            auto h1 = wx1 * gmap(ix + 0, iy + 0) + wx2 * gmap(ix + 1, iy + 0);
            auto h2 = wx1 * gmap(ix + 0, iy + 1) + wx2 * gmap(ix + 1, iy + 1);
            auto z = height_range[0] + (wy1 * h1 + wy2 * h2) * h_scale;
            int Iz = (int)std::round(z / m_spacing);

            // Create SPH particle locations below current point
            int nz = uniform_depth ? Nz : Iz + Nz;
            for (int k = 0; k < nz; k++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));
                Iz--;
            }

            // Create BCE marker locations below the SPH particles
            if (bottom_wall) {
                for (int k = 0; k < bce_layers; k++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                    Iz--;
                }
            }
        }
    }

    std::cout << "IZmin = " << Iz_min << std::endl;
    std::cout << "IZmax = " << Iz_max << std::endl;

    // Generate side BCE points
    if (side_walls) {
        auto nz_min = Iz_min - Nz - bce_layers;
        auto nz_max = Iz_max + bce_layers;

        Iz_min -= Nz + bce_layers;
        Iz_max += bce_layers;
        Nz += 2 * bce_layers;
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int k = nz_min; k < nz_max; k++) {
                    bce.push_back(ChVector3i(Ix, Iy, k));
                    bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, k));
                }
            }
        }
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int k = nz_min; k < nz_max; k++) {
                    bce.push_back(ChVector3i(Ix, Iy, k));
                    bce.push_back(ChVector3i(Nx - 1 - Ix, Iy, k));
                }
            }
        }
    }

    // Note that pixels in image start at top-left.
    // Modify y coordinates so that particles start at bottom-left before inserting in cached sets.
    for (auto& p : sph) {
        p.y() = (Ny - 1) - p.y();
        m_sph.insert(p);
    }
    for (auto& p : bce) {
        p.y() = (Ny - 1) - p.y();
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "  Heightmap filename: " << heightmap_file << endl;
        cout << "  Num. SPH particles: " << m_sph.size() << endl;
        cout << "  Num. BCE markers: " << m_bce.size() << endl;
    }

    m_offset_sph = pos - ChVector3d(length / 2, width / 2, 0);
    if (bottom_wall || side_walls)
        m_offset_bce = m_offset_sph;
}

size_t ChFsiProblemCartesian::AddBoxContainer(const ChVector3d& box_size,  // box dimensions
                                              const ChVector3d& pos,       // reference positions
                                              bool bottom_wall,            // create bottom boundary
                                              bool side_walls,             // create side boundaries
                                              bool top_wall                // create top boundary
) {
    if (m_verbose) {
        cout << "Construct box container" << endl;
    }

    // Number of BCE layers
    int bce_layers = m_sysFSI.GetNumBCELayers();

    int Nx = std::round(box_size.x() / m_spacing) + 1;
    int Ny = std::round(box_size.y() / m_spacing) + 1;
    int Nz = std::round(box_size.z() / m_spacing) + 1;

    int num_bce = Nx * Ny * bce_layers;
    if (side_walls)
        num_bce += (Nx + Ny + 2 * bce_layers) * 2 * bce_layers * Nz;

    std::vector<ChVector3i> bce;
    bce.reserve(num_bce);

    // Bottom BCE points
    if (bottom_wall) {
        for (int Ix = 0; Ix < Nx; Ix++) {
            for (int Iy = 0; Iy < Ny; Iy++) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, -Iz));
                }
            }
        }
    }

    // Top BCE points
    if (top_wall) {
        for (int Ix = 0; Ix < Nx; Ix++) {
            for (int Iy = 0; Iy < Ny; Iy++) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Nz - 1 + Iz));
                }
            }
        }
    }

    // Generate side BCE points
    if (side_walls) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                    bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, Iz));
                }
            }
        }
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                    bce.push_back(ChVector3i(Nx - 1 - Ix, Iy, Iz));
                }
            }
        }
    }

    // Insert in cached sets
    for (auto& p : bce) {
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "  Particle grid size:      " << Nx << " " << Ny << " " << Nz << endl;
        cout << "  Num. bndry. BCE markers: " << m_bce.size() << " (" << bce.size() << ")" << endl;
    }

    m_offset_bce = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);

    return m_bce.size();
}

std::shared_ptr<ChBody> ChFsiProblemCartesian::AddWaveMaker(
    WavemakerType type,                     // wave generator type
    const ChVector3d& box_size,             // box dimensions
    const ChVector3d& pos,                  // reference position
    std::shared_ptr<ChFunction> piston_fun  // piston actuation function
) {
    if (m_verbose) {
        cout << "Construct piston wavemaker" << endl;
    }

    // Number of BCE layers
    int bce_layers = m_sysFSI.GetNumBCELayers();

    int Nx = std::round(box_size.x() / m_spacing) + 1;
    int Ny = std::round(box_size.y() / m_spacing) + 1;
    int Nz = std::round(box_size.z() / m_spacing) + 1;

    int num_bce = Nx * Ny * bce_layers;                      // bottom
    num_bce += (Nx + 2 * bce_layers) * 2 * bce_layers * Nz;  // -y and +y
    num_bce += (Ny + 2 * bce_layers) * bce_layers * Nz;      // +x

    std::vector<ChVector3i> bce;
    bce.reserve(num_bce);

    // Bottom BCE points
    for (int Ix = -bce_layers; Ix < Nx; Ix++) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Iz = 1; Iz <= bce_layers; Iz++) {
                bce.push_back(ChVector3i(Ix, Iy, -Iz));  // BCE markers below 0
            }
        }
    }

    // Generate side BCE points
    for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
        for (int Iy = -bce_layers; Iy < 0; Iy++) {
            for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                bce.push_back(ChVector3i(Ix, Iy, Iz));
                bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, Iz));
            }
        }
    }
    for (int Iy = 0; Iy < Ny; Iy++) {
        for (int Ix = -bce_layers; Ix < 0; Ix++) {
            for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                bce.push_back(ChVector3i(Nx - 1 - Ix, Iy, Iz));
            }
        }
    }

    // Insert in cached sets
    for (auto& p : bce) {
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "  Particle grid size:      " << Nx << " " << Ny << " " << Nz << endl;
        cout << "  Num. bndry. BCE markers: " << m_bce.size() << " (" << bce.size() << ")" << endl;
    }

    m_offset_bce = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);

    switch (type) {
        case WavemakerType::PISTON: {
            // Create the piston body and a linear motor
            double thickness = (bce_layers - 1) * m_spacing;
            ChVector3d piston_size(thickness, box_size.y(), box_size.z());
            ChVector3d piston_pos(-box_size.x() / 2 - thickness / 2 - m_spacing, 0, box_size.z() / 2);

            auto piston = chrono_types::make_shared<ChBody>();
            piston->SetPos(pos + piston_pos);
            piston->SetRot(QUNIT);
            piston->SetFixed(false);
            piston->EnableCollision(false);
            m_sys.AddBody(piston);

            utils::ChBodyGeometry geometry;
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, piston_size));
            geometry.CreateVisualizationAssets(piston, VisualizationType::COLLISION);

            auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
            motor->Initialize(piston, m_ground, ChFramed(piston->GetPos(), Q_ROTATE_Z_TO_X));
            motor->SetMotorFunction(piston_fun);
            m_sys.AddLink(motor);

            // Add piston as FSI body
            auto num_piston_bce = AddRigidBody(piston, geometry, true);

            if (m_verbose) {
                cout << "  Piston initialized at:   " << piston->GetPos() << endl;
                cout << "  Num. BCE markers:        " << num_piston_bce << endl;
            }

            return piston;
        }
        case WavemakerType::FLAP: {
            // Create the piston body and a linear motor
            double thickness = (bce_layers - 1) * m_spacing;
            ChVector3d flap_size(thickness, box_size.y(), box_size.z());
            ChVector3d flap_pos(-box_size.x() / 2 - thickness / 2 - m_spacing, 0, box_size.z() / 2);

            auto flap = chrono_types::make_shared<ChBody>();
            flap->SetPos(pos + flap_pos);
            flap->SetRot(QUNIT);
            flap->SetFixed(false);
            flap->EnableCollision(false);
            m_sys.AddBody(flap);

            utils::ChBodyGeometry geometry;
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, flap_size));
            geometry.CreateVisualizationAssets(flap, VisualizationType::COLLISION);

            auto rev_pos = pos + flap_pos - ChVector3d(0, 0, box_size.z() / 2);
            auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            motor->Initialize(flap, m_ground, ChFramed(rev_pos, Q_ROTATE_Z_TO_Y));
            motor->SetMotorFunction(piston_fun);
            m_sys.AddLink(motor);

            // Add flap as FSI body
            auto num_piston_bce = AddRigidBody(flap, geometry, true);

            if (m_verbose) {
                cout << "  Flap initialized at:   " << flap->GetPos() << endl;
                cout << "  Num. BCE markers:       " << num_piston_bce << endl;
            }

            return flap;
        }
    }
}

ChVector3i ChFsiProblemCartesian::Snap2Grid(const ChVector3d& point) {
    return ChVector3i((int)std::round(point.x() / m_spacing),  //
                      (int)std::round(point.y() / m_spacing),  //
                      (int)std::round(point.z() / m_spacing));
}

ChVector3d ChFsiProblemCartesian::Grid2Point(const ChVector3i& p) {
    return ChVector3d(m_spacing * p.x(), m_spacing * p.y(), m_spacing * p.z());
}

// ----------------------------------------------------------------------------

void ChFsiProblemCylindrical::Construct(double radius_inner,
                                        double radius_outer,
                                        double height,
                                        const ChVector3d& pos,
                                        bool bottom_wall,
                                        double side_walls) {
    if (m_verbose) {
        cout << "Construct cylinder ChFsiProblem" << endl;
    }

    bool filled = (radius_inner < 0.5 * m_spacing);

    // Number of BCE layers
    int bce_layers = m_sysFSI.GetNumBCELayers();

    // Number of points in each direction
    int Ir_start = std::round(radius_inner / m_spacing);
    int Nr = std::round((radius_outer - radius_inner) / m_spacing) + 1;
    int Nz = std::round(height / m_spacing) + 1;

    // 1. Generate SPH and bottom BCE points
    if (filled) {
        assert(Ir_start == 0);

        // Create central SPH particles
        for (int Iz = 0; Iz < Nz; Iz++) {
            m_sph.insert(ChVector3i(0, 0, Iz));
        }
        if (bottom_wall) {
            for (int Iz = 1; Iz <= bce_layers; Iz++) {
                m_bce.insert(ChVector3i(0, 0, -Iz));
            }
        }
    }

    for (int Ir = Ir_start; Ir < Ir_start + Nr; Ir++) {
        int Na = std::floor(CH_2PI * Ir);
        for (int Ia = 0; Ia < Na; Ia++) {
            for (int Iz = 0; Iz < Nz; Iz++) {
                m_sph.insert(ChVector3i(Ir, Ia, Iz));
            }
            if (bottom_wall) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(Ir, Ia, -Iz));
                }
            }
        }
    }

    // 2. Generate side BCE points
    if (side_walls) {
        // Exterior
        for (int Ir = Ir_start + Nr; Ir <= Ir_start + Nr + bce_layers; Ir++) {
            int Na = std::round(2 * CH_PI * Ir);
            for (int Ia = 0; Ia < Na; Ia++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(Ir, Ia, Iz));
                }
            }
        }

        // Interior
        if (!filled) {
            if (Ir_start <= bce_layers) {
                // Create central BCE markers
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(0, 0, Iz));
                }
            }

            for (int Ir = Ir_start - bce_layers; Ir <= Ir_start - 1; Ir++) {
                if (Ir <= 0)
                    continue;
                int Na = std::round(2 * CH_PI * Ir);
                for (int Ia = 0; Ia < Na; Ia++) {
                    for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                        m_bce.insert(ChVector3i(Ir, Ia, Iz));
                    }
                }
            }
        }
    }

    //// DEBUG
    ////for (const auto& p : m_bce) {
    ////    auto point = Grid2Point(p);
    ////    auto p1 = Snap2Grid(point);
    ////    cout << p << " | " << point << " | " << p1 << endl;
    ////}

    if (m_verbose) {
        cout << "  Num. SPH particles:      " << m_sph.size() << " (" << m_sph.size() << ")" << endl;
        cout << "  Num. bndry. BCE markers: " << m_bce.size() << " (" << m_bce.size() << ")" << endl;
    }

    m_offset_sph = pos;
    if (bottom_wall || side_walls)
        m_offset_bce = m_offset_sph;
}

size_t ChFsiProblemCylindrical::AddCylindricalContainer(double radius_inner,
                                                        double radius_outer,
                                                        double height,
                                                        const ChVector3d& pos,
                                                        bool bottom_wall,
                                                        bool side_walls,
                                                        bool top_wall) {
    if (m_verbose) {
        cout << "Construct cylinder container" << endl;
    }

    bool filled = (radius_inner < 0.5 * m_spacing);

    // Number of BCE layers
    int bce_layers = m_sysFSI.GetNumBCELayers();

    // Number of points in each direction
    int Ir_start = std::round(radius_inner / m_spacing);
    int Nr = std::round((radius_outer - radius_inner) / m_spacing) + 1;
    int Nz = std::round(height / m_spacing) + 1;

    // 1. Generate bottom BCE points
    if (bottom_wall) {
        if (filled) {
            assert(Ir_start == 0);
            for (int Iz = 1; Iz <= bce_layers; Iz++) {
                m_bce.insert(ChVector3i(0, 0, -Iz));
            }
        }

        for (int Ir = Ir_start; Ir < Ir_start + Nr; Ir++) {
            int Na = std::floor(CH_2PI * Ir);
            for (int Ia = 0; Ia < Na; Ia++) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(Ir, Ia, -Iz));
                }
            }
        }
    }

    // 2. Generate top BCE points
    if (top_wall) {
        if (filled) {
            assert(Ir_start == 0);
            for (int Iz = Nz; Iz < Nz + bce_layers; Iz++) {
                m_bce.insert(ChVector3i(0, 0, Iz));
            }
        }

        for (int Ir = Ir_start; Ir < Ir_start + Nr; Ir++) {
            int Na = std::floor(CH_2PI * Ir);
            for (int Ia = 0; Ia < Na; Ia++) {
                for (int Iz = Nz; Iz < Nz + bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(Ir, Ia, Iz));
                }
            }
        }
    }

    // 3. Generate side BCE points
    if (side_walls) {
        // Exterior
        for (int Ir = Ir_start + Nr; Ir <= Ir_start + Nr + bce_layers; Ir++) {
            int Na = std::round(2 * CH_PI * Ir);
            for (int Ia = 0; Ia < Na; Ia++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(Ir, Ia, Iz));
                }
            }
        }

        // Interior
        if (!filled) {
            if (Ir_start <= bce_layers) {
                // Create central BCE markers
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(0, 0, Iz));
                }
            }

            for (int Ir = Ir_start - bce_layers; Ir <= Ir_start - 1; Ir++) {
                if (Ir <= 0)
                    continue;
                int Na = std::round(2 * CH_PI * Ir);
                for (int Ia = 0; Ia < Na; Ia++) {
                    for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                        m_bce.insert(ChVector3i(Ir, Ia, Iz));
                    }
                }
            }
        }
    }

    if (m_verbose) {
        cout << "  Num. bndry. BCE markers: " << m_bce.size() << " (" << m_bce.size() << ")" << endl;
    }

    m_offset_bce = pos;

    return m_bce.size();
}

ChVector3i ChFsiProblemCylindrical::Snap2Grid(const ChVector3d& point) {
    auto r = std::sqrt(point.x() * point.x() + point.y() * point.y());
    auto theta = std::atan2(point.y(), point.x());
    if (theta < 0)
        theta += CH_2PI;

    auto ir = (int)std::round(r / m_spacing);
    auto d_theta = CH_2PI / std::floor(CH_2PI * ir);
    auto ia = (int)std::round(theta / d_theta);
    auto iz = (int)std::round(point.z() / m_spacing);

    return ChVector3i(ir, ia, iz);
}

ChVector3d ChFsiProblemCylindrical::Grid2Point(const ChVector3i& p) {
    if (p.x() == 0) {
        return ChVector3d(0, 0, p.z() * m_spacing);
    }

    auto r = p.x() * m_spacing;
    auto d_theta = CH_2PI / std::floor(CH_2PI * p.x());
    auto theta = p.y() * d_theta;
    auto z = p.z() * m_spacing;

    auto x = r * std::cos(theta);
    auto y = r * std::sin(theta);

    return ChVector3d(x, y, z);
}

}  // end namespace fsi
}  // end namespace chrono
