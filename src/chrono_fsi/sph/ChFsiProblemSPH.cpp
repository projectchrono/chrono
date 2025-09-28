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
#include <stdexcept>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#include "chrono_fsi/sph/utils/SphUtilsTypeConvert.cuh"

#include "chrono_thirdparty/stb/stb.h"
#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {
namespace sph {

// ----------------------------------------------------------------------------

ChFsiProblemSPH::ChFsiProblemSPH(double spacing, ChSystem* sys)
    : m_sysMBS(sys),
      m_spacing(spacing),
      m_initialized(false),
      m_offset_sph(VNULL),
      m_offset_bce(VNULL),
      m_bc_type({BCType::NONE, BCType::NONE, BCType::NONE}),
      m_verbose(false) {
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetFixed(true);

    // Create the underlying SPH system
    m_sysSPH = chrono_types::make_shared<ChFsiFluidSystemSPH>();

    // Set parameters for underlying SPH system
    m_sysSPH->SetInitialSpacing(spacing);
    m_sysSPH->SetKernelMultiplier(1.2);

    // Create the underlying FSI system
    m_sysFSI = chrono_types::make_shared<ChFsiSystemSPH>(sys, m_sysSPH.get());
    m_sysFSI->SetVerbose(m_verbose);

    // Create the surface reconstructor and set default parameters
    m_splashsurf = chrono_types::make_shared<ChFsiSplashsurfSPH>(*m_sysSPH);
    m_splashsurf->SetSmoothingLength(1.5);
    m_splashsurf->SetCubeSize(0.5);
    m_splashsurf->SetSurfaceThreshold(0.6);
}

void ChFsiProblemSPH::AttachMultibodySystem(ChSystem* sys) {
    m_sysMBS = sys;
    m_sysFSI->AttachMultibodySystem(sys);
}

void ChFsiProblemSPH::SetVerbose(bool verbose) {
    m_sysFSI->SetVerbose(verbose);
    m_verbose = verbose;
}

void ChFsiProblemSPH::SetCfdSPH(const ChFsiFluidSystemSPH::FluidProperties& fluid_props) {
    m_sysSPH->SetCfdSPH(fluid_props);
}

void ChFsiProblemSPH::SetElasticSPH(const ChFsiFluidSystemSPH::ElasticMaterialProperties& mat_props) {
    m_sysSPH->SetElasticSPH(mat_props);
}

void ChFsiProblemSPH::SetSPHParameters(const ChFsiFluidSystemSPH::SPHParameters& sph_params) {
    m_sysSPH->SetSPHParameters(sph_params);
}

void ChFsiProblemSPH::SetSplashsurfParameters(const ChFsiFluidSystemSPH::SplashsurfParameters& params) {
    m_splashsurf->SetSmoothingLength(params.smoothing_length);
    m_splashsurf->SetCubeSize(params.cube_size);
    m_splashsurf->SetSurfaceThreshold(params.surface_threshold);
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::AddRigidBody(std::shared_ptr<ChBody> body,
                                   std::shared_ptr<utils::ChBodyGeometry> geometry,
                                   bool check_embedded,
                                   bool use_grid) {
    if (m_verbose)
        cout << "Add rigid body '" << body->GetName() << "'" << endl;

    // Add the FSI rigid body to the underlying FSI system
    auto fsi_body = m_sysFSI->AddFsiBody(body, geometry, check_embedded);
    m_fsi_bodies[body] = fsi_body->index;
}

void ChFsiProblemSPH::AddRigidBodySphere(std::shared_ptr<ChBody> body,
                                         const ChVector3d& pos,
                                         double radius,
                                         bool use_grid) {
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_spheres.push_back(utils::ChBodyGeometry::SphereShape(pos, radius));
    AddRigidBody(body, geometry, true, use_grid);
}

void ChFsiProblemSPH::AddRigidBodyBox(std::shared_ptr<ChBody> body, const ChFramed& frame, const ChVector3d& size) {
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(frame.GetPos(), frame.GetRot(), size));
    AddRigidBody(body, geometry, true, true);
}

void ChFsiProblemSPH::AddRigidBodyCylinderX(std::shared_ptr<ChBody> body,
                                            const ChFramed& frame,
                                            double radius,
                                            double length,
                                            bool use_grid) {
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_cylinders.push_back(
        utils::ChBodyGeometry::CylinderShape(frame.GetPos(), frame.GetRotMat().GetAxisX(), radius, length));
    AddRigidBody(body, geometry, true, use_grid);
}

void ChFsiProblemSPH::AddRigidBodyMesh(std::shared_ptr<ChBody> body,
                                       const ChFramed& pos,
                                       const std::string& obj_filename,
                                       const ChVector3d& interior_point,
                                       double scale) {
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_meshes.push_back(
        utils::ChBodyGeometry::TrimeshShape(pos.GetPos(), pos.GetRot(), obj_filename, interior_point, scale));
    AddRigidBody(body, geometry, true, true);
}

size_t ChFsiProblemSPH::GetNumBCE(std::shared_ptr<ChBody> body) const {
    auto index = m_fsi_bodies.at(body);
    return m_sysSPH->m_bodies[index].bce_coords.size();
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::UseNodeDirections(NodeDirectionsMode mode) {
    m_sysFSI->UseNodeDirections(mode);
}

void ChFsiProblemSPH::SetBcePattern1D(BcePatternMesh1D pattern, bool remove_center) {
    m_sysSPH->SetBcePattern1D(pattern, remove_center);
}

void ChFsiProblemSPH::SetBcePattern2D(BcePatternMesh2D pattern, bool remove_center) {
    m_sysSPH->SetBcePattern2D(pattern, remove_center);
}

void ChFsiProblemSPH::AddFeaMesh(std::shared_ptr<fea::ChMesh> mesh, bool check_embedded) {
    if (m_verbose)
        cout << "Add FEA mesh '" << mesh->GetName() << "'" << endl;

    // Add 1D surfaces from given FEA mesh to the underlying FSI system
    auto fsi_mesh1D = m_sysFSI->AddFsiMesh1D(mesh, check_embedded);
    if (m_verbose) {
        if (fsi_mesh1D)
            cout << "  added " << fsi_mesh1D->GetNumElements() << " segments" << endl;
        else
            cout << "  mesh does not contain any 1D elements" << endl;
    }

    // Add 2D surfaces from given mesh to the underlying FSI system
    auto fsi_mesh2D = m_sysFSI->AddFsiMesh2D(mesh, check_embedded);
    if (m_verbose) {
        if (fsi_mesh2D)
            cout << "  added " << fsi_mesh2D->GetNumElements() << " faces" << endl;
        else
            cout << "  mesh does not contain any 2D elements" << endl;
    }
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::Initialize() {
    // Add ground body to MBS system
    if (m_sysMBS)
        m_sysMBS->AddBody(m_ground);

    // Prune SPH particles at grid locations that overlap with obstacles
    if (!m_sysSPH->m_bodies.empty()) {
        if (m_verbose)
            cout << "Remove SPH particles inside FSI solid volumes" << endl;

        for (auto& b : m_sysSPH->m_bodies)
            if (b.check_embedded)
                ProcessBody(b);

        for (auto m : m_sysSPH->m_meshes1D)
            if (m.check_embedded)
                ProcessFeaMesh1D(m);

        for (auto m : m_sysSPH->m_meshes2D)
            if (m.check_embedded)
                ProcessFeaMesh2D(m);

        if (m_verbose)
            cout << "  Num. SPH particles: " << m_sph.size() << endl;
    }

    // Keep track of the AABB of all SPH particles and BCE markers
    ChAABB aabb;

    // Convert SPH grid points to real coordinates and apply position offset
    // Include SPH particles in AABB
    std::vector<ChVector3d> sph_points;
    sph_points.reserve(m_sph.size());
    for (const auto& p : m_sph) {
        ChVector3d point = Grid2Point(p);
        point += m_offset_sph;
        sph_points.push_back(point);
        aabb += point;
    }

    m_sph_aabb = aabb;

    // Convert boundary grid points to real coordinates and apply position offset
    // Include boundary BCE markers in AABB
    std::vector<ChVector3d> bce_points;
    bce_points.reserve(m_bce.size());
    for (const auto& p : m_bce) {
        ChVector3d point = Grid2Point(p);
        point += m_offset_bce;
        bce_points.push_back(point);
        aabb += point;
    }

    // Callback for setting initial particle properties
    if (!m_props_cb)
        m_props_cb = chrono_types::make_shared<ParticlePropertiesCallback>();

    // Create SPH particles
    switch (m_sysSPH->GetPhysicsProblem()) {
        case PhysicsProblem::CFD: {
            for (const auto& pos : sph_points) {
                m_props_cb->set(*m_sysSPH, pos);
                m_sysSPH->AddSPHParticle(pos, m_props_cb->rho0, m_props_cb->p0, m_props_cb->mu0, m_props_cb->v0);
            }
            break;
        }
        case PhysicsProblem::CRM: {
            ChVector3d tau_offdiag(0);
            for (const auto& pos : sph_points) {
                m_props_cb->set(*m_sysSPH, pos);
                ChVector3d tau_diag(-m_props_cb->p0);
                m_sysSPH->AddSPHParticle(pos, m_props_cb->rho0, m_props_cb->p0, m_props_cb->mu0, m_props_cb->v0,  //
                                         tau_diag, tau_offdiag);
            }
            break;
        }
    }

    // Create boundary BCE markers
    // (ATTENTION: BCE markers must be created after the SPH particles!)
    m_sysSPH->AddBCEBoundary(bce_points, m_ground->GetFrameRefToAbs());

    // Update AABB using geometry of FSI solids
    for (const auto& b : m_sysSPH->m_bodies) {
        auto body_aabb = b.fsi_body->geometry->CalculateAABB();
        aabb += body_aabb.Transform(b.fsi_body->body->GetFrameRefToAbs());
    }
    for (const auto& m : m_sysSPH->m_meshes1D) {
        auto mesh_aabb = m.fsi_mesh->contact_surface->GetAABB();
        aabb += mesh_aabb;
    }
    for (const auto& m : m_sysSPH->m_meshes2D) {
        auto mesh_aabb = m.fsi_mesh->contact_surface->GetAABB();
        aabb += mesh_aabb;
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
        m_sysSPH->SetComputationalDomain(m_domain_aabb, m_bc_type);
    } else {
        // Calculate computational domain based on actual AABB of all markers
        int bce_layers = m_sysSPH->GetNumBCELayers();
        m_domain_aabb = ChAABB(aabb.min - bce_layers * m_spacing, aabb.max + bce_layers * m_spacing);
        m_sysSPH->SetComputationalDomain(m_domain_aabb, BC_NONE);
    }

    // Initialize the underlying FSI system
    m_sysFSI->Initialize();

    // Set SPH particle radius for the surface reconstructor
    m_splashsurf->SetParticleRadius(m_spacing / 2);

    m_initialized = true;
}

void ChFsiProblemSPH::PrintStats() const {
    m_sysSPH->PrintStats();
}

void ChFsiProblemSPH::PrintTimeSteps(const std::string& path) const {
    m_sysSPH->PrintTimeSteps(path);
}

// Check if specified point is inside a primitive shape of the given geometry.
// Test a shape volume enlarged by the specified envelope.
bool InsidePoint(const utils::ChBodyGeometry& geometry, const ChVector3d& p, double envelope) {
    for (const auto& sphere : geometry.coll_spheres) {
        auto radius = sphere.radius + envelope;
        if ((p - sphere.pos).Length2() <= radius * radius)
            return true;
    }

    for (const auto& box : geometry.coll_boxes) {
        auto pp = box.rot.RotateBack(p - box.pos);
        auto hdims = box.dims / 2 + envelope;
        if (std::abs(pp.x()) <= hdims.x() &&  //
            std::abs(pp.y()) <= hdims.y() &&  //
            std::abs(pp.z()) <= hdims.z())
            return true;
    }

    for (const auto& cyl : geometry.coll_cylinders) {
        auto pp = cyl.rot.RotateBack(p - cyl.pos);
        auto radius = cyl.radius + envelope;
        auto hlength = cyl.length / 2 + envelope;
        if (pp.x() * pp.x() + pp.y() * pp.y() <= radius * radius &&  //
            std::abs(pp.z()) <= hlength)
            return true;
    }

    return false;
}

// Prune SPH particles inside a body volume.
void ChFsiProblemSPH::ProcessBody(ChFsiFluidSystemSPH::FsiSphBody& b) {
    int num_removed = 0;

    // Traverse all body BCEs (with potential duplicates), transform into the ChFsiProblemSPH frame, express in grid
    // coordinates and calculate the (integer) body AABB.
    ChVector3i aabb_min(+std::numeric_limits<int>::max());
    ChVector3i aabb_max(-std::numeric_limits<int>::max());
    for (auto& p_abs : b.bce) {
        auto p_sph = p_abs - m_offset_sph;  // BCE point in ChFsiProblemSPH frame
        auto p_grd = Snap2Grid(p_sph);      // BCE point in integer grid coordinates
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
                ChVector3d p_loc = b.fsi_body->body->TransformPointParentToLocal(p_abs);
                // Check if inside a primitive shape
                if (InsidePoint(*b.fsi_body->geometry, p_loc, m_spacing))
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
    for (const auto& mesh : b.fsi_body->geometry->coll_meshes) {
        auto num_removed_mesh = ProcessBodyMesh(b, *mesh.trimesh, mesh.int_point);
        num_removed += num_removed_mesh;
    }

    if (m_verbose) {
        cout << "  Body '" << b.fsi_body->body->GetName() << "'" << endl;
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
int ChFsiProblemSPH::ProcessBodyMesh(ChFsiFluidSystemSPH::FsiSphBody& b,
                                     ChTriangleMeshConnected trimesh,
                                     const ChVector3d& interior_point) {
    // Transform mesh in ChFsiProblemSPH frame
    // (to address any roundoff issues that may result in a set of BCE markers that are not watertight)
    for (auto& v : trimesh.GetCoordsVertices()) {
        auto v_abs = b.fsi_body->body->TransformPointLocalToParent(v);  // vertex in absolute frame
        v = v_abs - m_offset_sph;                                       // vertex in FSIProblem frame
    }

    // BCE marker locations (in FSIProblem frame)
    auto bce = m_sysSPH->CreatePointsMesh(trimesh);

    // BCE marker locations in integer grid coordinates
    GridPoints gbce;
    for (auto& p : bce) {
        gbce.insert(Snap2Grid(p));
    }

    // Express the provided interior point in ChFsiProblemSPH grid coordinates
    auto c_abs = b.fsi_body->body->TransformPointLocalToParent(interior_point);  // interior point (abs frame)
    auto c_sph = c_abs - m_offset_sph;                                           // interior point (FSI problem frame)
    auto c = Snap2Grid(c_sph);                                                   // interior point (integer grid coords)

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
            cerr << "Mesh BCE set is NOT watertight!" << endl;
            throw std::invalid_argument("Mesh BCE set is NOT watertight!");
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
    int num_removed = 0;
    for (const auto& p : list) {
        auto iter = m_sph.find(p);
        if (iter != m_sph.end()) {
            m_sph.erase(iter);
            num_removed++;
        }
    }

    return num_removed;
}

void ChFsiProblemSPH::ProcessFeaMesh1D(ChFsiFluidSystemSPH::FsiSphMesh1D& m) {
    // If the mesh BCEs do not include the central marker, regenerate them. Otherwise, use existing BCEs
    std::vector<ChVector3d> bce;
    if (m_sysSPH->m_remove_center1D) {
        std::vector<ChVector3i> bce_ids;
        std::vector<ChVector3d> bce_coords;
        m_sysSPH->CreateBCEFsiMesh1D(m.fsi_mesh, m_sysSPH->m_pattern1D, false, bce_ids, bce_coords, bce);
    } else {
        bce = m.bce;
    }

    // Transform BCE marker locations in FSIProblem frame and generate BCE marker locations in integer grid coordinates
    GridPoints gbce;
    for (auto& p : bce) {
        ////auto p_offset = p - m_offset_sph;
        ////auto p_grid = Snap2Grid(p_offset);
        ////cout << p << "   |   " << p_offset << "  |   " << p_grid << endl;
        ////gbce.insert(p_grid);
        gbce.insert(Snap2Grid(p - m_offset_sph));
    }

    // Remove any SPH particles at the mesh BCE locations
    int num_removed = 0;
    for (const auto& p : gbce) {
        auto iter = m_sph.find(p);
        if (iter != m_sph.end()) {
            m_sph.erase(iter);
            num_removed++;
        }
    }

    if (m_verbose) {
        cout << "  1D FEA mesh" << endl;
        cout << "    Num. SPH particles removed: " << num_removed << endl;
    }
}

void ChFsiProblemSPH::ProcessFeaMesh2D(ChFsiFluidSystemSPH::FsiSphMesh2D& m) {
    // If the mesh BCEs do not include the central marker, regenerate them. Otherwise, use existing BCEs
    bool regenerate_bce = m_sysSPH->m_pattern2D == BcePatternMesh2D::CENTERED && m_sysSPH->m_remove_center2D;
    std::vector<ChVector3d> bce;
    if (regenerate_bce) {
        std::vector<ChVector3i> bce_ids;
        std::vector<ChVector3d> bce_coords;
        m_sysSPH->CreateBCEFsiMesh2D(m.fsi_mesh, BcePatternMesh2D::CENTERED, false, bce_ids, bce_coords, bce);
    } else {
        bce = m.bce;
    }

    // Transform BCE marker locations in FSIProblem frame and generate BCE marker locations in integer grid coordinates
    GridPoints gbce;
    for (auto& p : bce)
        gbce.insert(Snap2Grid(p - m_offset_sph));

    // Remove any SPH particles at the mesh BCE locations
    int num_removed = 0;
    for (const auto& p : gbce) {
        auto iter = m_sph.find(p);
        if (iter != m_sph.end()) {
            m_sph.erase(iter);
            num_removed++;
        }
    }

    if (m_verbose) {
        cout << "  2D FEA mesh" << endl;
        cout << "    Num. SPH particles removed: " << num_removed << endl;
    }
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::SetOutputLevel(OutputLevel output_level) {
    m_sysSPH->SetOutputLevel(output_level);
}

void ChFsiProblemSPH::SaveOutputData(double time, const std::string& sph_dir, const std::string& fsi_dir) {
    m_sysSPH->SaveParticleData(sph_dir);
    m_sysSPH->SaveSolidData(fsi_dir, time);
}

void ChFsiProblemSPH::SaveInitialMarkers(const std::string& out_dir) const {
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
    for (const auto& b : m_sysSPH->m_bodies) {
        for (const auto& p : b.bce_coords)
            obs_bce << p << std::endl;
    }
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::DoStepDynamics(double step) {
    m_sysFSI->DoStepDynamics(step);
}

// ----------------------------------------------------------------------------

const ChVector3d& ChFsiProblemSPH::GetFsiBodyForce(std::shared_ptr<ChBody> body) const {
    auto index = m_fsi_bodies.at(body);
    return m_sysFSI->GetFsiBodyForce(index);
}

const ChVector3d& ChFsiProblemSPH::GetFsiBodyTorque(std::shared_ptr<ChBody> body) const {
    auto index = m_fsi_bodies.at(body);
    return m_sysFSI->GetFsiBodyTorque(index);
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::CreateParticleRelocator() {
    SphParticleRelocator::DefaultProperties props;
    props.rho0 = m_sysSPH->GetDensity();
    props.mu0 = m_sysSPH->GetViscosity();

    m_relocator = chrono_types::make_unique<SphParticleRelocator>(*m_sysSPH->m_data_mgr, props);
}

void ChFsiProblemSPH::BCEShift(const ChVector3d& shift_dist) {
    ChAssertAlways(m_relocator);

    m_relocator->Shift(MarkerType::BCE_WALL, ToReal3(shift_dist));
}

void ChFsiProblemSPH::SPHShift(const ChVector3d& shift_dist) {
    ChAssertAlways(m_relocator);

    m_relocator->Shift(MarkerType::SPH_PARTICLE, ToReal3(shift_dist));
}

void ChFsiProblemSPH::SPHMoveAABB2AABB(const ChAABB& aabb_src, const ChIntAABB& aabb_dest) {
    ChAssertAlways(m_relocator);

    m_relocator->MoveAABB2AABB(MarkerType::SPH_PARTICLE, ToRealAABB(aabb_src), ToIntAABB(aabb_dest), Real(m_spacing));
}

void ChFsiProblemSPH::ForceProximitySearch() {
    m_sysSPH->m_force_proximity_search = true;
}

// ----------------------------------------------------------------------------

void ChFsiProblemSPH::WriteReconstructedSurface(const std::string& dir, const std::string& name, bool quiet) {
#ifndef CHRONO_HAS_SPLASHSURF
    std::cerr << "Warning: splashsurf not available; no mesh was generated." << std::endl;
    return;
#endif

    std::string in_filename = dir + "/" + name + ".json";
    std::string out_filename = dir + "/" + name + ".obj";

    m_splashsurf->WriteParticleFileJSON(in_filename);
    m_splashsurf->WriteReconstructedSurface(in_filename, out_filename, quiet);
}

// ============================================================================

ChFsiProblemCartesian::ChFsiProblemCartesian(double spacing, ChSystem* sys) : ChFsiProblemSPH(spacing, sys) {}

void ChFsiProblemCartesian::Construct(const std::string& sph_file, const std::string& bce_file, const ChVector3d& pos) {
    if (m_verbose) {
        cout << "Construct ChFsiProblemSPH from data files" << endl;
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

void ChFsiProblemCartesian::Construct(const ChVector3d& box_size, const ChVector3d& pos, int side_flags) {
    // Number of points in each direction
    int Nx = std::round(box_size.x() / m_spacing) + 1;
    int Ny = std::round(box_size.y() / m_spacing) + 1;
    int Nz = std::round(box_size.z() / m_spacing) + 1;

    // Reserve space for containers
    int num_sph = Nx * Ny * Nz;

    std::vector<ChVector3i> sph;
    sph.reserve(num_sph);

    // Generate SPH points
    for (int Ix = 0; Ix < Nx; Ix++) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Iz = 0; Iz < Nz; Iz++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above 0
            }
        }
    }

    // Insert in cached sets
    for (auto& p : sph) {
        m_sph.insert(p);
    }

    if (m_verbose) {
        cout << "Construct box ChFsiProblemSPH" << endl;
        cout << "  Particle grid size: " << Nx << " " << Ny << " " << Nz << endl;
        cout << "  Num. SPH particles: " << m_sph.size() << endl;
    }

    m_offset_sph = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);

    if (side_flags != BoxSide::NONE)
        AddBoxContainer(box_size, pos, side_flags);
}

void ChFsiProblemCartesian::Construct(const std::string& heightmap_file,
                                      double length,
                                      double width,
                                      const ChVector2d& height_range,
                                      double depth,
                                      bool uniform_depth,
                                      const ChVector3d& pos,
                                      int side_flags) {
    // Read the image file (request only 1 channel) and extract number of pixels
    STB cmap;
    if (!cmap.ReadFromFile(heightmap_file, 1)) {
        cerr << "Cannot open height map image file " << heightmap_file << endl;
        throw std::invalid_argument("Cannot open height map image file");
    }

    bool x_pos = (side_flags & static_cast<int>(BoxSide::X_POS)) != 0;
    bool x_neg = (side_flags & static_cast<int>(BoxSide::X_NEG)) != 0;
    bool y_pos = (side_flags & static_cast<int>(BoxSide::Y_POS)) != 0;
    bool y_neg = (side_flags & static_cast<int>(BoxSide::Y_NEG)) != 0;
    bool z_neg = (side_flags & static_cast<int>(BoxSide::Z_NEG)) != 0;

    int nx = cmap.GetWidth();   // number of pixels in x direction
    int ny = cmap.GetHeight();  // number of pixels in y direction

    double dx = length / (nx - 1);
    double dy = width / (ny - 1);

    // Create a matrix with gray levels (g = 0x0000 is black, g = 0xffff is white)
    unsigned short g_min = std::numeric_limits<unsigned short>::max();
    unsigned short g_max = 0;
    ChMatrixDynamic<unsigned short> gmap(nx, ny);
    for (int ix = 0; ix < nx; ix++) {
        for (int iy = 0; iy < ny; iy++) {
            auto gray = cmap.Gray(ix, iy);
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

    ////cout << "g_min = " << g_min << endl;
    ////cout << "g_max = " << g_max << endl;

    // Number of particles in each direction
    int Nx = (int)std::round(length / m_spacing);
    int Ny = (int)std::round(width / m_spacing);
    double Dx = length / Nx;
    double Dy = width / Ny;
    Nx += 1;
    Ny += 1;

    // Number of particles in Z direction over specified depth
    int Nz = (int)std::round(depth / m_spacing) + 1;

    // Number of BCE layers
    int bce_layers = m_sysSPH->GetNumBCELayers();

    // Reserve space for containers
    std::vector<ChVector3i> sph;
    std::vector<ChVector3i> bce;
    sph.reserve(Nx * Ny * Nz);            // underestimate if not uniform depth
    bce.reserve(bce_layers * (Nx * Ny));  // underestimate if creating side walls

    // Generate SPH and bottom BCE points
    for (int Ix = 0; Ix < Nx; Ix++) {
        double x = Ix * Dx;
        // x location between pixels ix1 and ix2
        int ix1 = (Ix == Nx - 1) ? nx - 2 : (int)std::floor(x / dx);
        int ix2 = (Ix == 0) ? 1 : (int)std::ceil(x / dx);
        // weight for bi-linear interpolation
        double wx = (ix2 * dx - x) / dx;

        for (int Iy = 0; Iy < Ny; Iy++) {
            double y = Iy * Dy;
            // y location between pixels iy1 and iy2
            int iy1 = (Iy == Ny - 1) ? ny - 2 : (int)std::floor(y / dy);
            int iy2 = (Iy == 0) ? 1 : (int)std::ceil(y / dy);
            // weight for bi-linear interpolation
            double wy = (iy2 * dy - y) / dy;

            // Calculate surface height at current location (bi-linear interpolation)
            auto h1 = wx * gmap(ix1, iy1) + (1 - wx) * gmap(ix2, iy1);
            auto h2 = wx * gmap(ix1, iy2) + (1 - wx) * gmap(ix2, iy2);
            auto z = height_range[0] + (wy * h1 + (1 - wy) * h2) * h_scale;
            int Iz = (int)std::round(z / m_spacing);

            // Create SPH particle locations below current point
            int nz = uniform_depth ? Nz : Iz + Nz;
            for (int k = 0; k < nz; k++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));
                Iz--;
            }

            // Create BCE marker locations below the SPH particles
            if (z_neg) {
                for (int k = 0; k < bce_layers; k++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                    Iz--;
                }
            }
        }
    }

    ////cout << "IZmin = " << Iz_min << endl;
    ////cout << "IZmax = " << Iz_max << endl;

    // Generate side BCE points
    auto nz_min = Iz_min - Nz - bce_layers;
    auto nz_max = Iz_max + bce_layers;

    if (x_neg) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int k = nz_min; k < nz_max; k++) {
                    bce.push_back(ChVector3i(Ix, Iy, k));
                }
            }
        }
    }

    if (x_pos) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int k = nz_min; k < nz_max; k++) {
                    bce.push_back(ChVector3i(Nx - 1 - Ix, Iy, k));
                }
            }
        }
    }

    if (y_neg) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int k = nz_min; k < nz_max; k++) {
                    bce.push_back(ChVector3i(Ix, Iy, k));
                }
            }
        }
    }

    if (y_pos) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int k = nz_min; k < nz_max; k++) {
                    bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, k));
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
        cout << "Construct ChFsiProblemSPH from heightmap file" << endl;
        cout << "  Heightmap filename: " << heightmap_file << endl;
        cout << "  Num. SPH particles: " << m_sph.size() << endl;
        cout << "  Num. BCE markers:   " << m_bce.size() << endl;
    }

    m_offset_sph = pos - ChVector3d(length / 2, width / 2, 0);
    if (side_flags != BoxSide::NONE)
        m_offset_bce = m_offset_sph;
}

size_t ChFsiProblemCartesian::AddBoxContainer(const ChVector3d& box_size,  // box dimensions
                                              const ChVector3d& pos,       // reference positions
                                              int side_flags               // sides for which BCE markers are created
) {
    if (m_verbose) {
        cout << "Construct box container" << endl;
    }

    int bce_layers = m_sysSPH->GetNumBCELayers();

    int Nx = std::round(box_size.x() / m_spacing) + 1;
    int Ny = std::round(box_size.y() / m_spacing) + 1;
    int Nz = std::round(box_size.z() / m_spacing) + 1;

    bool x_pos = (side_flags & static_cast<int>(BoxSide::X_POS)) != 0;
    bool x_neg = (side_flags & static_cast<int>(BoxSide::X_NEG)) != 0;
    bool y_pos = (side_flags & static_cast<int>(BoxSide::Y_POS)) != 0;
    bool y_neg = (side_flags & static_cast<int>(BoxSide::Y_NEG)) != 0;
    bool z_pos = (side_flags & static_cast<int>(BoxSide::Z_POS)) != 0;
    bool z_neg = (side_flags & static_cast<int>(BoxSide::Z_NEG)) != 0;

    int num_bce = 0;
    if (x_pos)
        num_bce += Ny * Nz * bce_layers;
    if (x_neg)
        num_bce += Ny * Nz * bce_layers;
    if (y_pos)
        num_bce += Nz * Nx * bce_layers;
    if (y_neg)
        num_bce += Nz * Nx * bce_layers;
    if (z_pos)
        num_bce += Nx * Ny * bce_layers;
    if (z_neg)
        num_bce += Nx * Ny * bce_layers;

    std::vector<ChVector3i> bce;
    bce.reserve(num_bce);

    // Bottom/top BCE points
    if (z_neg) {
        for (int Ix = 0; Ix < Nx; Ix++) {
            for (int Iy = 0; Iy < Ny; Iy++) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, -Iz));
                }
            }
        }
    }
    if (z_pos) {
        for (int Ix = 0; Ix < Nx; Ix++) {
            for (int Iy = 0; Iy < Ny; Iy++) {
                for (int Iz = 1; Iz <= bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Nz - 1 + Iz));
                }
            }
        }
    }

    // Left/right BCE points
    if (x_neg) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                }
            }
        }
    }
    if (x_pos) {
        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Ix = -bce_layers; Ix < 0; Ix++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Nx - 1 - Ix, Iy, Iz));
                }
            }
        }
    }

    // Front/back BCE points
    if (y_neg) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                }
            }
        }
    }
    if (y_pos) {
        for (int Ix = -bce_layers; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -bce_layers; Iy < 0; Iy++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, Iz));
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

ChVector3i ChFsiProblemCartesian::Snap2Grid(const ChVector3d& point) {
    return ChVector3i((int)std::round(point.x() / m_spacing),  //
                      (int)std::round(point.y() / m_spacing),  //
                      (int)std::round(point.z() / m_spacing));
}

ChVector3d ChFsiProblemCartesian::Grid2Point(const ChVector3i& p) {
    return ChVector3d(m_spacing * p.x(), m_spacing * p.y(), m_spacing * p.z());
}

// ============================================================================

ChFsiProblemWavetank::ChFsiProblemWavetank(double spacing, ChSystem* sys)
    : ChFsiProblemCartesian(spacing, sys), m_periodic_BC(false), m_end_wall(true) {}

void ChFsiProblemWavetank::SetProfile(std::shared_ptr<Profile> profile, bool end_wall) {
    m_profile = profile;
    m_end_wall = end_wall;
}

std::shared_ptr<ChBody> ChFsiProblemWavetank::ConstructWaveTank(
    WavemakerType type,                    // wave generator type
    const ChVector3d& pos,                 // reference position
    const ChVector3d& box_size,            // box dimensions
    double depth,                          // fluid depth
    std::shared_ptr<ChFunction> actuation  // actuation function
) {
    // Number of BCE layers
    int bce_layers = m_sysSPH->GetNumBCELayers();

    int Nx = std::round(box_size.x() / m_spacing) + 1;
    int Ny = std::round(box_size.y() / m_spacing) + 1;

    int Nzc = std::round(box_size.z() / m_spacing) + 1;
    int Nzf = std::round(depth / m_spacing) + 1;

    // Reserve space for containers
    int num_sph = Nx * Ny * Nzf;                              // overestimate, depending on bottom profile
    int num_bce = Nx * Ny * bce_layers;                       // BCE on bottom
    num_bce += (Nx + 2 * bce_layers) * bce_layers * Nzc * 2;  // BCE at -y and +y
    num_bce += (Ny + 2 * bce_layers) * bce_layers * Nzc;      // BCE at +x

    std::vector<ChVector3i> sph;
    std::vector<ChVector3i> bce;
    sph.reserve(num_sph);
    bce.reserve(num_bce);

    // Generate SPH, bottom BCE, and side BCE points
    int Iy0 = m_periodic_BC ? 0 : bce_layers;  // lateral BCE width (no lateral walls if periodic BC)
    int Iz0 = 0;                               // fluid start index at bottom

    for (int Ix = 0; Ix < Nx; Ix++) {
        double x = Ix * m_spacing;                   // current downstream location
        double z = m_profile ? (*m_profile)(x) : 0;  // bottom height

        ////std::cout << x << "  " << z << std::endl;

        Iz0 = std::round(z / m_spacing);  // fluid start index at bottom

        for (int Iy = 0; Iy < Ny; Iy++) {
            for (int Iz = Iz0; Iz < Nzf; Iz++) {
                sph.push_back(ChVector3i(Ix, Iy, Iz));  // SPH particles above bottom height
            }
            for (int Iz = 1; Iz <= bce_layers; Iz++) {
                bce.push_back(ChVector3i(Ix, Iy, Iz0 - Iz));  // BCE markers below bottom height
            }
        }

        for (int Iy = -Iy0; Iy < 0; Iy++) {
            for (int Iz = Iz0 - bce_layers; Iz < Nzc + bce_layers; Iz++) {
                bce.push_back(ChVector3i(Ix, Iy, Iz));           // BCE markers on positive side
                bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, Iz));  // BCE markers on negative side
            }
        }
    }

    // Generate BCE points around wavemaker body (extend to negative Ix)
    for (int Ix = -bce_layers; Ix < 0; Ix++) {
        for (int Iy = -Iy0; Iy < Ny + Iy0; Iy++) {
            for (int Iz = 1; Iz <= bce_layers; Iz++) {
                bce.push_back(ChVector3i(Ix, Iy, -Iz));  // BCE markers below bottom height
            }
        }

        for (int Iy = -Iy0; Iy < 0; Iy++) {
            for (int Iz = -bce_layers; Iz < Nzc + bce_layers; Iz++) {
                bce.push_back(ChVector3i(Ix, Iy, Iz));           // BCE markers on positive side
                bce.push_back(ChVector3i(Ix, Ny - 1 - Iy, Iz));  // BCE markers on negative side
            }
        }
    }

    // Generate end wall BCE points (force end wall if no profile, i.e. if flat bottom)
    if (!m_profile)
        m_end_wall = true;

    if (m_end_wall) {
        for (int Ix = Nx; Ix < Nx + bce_layers; Ix++) {
            for (int Iy = -Iy0; Iy < Ny + Iy0; Iy++) {
                for (int Iz = Iz0 - bce_layers; Iz < Nzc + bce_layers; Iz++) {
                    bce.push_back(ChVector3i(Ix, Iy, Iz));
                }
            }
        }
    }

    // Insert particles and markers in cached sets
    for (auto& p : sph) {
        m_sph.insert(p);
    }
    for (auto& p : bce) {
        m_bce.insert(p);
    }

    if (m_verbose) {
        cout << "Construct wave tank" << endl;
        cout << "  Particle grid size:      " << Nx << " " << Ny << " " << Nzf << endl;
        cout << "  Num. SPH particles:      " << m_sph.size() << " (" << sph.size() << ")" << endl;
        cout << "  Num. bndry. BCE markers: " << m_bce.size() << " (" << bce.size() << ")" << endl;
    }

    m_offset_sph = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);
    m_offset_bce = pos - ChVector3d(box_size.x() / 2, box_size.y() / 2, 0);

    // If using periodic BC in lateral direction, explicitly set computational domain and BC types
    if (m_periodic_BC) {
        auto y_min = (0 + 0.5) * m_spacing;
        auto y_max = ((Ny - 1) - 0.5) * m_spacing;
        auto aabb_min = ChVector3d(-bce_layers * m_spacing, y_min, -bce_layers * m_spacing);
        auto aabb_max = ChVector3d((Nx + bce_layers) * m_spacing, y_max, (Nzc + bce_layers) * m_spacing);
        m_domain_aabb = ChAABB(m_offset_sph + aabb_min, m_offset_sph + aabb_max);
        m_bc_type = BC_Y_PERIODIC;
    }

    // Visualization assets for wavetank container
    double thickness = (bce_layers - 1) * m_spacing;
    /*
    double length = box_size.x() + 2 * thickness;
    double width = box_size.y() + 2 * thickness;
    double height = box_size.z() + 2 * thickness;

    ChColor color(0.3f, 0.3f, 0.3f);

    {
        ChVector3d size(length, thickness, height);
        ChVector3d loc(0, -box_size.y() / 2 - m_spacing, box_size.z() / 2 + m_spacing);
        auto shape = chrono_types::make_shared<ChVisualShapeBox>(size);
        shape->SetColor(color);
        m_ground->AddVisualShape(shape, ChFramed(pos + loc, QUNIT));
    }

    {
        ChVector3d size(length, thickness, height);
        ChVector3d loc(0, box_size.y() / 2 + m_spacing, box_size.z() / 2 + m_spacing);
        auto shape = chrono_types::make_shared<ChVisualShapeBox>(size);
        shape->SetColor(color);
        m_ground->AddVisualShape(shape, ChFramed(pos + loc, QUNIT));
    }

    if (m_end_wall) {
        ChVector3d size(thickness, width, height - Iz0 * m_spacing);
        ChVector3d loc(box_size.x() / 2 + thickness / 2 + m_spacing, 0, Iz0 * m_spacing / 2 + box_size.z() / 2 +
    m_spacing); auto shape = chrono_types::make_shared<ChVisualShapeBox>(size); shape->SetColor(color);
        m_ground->AddVisualShape(shape, ChFramed(pos + loc, QUNIT));
    }
    */

    // Create wavemaker body
    m_wavemaker_size = ChVector3d(thickness, box_size.y(), box_size.z());
    m_wavemaker_pos = ChVector3d(-box_size.x() / 2 - thickness / 2 - m_spacing, 0, box_size.z() / 2);

    switch (type) {
        case WavemakerType::PISTON: {
            // Create the piston body and a linear motor
            m_wavemaker_body = chrono_types::make_shared<ChBody>();
            m_wavemaker_body->SetName("internal_wavemaker_piston");
            m_wavemaker_body->SetPos(pos + m_wavemaker_pos);
            m_wavemaker_body->SetRot(QUNIT);
            m_wavemaker_body->SetFixed(false);
            m_wavemaker_body->EnableCollision(false);

            m_wavemaker_motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
            m_wavemaker_motor->Initialize(m_wavemaker_body, m_ground,
                                          ChFramed(m_wavemaker_body->GetPos(), Q_ROTATE_Z_TO_X));
            m_wavemaker_motor->SetMotorFunction(actuation);

            break;
        }
        case WavemakerType::FLAP: {
            // Create the flap body and a rotational motor
            auto rev_pos = pos + m_wavemaker_pos - ChVector3d(0, 0, box_size.z() / 2);

            m_wavemaker_pos.z() -= thickness / 2;
            m_wavemaker_size.z() += thickness;

            m_wavemaker_body = chrono_types::make_shared<ChBody>();
            m_wavemaker_body->SetName("internal_wavemaker_flap");
            m_wavemaker_body->SetPos(pos + m_wavemaker_pos);
            m_wavemaker_body->SetRot(QUNIT);
            m_wavemaker_body->SetFixed(false);
            m_wavemaker_body->EnableCollision(false);

            m_wavemaker_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            m_wavemaker_motor->Initialize(m_wavemaker_body, m_ground, ChFramed(rev_pos, Q_ROTATE_Z_TO_Y));
            m_wavemaker_motor->SetMotorFunction(actuation);

            break;
        }
    }

    if (m_verbose) {
        cout << "  Body initialized at:   " << m_wavemaker_body->GetPos() << endl;
    }

    return m_wavemaker_body;
}

void ChFsiProblemWavetank::Initialize() {
    if (m_sysMBS) {
        // Add wavemaker body and motor to multibody system
        m_sysMBS->AddBody(m_wavemaker_body);
        m_sysMBS->AddLink(m_wavemaker_motor);

        // Add wavemaker body as FSI body
        auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, m_wavemaker_size));
        geometry->CreateVisualizationAssets(m_wavemaker_body, VisualizationType::COLLISION);

        AddRigidBody(m_wavemaker_body, geometry, true);
    }

    // Complete initialization of the FSI problem
    ChFsiProblemSPH::Initialize();
}

// ============================================================================

ChFsiProblemCylindrical::ChFsiProblemCylindrical(double spacing, ChSystem* sys) : ChFsiProblemSPH(spacing, sys) {}

void ChFsiProblemCylindrical::Construct(double radius_inner,
                                        double radius_outer,
                                        double height,
                                        const ChVector3d& pos,
                                        int side_flags) {
    bool filled = (radius_inner < 0.5 * m_spacing);

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
    }

    for (int Ir = Ir_start; Ir < Ir_start + Nr; Ir++) {
        int Na = std::floor(CH_2PI * Ir);
        for (int Ia = 0; Ia < Na; Ia++) {
            for (int Iz = 0; Iz < Nz; Iz++) {
                m_sph.insert(ChVector3i(Ir, Ia, Iz));
            }
        }
    }

    if (m_verbose) {
        cout << "Construct cylinder ChFsiProblemSPH;  num. SPH particles: " << m_sph.size() << " (" << m_sph.size()
             << ")" << endl;
    }

    m_offset_sph = pos;

    if (side_flags != CylSide::NONE)
        AddCylindricalContainer(radius_inner, radius_outer, height, pos, side_flags);
}

size_t ChFsiProblemCylindrical::AddCylindricalContainer(double radius_inner,
                                                        double radius_outer,
                                                        double height,
                                                        const ChVector3d& pos,
                                                        int side_flags) {
    bool filled = (radius_inner < 0.5 * m_spacing);

    bool z_pos = (side_flags & static_cast<int>(CylSide::Z_POS)) != 0;
    bool z_neg = (side_flags & static_cast<int>(CylSide::Z_NEG)) != 0;
    bool side_int = (side_flags & static_cast<int>(CylSide::SIDE_INT)) != 0;
    bool side_ext = (side_flags & static_cast<int>(CylSide::SIDE_EXT)) != 0;

    // Number of BCE layers
    int bce_layers = m_sysSPH->GetNumBCELayers();

    // Number of points in each direction
    int Ir_start = std::round(radius_inner / m_spacing);
    int Nr = std::round((radius_outer - radius_inner) / m_spacing) + 1;
    int Nz = std::round(height / m_spacing) + 1;

    // 1. Generate bottom BCE points
    if (z_neg) {
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
    if (z_pos) {
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

    // 3. Generate exterior BCE points
    if (side_ext) {
        for (int Ir = Ir_start + Nr; Ir <= Ir_start + Nr + bce_layers; Ir++) {
            int Na = std::round(2 * CH_PI * Ir);
            for (int Ia = 0; Ia < Na; Ia++) {
                for (int Iz = -bce_layers; Iz < Nz + bce_layers; Iz++) {
                    m_bce.insert(ChVector3i(Ir, Ia, Iz));
                }
            }
        }
    }

    // 3. Generate interior BCE points
    if (side_int && !filled) {
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

    if (m_verbose) {
        cout << "Construct cylinder container;  num. bndry. BCE markers: " << m_bce.size() << " (" << m_bce.size()
             << ")" << endl;
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

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono
