// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Definition of the SPH granular TERRAIN NODE (using Chrono::FSI).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.h"

#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

using std::cout;
using std::endl;

using namespace chrono::fsi;
using namespace rapidjson;

namespace chrono {
namespace vehicle {

// All obstacle bodies have this tag
static constexpr int tag_obstacles = 100;

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono system and set solver parameters
// - create the Chrono FSI system
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(double length, double width)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_SPH, length, width, ChContactMethod::SMC),
      m_terrain(nullptr),
      m_depth(0),
      m_active_box_size(0) {
    // Create the system and set default method-specific solver settings
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);

    // Default granular material properties
    m_radius = 0.01;
    m_density = 2000;
    m_cohesion = 0;
}

ChVehicleCosimTerrainNodeGranularSPH::ChVehicleCosimTerrainNodeGranularSPH(const std::string& specfile)
    : ChVehicleCosimTerrainNodeChrono(Type::GRANULAR_SPH, 0, 0, ChContactMethod::SMC),
      m_terrain(nullptr),
      m_active_box_size(0) {
    // Create systems
    m_system = new ChSystemSMC;

    // Solver settings independent of method type
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Set number of threads
    m_system->SetNumThreads(1);

    // Read SPH granular terrain parameters from provided specfile
    SetFromSpecfile(specfile);
}

ChVehicleCosimTerrainNodeGranularSPH::~ChVehicleCosimTerrainNodeGranularSPH() {
    delete m_terrain;
    delete m_system;
}

// -----------------------------------------------------------------------------

//// TODO: error checking
void ChVehicleCosimTerrainNodeGranularSPH::SetFromSpecfile(const std::string& specfile) {
    Document d;
    ReadSpecfile(specfile, d);

    if (d.HasMember("Patch data files")) {
        auto sph_filename = d["Patch data files"]["SPH particles file"].GetString();
        auto bce_filename = d["Patch data files"]["BCE markers file"].GetString();
        m_sph_filename = vehicle::GetDataFile(sph_filename);
        m_bce_filename = vehicle::GetDataFile(bce_filename);
        m_depth = 0;
        m_terrain_type = ConstructionMethod::FILES;
    } else {
        assert(d.HasMember("Patch dimensions"));
        m_dimX = d["Patch dimensions"]["Length"].GetDouble();
        m_dimY = d["Patch dimensions"]["Width"].GetDouble();
        m_depth = d["Patch dimensions"]["Depth"].GetDouble();
        m_terrain_type = ConstructionMethod::PATCH;
    }

    m_radius = d["Granular material"]["Radius"].GetDouble();
    m_density = d["Granular material"]["Density"].GetDouble();
    m_cohesion = d["Granular material"]["Cohesion"].GetDouble();
    m_init_height = m_depth;

    // Cache the name of the specfile (used when the CRMTerrain is actually constructed)
    m_specfile = specfile;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetGranularMaterial(double radius, double density, double cohesion) {
    m_radius = radius;
    m_density = density;
    m_cohesion = cohesion;
}

void ChVehicleCosimTerrainNodeGranularSPH::SetPropertiesSPH(const std::string& specfile, double depth) {
    m_depth = depth;
    m_init_height = m_depth;

    // Cache the name of the specfile (used when the CRMTerrain is actually constructed)
    m_specfile = specfile;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from OnInitialize.
// - adjust system settings
// - create the container body
// - add fluid particles
// - create obstacle bodies (if any)
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNodeGranularSPH::Construct() {
    if (m_verbose)
        cout << "[Terrain node] GRANULAR_SPH " << endl;

    // Create the SPH terrain
    double initSpace0 = 2 * m_radius;
    m_terrain = new CRMTerrain(*m_system, initSpace0);
    //////m_terrain->SetVerbose(true);
    ChSystemFsi& sysFSI = m_terrain->GetSystemFSI();

    // Let the FSI system read its parameters
    if (!m_specfile.empty())
        sysFSI.ReadParametersFromFile(m_specfile);

    // Reload simulation parameters to FSI system
    sysFSI.SetStepSize(m_step_size);
    sysFSI.SetConsistentDerivativeDiscretization(false, false);
    sysFSI.SetOutputLength(0);

    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));
    sysFSI.SetDensity(m_density);
    sysFSI.SetCohesionForce(m_cohesion);

    // Set the SPH method
    sysFSI.SetSPHMethod(SPHMethod::WCSPH);

    // Construct the CRMTerrain (generate SPH particles and boundary BCE markers)
    switch (m_terrain_type) {
        case ConstructionMethod::PATCH:
            m_terrain->Construct({m_dimX, m_dimY, m_depth}, VNULL, true, true);
            break;
        case ConstructionMethod::FILES:
            m_terrain->Construct(m_sph_filename, m_bce_filename, VNULL);
            break;
    }
    m_aabb_particles = m_terrain->GetSPHBoundingBox();

    /*
    //// TODO
    // Add all rigid obstacles
    // (ATTENTION: BCE markers for moving objects must be created after the SPH particles and after fixed BCE markers!)
    for (auto& b : m_obstacles) {
        auto mat = b.m_contact_mat.CreateMaterial(m_system->GetContactMethod());
        auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(GetChronoDataFile(b.m_mesh_filename), true, true);
        double mass;
        ChVector3d baricenter;
        ChMatrix33<> inertia;
        trimesh->ComputeMassProperties(true, mass, baricenter, inertia);

        auto body = chrono_types::make_shared<ChBody>();
        body->SetName("obstacle");
        body->SetTag(tag_obstacles);
        body->SetPos(b.m_init_pos);
        body->SetRot(b.m_init_rot);
        body->SetMass(mass * b.m_density);
        body->SetInertia(inertia * b.m_density);
        body->SetFixed(false);
        body->EnableCollision(true);

        auto trimesh_shape = chrono_types::make_shared<ChCollsionShapeTriangleMesh>(mat, trimesh,
                                                                                    false, false, m_radius_g);
        body->AddCollisionShape(trimesh_shape); body->GetCollisionModel()->SetFamily(2);

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(b.m_mesh_filename).stem());
        body->AddVisualShape(trimesh_shape, ChFrame<>());

        m_system->AddBody(body);

        // Add this body to the FSI system
        m_systemFSI->AddFsiBody(body);

        // Create BCE markers associated with trimesh
        std::vector<ChVector3d> point_cloud;
        m_systemFSI->CreateMeshPoints(*trimesh, (double)initSpace0, point_cloud);
        m_systemFSI->AddPointsBCE(body, point_cloud, ChFrame<>(), true);
    }
    */

    // Write file with terrain node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);
    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Patch dimensions" << endl;
    outf << "   X = " << m_dimX << "  Y = " << m_dimY << endl;
    outf << "   depth = " << m_depth << endl;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::CreateRigidProxy(unsigned int i) {
    ChSystemFsi& sysFSI = m_terrain->GetSystemFSI();

    // Get shape associated with the given object
    int i_shape = m_obj_map[i];

    // Create the proxy associated with the given object
    auto proxy = chrono_types::make_shared<ProxyBodySet>();

    // Create wheel proxy body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetTag(0);
    body->SetMass(m_load_mass[i]);
    body->SetFixed(true);  // proxy body always fixed
    body->EnableCollision(false);

    // Create visualization asset (use collision shapes)
    m_geometry[i_shape].CreateVisualizationAssets(body, VisualizationType::COLLISION);

    // Create collision shapes (only if obstacles are present)
    auto num_obstacles = m_obstacles.size();
    if (num_obstacles > 0) {
        for (auto& mesh : m_geometry[i_shape].coll_meshes)
            mesh.radius = m_radius;
        m_geometry[i_shape].CreateCollisionShapes(body, 1, m_method);
        body->GetCollisionModel()->SetFamily(1);
        body->GetCollisionModel()->DisallowCollisionsWith(1);
    }

    // Add this body to the Chrono and FSI systems
    m_system->AddBody(body);
    sysFSI.AddFsiBody(body);
    proxy->AddBody(body, 0);

    m_proxies[i] = proxy;

    // Create BCE markers associated with collision shapes
    for (const auto& box : m_geometry[i_shape].coll_boxes) {
        sysFSI.AddBoxBCE(body, ChFrame<>(box.pos, box.rot), box.dims, true);
    }
    for (const auto& sphere : m_geometry[i_shape].coll_spheres) {
        sysFSI.AddSphereBCE(body, ChFrame<>(sphere.pos, QUNIT), sphere.radius, true);
    }
    for (const auto& cyl : m_geometry[i_shape].coll_cylinders) {
        sysFSI.AddCylinderBCE(body, ChFrame<>(cyl.pos, cyl.rot), cyl.radius, cyl.length, true);
    }
    for (const auto& mesh : m_geometry[i_shape].coll_meshes) {
        std::vector<ChVector3d> point_cloud;
        sysFSI.CreateMeshPoints(*mesh.trimesh, (double)sysFSI.GetInitialSpacing(), point_cloud);
        sysFSI.AddPointsBCE(body, point_cloud, ChFrame<>(VNULL, QUNIT), true);
    }

    // Update dimension of FSI active domain based on shape AABB
    m_active_box_size = std::max(m_active_box_size, m_aabb[i_shape].Size().Length());
}

// Once all proxy bodies are created, complete construction of the underlying FSI system.
void ChVehicleCosimTerrainNodeGranularSPH::OnInitialize(unsigned int num_objects) {
    ChVehicleCosimTerrainNodeChrono::OnInitialize(num_objects);

    // Initialize the SPH terrain
    ChSystemFsi& sysFSI = m_terrain->GetSystemFSI();
    m_terrain->Initialize();

    // Initialize run-time visualization
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_VSG = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        vsys_VSG->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
        m_vsys = vsys_VSG;
#elif defined(CHRONO_OPENGL)
        m_vsys = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
        if (m_vsys) {
            m_vsys->SetTitle("Terrain Node (GranularSPH)");
            m_vsys->SetVerbose(false);
            m_vsys->SetSize(1280, 720);
            m_vsys->AddCamera(m_cam_pos, ChVector3d(0, 0, 0));
            m_vsys->SetCameraMoveScale(0.2f);
            m_vsys->EnableFluidMarkers(true);
            m_vsys->EnableBoundaryMarkers(false);
            m_vsys->EnableRigidBodyMarkers(true);
            m_vsys->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
            m_vsys->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
            m_vsys->SetSPHColorCallback(chrono_types::make_shared<HeightColorCallback>(
                ChColor(0.10f, 0.40f, 0.65f), m_aabb_particles.min.z(), m_aabb_particles.max.z()));
            m_vsys->SetImageOutputDirectory(m_node_out_dir + "/images");
            m_vsys->SetImageOutput(m_writeRT);
            m_vsys->AttachSystem(m_system);
            m_vsys->Initialize();
        }
    }
}

// Set state of proxy rigid body.
void ChVehicleCosimTerrainNodeGranularSPH::UpdateRigidProxy(unsigned int i, BodyState& rigid_state) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    proxy->bodies[0]->SetPos(rigid_state.pos);
    proxy->bodies[0]->SetPosDt(rigid_state.lin_vel);
    proxy->bodies[0]->SetRot(rigid_state.rot);
    proxy->bodies[0]->SetAngVelParent(rigid_state.ang_vel);
    proxy->bodies[0]->SetAngAccParent(VNULL);
}

// Collect resultant contact force and torque on rigid proxy body.
void ChVehicleCosimTerrainNodeGranularSPH::GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) {
    auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[i]);
    rigid_contact.point = ChVector3d(0, 0, 0);
    rigid_contact.force = proxy->bodies[0]->GetAccumulatedForce();
    rigid_contact.moment = proxy->bodies[0]->GetAccumulatedTorque();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::CreateMeshProxy(unsigned int i) {}

void ChVehicleCosimTerrainNodeGranularSPH::UpdateMeshProxy(unsigned int i, MeshState& mesh_state) {}

void ChVehicleCosimTerrainNodeGranularSPH::GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) {}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OnAdvance(double step_size) {
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_terrain->GetSystemFSI().DoStepDynamics_FSI();
        t += h;
    }
}

void ChVehicleCosimTerrainNodeGranularSPH::OnRender() {
    if (!m_vsys)
        return;

    if (m_track) {
        auto proxy = std::static_pointer_cast<ProxyBodySet>(m_proxies[0]);  // proxy for first object
        ChVector3d cam_point = proxy->bodies[0]->GetPos();                  // position of first body in proxy set
        m_vsys->UpdateCamera(m_cam_pos, cam_point);
    }

    auto ok = m_vsys->Render();
    if (!ok)
        MPI_Abort(MPI_COMM_WORLD, 1);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNodeGranularSPH::OnOutputData(int frame) {
    // Save SPH and BCE particles' information into CSV files
    m_terrain->GetSystemFSI().PrintParticleToFile(m_node_out_dir + "/simulation");
}

void ChVehicleCosimTerrainNodeGranularSPH::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "chpf", frame, 5);
    m_terrain->GetSystemFSI().SetParticleOutputMode(ChSystemFsi::OutputMode::CHPF);
    m_terrain->GetSystemFSI().WriteParticleFile(filename);
    if (m_obstacles.size() > 0) {
        filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
        // Include only obstacle bodies
        utils::WriteVisualizationAssets(
            m_system, filename, [](const ChBody& b) -> bool { return b.GetTag() == tag_obstacles; }, true);
    }
}

void ChVehicleCosimTerrainNodeGranularSPH::PrintMeshProxiesUpdateData(unsigned int i, const MeshState& mesh_state) {}

}  // end namespace vehicle
}  // end namespace chrono
