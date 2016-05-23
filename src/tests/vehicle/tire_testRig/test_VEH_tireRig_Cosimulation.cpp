// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a Chrono::Parallel system for the granular terrain.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO: for use with granular terrain, cannot use a node cloud of proxy bodies

#define RIG_NODE_RANK 0
#define TERRAIN_NODE_RANK 1

#include <omp.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>
#include "mpi.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Value of gravitational acceleration (Z direction), common on both systems
static const double gacc = -9.81;

// Specify whether or not contact coefficients are based on material properties
static const bool use_mat_properties = false;

// Number of OpenMP threads on each MPI node
static const int nthreads_rignode = 2;
static const int nthreads_terrainnode = 2;

// =============================================================================
// RIG NODE CLASS
// =============================================================================

class RigNode {
  public:
    RigNode(int num_threads);
    ~RigNode();

    void SetOutputFile(const std::string& name);

    void Initialize();
    void Synchronize(int step_number, double time);
    void Advance(double step_size);

    void OutputData();

  private:
    ChSystemDEM* m_system;  ///< containing system
    double m_step_size;     ///< integration step size

    std::shared_ptr<ChBody> m_ground;  ///< ground body
    std::shared_ptr<ChBody> m_rim;     ///< wheel rim body

    std::shared_ptr<ChLinkLockPlanePlane> m_plane_plane;  ///< ground-rim joint

    std::shared_ptr<ChDeformableTire> m_tire;                       ///< deformable tire
    std::shared_ptr<fea::ChLoadContactSurfaceMesh> m_contact_load;  ///< tire contact surface

    double m_init_vel;  ///< initial wheel forward linear velocity

    std::ofstream m_outf;  ///< output file stream

    // Private methods
    void PrintLowestNode();
    void PrintLowestVertex(const std::vector<ChVector<>>& vert_pos, const std::vector<ChVector<>>& vert_vel);
    void PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indeces);
};

// -----------------------------------------------------------------------------
// Construction of the rig node:
// - create the (sequential) Chrono system and set solver parameters
// - create (but do not initialize) the rig mechanism bodies and joints
// - create (but do not initialize) the tire
// - send information on tire contact material
// -----------------------------------------------------------------------------
RigNode::RigNode(int num_threads) {
    // ----------------
    // Model parameters
    // ----------------

    m_step_size = 1e-4;

    double rim_mass = 100;            //// 0.1;
    ChVector<> rim_inertia(1, 1, 1);  //// (1e-2, 1e-2, 1e-2);

    m_init_vel = 0;  //// 20;

    // ----------------------------------
    // Create the (sequential) DEM system
    // ----------------------------------
    m_system = new ChSystemDEM;
    m_system->Set_G_acc(ChVector<>(0, 0, gacc));

    // Set number threads
    m_system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

#ifdef CHRONO_MKL
    // Solver settings
    ChSolverMKL* mkl_solver_stab = new ChSolverMKL;
    ChSolverMKL* mkl_solver_speed = new ChSolverMKL;
    m_system->ChangeSolverStab(mkl_solver_stab);
    m_system->ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);
#else
    // Solver settings
    m_system->SetMaxItersSolverSpeed(100);
    m_system->SetMaxItersSolverStab(100);
    m_system->SetSolverType(ChSystem::SOLVER_SOR);
    m_system->SetTol(1e-10);
    m_system->SetTolForce(1e-8);
#endif

    // Integrator settings
    m_system->SetIntegrationType(ChSystem::INT_HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(5e-05, 1.8e00);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetScaling(true);
    integrator->SetVerbose(true);

    // -------------------------------
    // Create the rig mechanism bodies
    // -------------------------------

    // Create ground body.
    m_ground = std::make_shared<ChBody>();
    m_system->AddBody(m_ground);
    m_ground->SetBodyFixed(true);

    // Create the rim body.
    m_rim = std::make_shared<ChBody>();
    m_system->AddBody(m_rim);
    m_rim->SetMass(rim_mass);
    m_rim->SetInertiaXX(rim_inertia);

    // -------------------------------
    // Create the rig mechanism joints
    // -------------------------------

    // Plane contraint on the rim
    m_plane_plane = std::make_shared<ChLinkLockPlanePlane>();
    m_system->AddLink(m_plane_plane);

    //// TODO: complete construction of the test rig

    // ---------------
    // Create the tire
    // ---------------

    std::string ancftire_file("hmmwv/tire/HMMWV_ANCFTire.json");

    m_tire = std::make_shared<ANCFTire>(vehicle::GetDataFile(ancftire_file));
    m_tire->EnablePressure(true);
    m_tire->EnableContact(true);
    m_tire->EnableRimConnection(true);
    m_tire->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);

    // -------------------------------------
    // Send tire contact material properties
    // -------------------------------------

    auto contact_mat = m_tire->GetContactMaterial();
    float mat_props[8] = {m_tire->GetCoefficientFriction(),
                          m_tire->GetCoefficientRestitution(),
                          m_tire->GetYoungModulus(),
                          m_tire->GetPoissonRatio(),
                          m_tire->GetKn(),
                          m_tire->GetGn(),
                          m_tire->GetKt(),
                          m_tire->GetGt()};

    MPI_Send(mat_props, 8, MPI_FLOAT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    std::cout << "[Rig node    ] friction = " << mat_props[0] << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigNode::~RigNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::SetOutputFile(const std::string& name) {
    m_outf.open(name, std::ios::out | std::ios::app);
    m_outf.precision(7);
    m_outf << std::scientific;
}

// -----------------------------------------------------------------------------
// Initialization of the rig node:
// - receive terrain height
// - initialize the mechanism bodies
// - initialize the mechanism joints
// - initialize the tire and extract contact surface
// - send information on tire mesh topology (number verices and triangles)
// -----------------------------------------------------------------------------
void RigNode::Initialize() {
    // ------------------------------
    // Receive initial terrain height
    // ------------------------------

    double init_height;
    MPI_Status status;
    MPI_Recv(&init_height, 1, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    std::cout << "[Rig node    ] Received init_height = " << init_height << std::endl;

    // -----------------------------------
    // Initialize the rig mechanism bodies
    // -----------------------------------

    // Initialize rim body.
    double tire_radius = m_tire->GetRadius();

    m_rim->SetPos(ChVector<>(0, 0, init_height + tire_radius));
    m_rim->SetRot(QUNIT);
    m_rim->SetPos_dt(ChVector<>(m_init_vel, 0, 0));
    m_rim->SetWvel_loc(ChVector<>(0, m_init_vel / tire_radius, 0));

    // -----------------------------------
    // Initialize the rig mechanism joints
    // -----------------------------------

    m_plane_plane->Initialize(m_ground, m_rim, ChCoordsys<>(m_rim->GetPos(), Q_from_AngX(CH_C_PI_2)));

    //// TODO: complete initialization of the test rig

    // ---------------
    // Initialize tire
    // ---------------

    m_tire->Initialize(m_rim, LEFT);

    // Create a mesh load for contact forces and add it to the tire's load container.
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire->GetContactSurface());
    m_contact_load = std::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire->GetLoadContainer()->Add(m_contact_load);

    // Mark completion of system construction
    m_system->SetupInitial();

    // ---------------------------------------
    // Send tire contact surface specification
    // ---------------------------------------

    unsigned int surf_props[2];
    surf_props[0] = contact_surface->GetNumVertices();
    surf_props[1] = contact_surface->GetNumTriangles();
    MPI_Send(surf_props, 2, MPI_UNSIGNED, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    std::cout << "[Rig node    ] vertices = " << surf_props[0] << "  triangles = " << surf_props[1] << std::endl;
}

// -----------------------------------------------------------------------------
// Synchronization of the rig node:
// - extract and send tire mesh vertex states
// - receive and apply vertex contact forces
// -----------------------------------------------------------------------------
void RigNode::Synchronize(int step_number, double time) {
    // Extract tire mesh vertex locations and velocites.
    std::vector<ChVector<>> vert_pos;
    std::vector<ChVector<>> vert_vel;
    std::vector<ChVector<int>> triangles;
    m_contact_load->OutputSimpleMesh(vert_pos, vert_vel, triangles);

    // Display information on lowest mesh node and lowest contact vertex.
    PrintLowestNode();
    PrintLowestVertex(vert_pos, vert_vel);

    // Send tire mesh vertex locations and velocities to the terrain node
    unsigned int num_vert = (unsigned int)vert_pos.size();
    unsigned int num_tri = (unsigned int)triangles.size();
    double* vert_data = new double[2 * 3 * num_vert];
    int* tri_data = new int[3 * num_tri];
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * iv + 0] = vert_pos[iv].x;
        vert_data[3 * iv + 1] = vert_pos[iv].y;
        vert_data[3 * iv + 2] = vert_pos[iv].z;
    }
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * num_vert + 3 * iv + 0] = vert_vel[iv].x;
        vert_data[3 * num_vert + 3 * iv + 1] = vert_vel[iv].y;
        vert_data[3 * num_vert + 3 * iv + 2] = vert_vel[iv].z;
    }
    for (unsigned int it = 0; it < num_tri; it++) {
        tri_data[3 * it + 0] = triangles[it].x;
        tri_data[3 * it + 1] = triangles[it].y;
        tri_data[3 * it + 2] = triangles[it].z;
    }
    MPI_Send(vert_data, 2 * 3 * num_vert, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);
    MPI_Send(tri_data, 3 * num_tri, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);

    delete[] vert_data;
    delete[] tri_data;

    // Receive terrain forces.
    // Note that we use MPI_Probe to figure out the number of indeces and forces received.
    MPI_Status status;
    int count;
    MPI_Probe(TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &count);
    int* index_data = new int[count];
    double* force_data = new double[3 * count];
    MPI_Recv(index_data, count, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(force_data, 3 * count, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    std::cout << "[Rig node    ] step number: " << step_number << "  vertices in contact: " << count << std::endl;

    // Repack data and apply forces to the mesh vertices
    std::vector<ChVector<>> vert_forces;
    std::vector<int> vert_indeces;
    for (int iv = 0; iv < count; iv++) {
        vert_forces.push_back(ChVector<>(force_data[3 * iv + 0], force_data[3 * iv + 1], force_data[3 * iv + 2]));
        vert_indeces.push_back(index_data[iv]);
    }
    m_contact_load->InputSimpleForces(vert_forces, vert_indeces);

    PrintContactData(vert_forces, vert_indeces);

    delete[] index_data;
    delete[] force_data;

    //// TODO: Perform any other required synchronization
}

// -----------------------------------------------------------------------------
// Advance simulation of the rig node by the specified duration
// -----------------------------------------------------------------------------
void RigNode::Advance(double step_size) {
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::OutputData() {
    if (!m_outf.is_open())
        return;

    std::string del("  ");
    const ChVector<>& rim_pos = m_rim->GetPos();

    m_outf << m_system->GetChTime() << del;
    m_outf << rim_pos.x << del << rim_pos.y << del << rim_pos.z << del;
    m_outf << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::PrintLowestNode() {
    // Unfortunately, we do not have access to the node container of a mesh,
    // so we cannot use some nice algorithm here...
    unsigned int num_nodes = m_tire->GetMesh()->GetNnodes();
    unsigned int index = 0;
    double zmin = 1e10;
    for (unsigned int i = 0; i < num_nodes; ++i) {
        // Ugly casting here. (Note also that we need dynamic downcasting, due to the virtual base)
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire->GetMesh()->GetNode(i));
        if (node->GetPos().z < zmin) {
            zmin = node->GetPos().z;
            index = i;
        }
    }

    ChVector<> vel = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire->GetMesh()->GetNode(index))->GetPos_dt();
    std::cout << "[Rig node    ] lowest node:    index = " << index << "  height = " << zmin << "  velocity = " << vel.x
              << "  " << vel.y << "  " << vel.z << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::PrintLowestVertex(const std::vector<ChVector<>>& vert_pos, const std::vector<ChVector<>>& vert_vel) {
    auto lowest = std::min_element(vert_pos.begin(), vert_pos.end(),
                                   [](const ChVector<>& a, const ChVector<>& b) { return a.z < b.z; });
    int index = lowest - vert_pos.begin();
    const ChVector<>& vel = vert_vel[index];
    std::cout << "[Rig node    ] lowest vertex:  index = " << index << "  height = " << (*lowest).z
              << "  velocity = " << vel.x << "  " << vel.y << "  " << vel.z << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indeces) {
    std::cout << "[Rig node    ] contact forces" << std::endl;
    for (int i = 0; i < indeces.size(); i++) {
        std::cout << "  id = " << indeces[i] << "  force = " << forces[i].x << "  " << forces[i].y << "  "
                  << forces[i].z << std::endl;
    }
}

// =============================================================================
// TERRAIN NODE CLASS
// =============================================================================

class TerrainNode {
  public:
    enum Type { RIGID, GRANULAR };

    TerrainNode(Type type, ChMaterialSurfaceBase::ContactMethod method, int num_threads);
    ~TerrainNode();

    void SetOutputFile(const std::string& name);

    void Settle();
    void Initialize();
    void Synchronize(int step_number, double time);
    void Advance(double step_size);

    void OutputData();

  private:
    struct ProxyNodeBody {
        ProxyNodeBody(std::shared_ptr<ChBody> body, int index) : m_body(body), m_index(index) {}
        std::shared_ptr<ChBody> m_body;
        int m_index;
    };

    Type m_type;  ///< terrain type (RIGID or GRANULAR)

    ChMaterialSurfaceBase::ContactMethod m_method;  ///< contact method (penalty or complementarity)
    ChSystemParallel* m_system;                     ///< containing system

    std::shared_ptr<ChMaterialSurfaceBase> m_material_tire;  ///< material properties for proxy bodies
    std::vector<ProxyNodeBody> m_proxies;  ///< list of proxy bodies and associated mesh vertex indices
    bool m_fixed_proxies;                  ///< flag indicating whether or not proxy bodies are fixed to ground
    double m_mass_p;                       ///< mass of a proxy body
    double m_radius_p;                     ///< radius of a spherical proxy body

    double m_init_height;  ///< initial terrain height (after optional settling)
    double m_radius_g;     ///< radius of one particle of granular material

    unsigned int m_num_vert;  ///< number of tire mesh vertices
    unsigned int m_num_tri;   ///< number of tire mesh triangles

    std::ofstream m_outf;  ///< output file stream

    // Private methods
    void PrintContactData();
    void PrintLowestVertex();
};

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - receive tire contact material properties and create the "tire" material
// - create the (parallel) Chrono system and set solver parameters
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
TerrainNode::TerrainNode(Type type, ChMaterialSurfaceBase::ContactMethod method, int num_threads)
    : m_type(type), m_method(method), m_init_height(0) {
    // ----------------
    // Model parameters
    // ----------------

    // Container dimensions
    double hdimX = 5.0;
    double hdimY = 0.25;
    double hdimZ = 0.5;
    double hthick = 0.25;

    // Granular material properties
    m_radius_g = 0.02;
    int Id_g = 10000;
    double rho_g = 2500;
    double vol_g = (4.0 / 3) * CH_C_PI * m_radius_g * m_radius_g * m_radius_g;
    double mass_g = rho_g * vol_g;
    ChVector<> inertia_g = 0.4 * mass_g * m_radius_g * m_radius_g * ChVector<>(1, 1, 1);
    unsigned int num_particles = 1;

    // Terrain contact properties
    float friction_terrain = 0.9f;
    float restitution_terrain = 0.0f;
    float Y_terrain = 2e6f;
    float nu_terrain = 0.3f;
    float kn_terrain = 9.0e4f;
    float gn_terrain = 1.0e1f;
    float kt_terrain = 0;
    float gt_terrain = 0;

    // Proxy bodies properties
    m_fixed_proxies = false;
    m_mass_p = 1;
    m_radius_p = 0.01;

    // ----------------------------------------
    // Receive tire contact material properties
    // ----------------------------------------

    // Set use_material_properties in the system configuration.
    // Create the "tire" contact material, but defer using it until the proxy bodies are created.
    float mat_props[8];
    MPI_Status status;
    MPI_Recv(mat_props, 8, MPI_FLOAT, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    switch (m_method) {
        case ChMaterialSurfaceBase::DEM: {
            // Properties for tire
            auto mat_tire = std::make_shared<ChMaterialSurfaceDEM>();
            mat_tire->SetFriction(mat_props[0]);
            mat_tire->SetRestitution(mat_props[1]);
            mat_tire->SetYoungModulus(mat_props[2]);
            mat_tire->SetPoissonRatio(mat_props[3]);
            mat_tire->SetKn(mat_props[4]);
            mat_tire->SetGn(mat_props[5]);
            mat_tire->SetKt(mat_props[6]);
            mat_tire->SetGt(mat_props[7]);

            m_material_tire = mat_tire;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            auto mat_tire = std::make_shared<ChMaterialSurface>();
            mat_tire->SetFriction(mat_props[0]);
            mat_tire->SetRestitution(mat_props[1]);

            m_material_tire = mat_tire;

            break;
        }
    }

    std::cout << "[Terrain node] friction = " << mat_props[0] << std::endl;

    // --------------------------
    // Create the parallel system
    // --------------------------

    // Create system and set method-specific solver settings
    switch (m_method) {
        case ChMaterialSurfaceBase::DEM: {
            ChSystemParallelDEM* sys = new ChSystemParallelDEM;
            sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hooke;
            sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
            sys->GetSettings()->solver.use_material_properties = use_mat_properties;
            m_system = sys;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            ChSystemParallelDVI* sys = new ChSystemParallelDVI;
            sys->GetSettings()->solver.solver_mode = SLIDING;
            sys->GetSettings()->solver.max_iteration_normal = 0;
            sys->GetSettings()->solver.max_iteration_sliding = 200;
            sys->GetSettings()->solver.max_iteration_spinning = 0;
            sys->GetSettings()->solver.alpha = 0;
            sys->GetSettings()->solver.contact_recovery_speed = -1;
            sys->GetSettings()->collision.collision_envelope = 0.1 * m_radius_g;
            sys->ChangeSolverType(APGD);
            m_system = sys;

            break;
        }
    }

    // Solver settings independent of method type
    m_system->Set_G_acc(ChVector<>(0, 0, gacc));
    m_system->GetSettings()->perform_thread_tuning = false;
    m_system->GetSettings()->solver.use_full_inertia_tensor = false;
    m_system->GetSettings()->solver.tolerance = 0.1;
    m_system->GetSettings()->solver.max_iteration_bilateral = 100;
    m_system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    m_system->GetSettings()->collision.bins_per_axis = I3(10, 10, 10);

    // Set number of threads
    m_system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    // ---------------------
    // Create terrain bodies
    // ---------------------

    // Create contact material for terrain
    std::shared_ptr<ChMaterialSurfaceBase> material_terrain;

    switch (m_method) {
        case ChMaterialSurfaceBase::DEM: {
            auto mat_ter = std::make_shared<ChMaterialSurfaceDEM>();
            mat_ter->SetFriction(friction_terrain);
            mat_ter->SetRestitution(restitution_terrain);
            mat_ter->SetYoungModulus(Y_terrain);
            mat_ter->SetPoissonRatio(nu_terrain);
            mat_ter->SetKn(kn_terrain);
            mat_ter->SetGn(gn_terrain);
            mat_ter->SetKt(kt_terrain);
            mat_ter->SetGt(gt_terrain);

            material_terrain = mat_ter;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            auto mat_ter = std::make_shared<ChMaterialSurface>();
            mat_ter->SetFriction(friction_terrain);
            mat_ter->SetRestitution(restitution_terrain);

            material_terrain = mat_ter;

            break;
        }
    }

    // Create container body
    auto container = std::shared_ptr<ChBody>(m_system->NewBody());
    m_system->AddBody(container);
    container->SetIdentifier(-1);
    container->SetMass(1);
    container->SetBodyFixed(true);
    container->SetCollide(true);
    container->SetMaterialSurface(material_terrain);

    container->GetCollisionModel()->ClearModel();
    // Bottom box
    utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
                          ChQuaternion<>(1, 0, 0, 0), true);
    // Front box
    utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Rear box
    utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Left box
    utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    // Right box
    utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
    container->GetCollisionModel()->BuildModel();

    // If using RIGID terrain, the contact will be between the container and proxy bodies.
    // Since collision between two bodies fixed to ground is ignored, if the proxy bodies
    // are fixed, we make the container a free body connected through a weld joint to ground.
    // If using GRANULAR terrain, this is not an issue as the proxy bodies do not interact
    // with the container, but rather with the granular material.
    if (m_type == RIGID && m_fixed_proxies) {
        container->SetBodyFixed(false);

        auto ground = std::shared_ptr<ChBody>(m_system->NewBody());
        ground->SetIdentifier(-2);
        ground->SetBodyFixed(true);
        ground->SetCollide(false);
        m_system->AddBody(ground);

        auto weld = std::make_shared<ChLinkLockLock>();
        weld->Initialize(ground, container, ChCoordsys<>(VNULL, QUNIT));
        m_system->AddLink(weld);
    }

    // Create particles
    if (m_type == GRANULAR) {
        // Create a particle generator and a mixture entirely made out of spheres
        utils::Generator gen(m_system);
        std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
        m1->setDefaultMaterial(material_terrain);
        m1->setDefaultDensity(rho_g);
        m1->setDefaultSize(m_radius_g);

        // Set starting value for body identifiers
        gen.setBodyIdentifier(Id_g);

        // Create particles in layers until reaching the desired number of particles
        double r = 1.01 * m_radius_g;
        ChVector<> hdims(hdimX - r, hdimY - r, 0);
        ChVector<> center(0, 0, 2 * r);

        while (gen.getTotalNumBodies() < num_particles) {
            gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
            center.z += 2 * r;
        }

        std::cout << "[Terrain node] Generated particles:  " << gen.getTotalNumBodies() << std::endl;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TerrainNode::~TerrainNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::SetOutputFile(const std::string& name) {
    m_outf.open(name, std::ios::out | std::ios::app);
    m_outf.precision(7);
    m_outf << std::scientific;
}

// -----------------------------------------------------------------------------
// Settling phase for the terrain node
// - if using granular material, allow it to settle
// - record height of terrain
// -----------------------------------------------------------------------------
void TerrainNode::Settle() {
    m_init_height = 0;

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Terrain Node", m_system);
    gl_window.SetCamera(ChVector<>(0, -1, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    // If rigid terrain, return now
    if (m_type == RIGID) {
        return;
    }

    // Simulate granular material
    double time_end = 0.5;
    double time_step = 1e-3;

    while (m_system->GetChTime() < time_end) {
#ifdef CHRONO_OPENGL
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else
            MPI_Abort(MPI_COMM_WORLD, 1);
#else
        m_system->DoStepDynamics(time_step);
#endif
    }

    // Find "height" of granular material
    for (size_t i = 0; i < m_system->Get_bodylist()->size(); ++i) {
        auto body = (*m_system->Get_bodylist())[i];
        if (body->GetIdentifier() > 0 && body->GetPos().z > m_init_height)
            m_init_height = body->GetPos().z;
    }
    m_init_height += m_radius_g;
}

// -----------------------------------------------------------------------------
// Initialization of the terrain node:
// - send terrain height
// - receive information on tire mesh topology (number vertices and triangles)
// - create the appropriate proxy bodies (state not set yet)
// -----------------------------------------------------------------------------
void TerrainNode::Initialize() {
    // ---------------------------
    // Send initial terrain height
    // ---------------------------

    // Note: take into account dimension of proxy bodies
    double init_height = m_init_height + m_radius_p;
    MPI_Send(&init_height, 1, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD);

    std::cout << "[Terrain node] Initial terrain height = " << init_height << std::endl;

    // ------------------------------------------
    // Receive tire contact surface specification
    // ------------------------------------------

    unsigned int surf_props[2];
    MPI_Status status;
    MPI_Recv(surf_props, 2, MPI_UNSIGNED, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);
    m_num_vert = surf_props[0];
    m_num_tri = surf_props[1];

    std::cout << "[Terrain node] Received vertices = " << surf_props[0] << " triangles = " << surf_props[1]
              << std::endl;

    // -------------------
    // Create proxy bodies
    // -------------------

    // Create bodies with spherical contact geometry as proxies for the tire
    // mesh vertices and add them to the Chrono system.
    // Assign to each body an identifier equal to the index of its corresponding mesh vertex.
    // Maintain lists of all bodies associated with the tire.
    ChVector<> inertia_p = 0.4 * m_mass_p * m_radius_p * m_radius_p * ChVector<>(1, 1, 1);
    for (unsigned int iv = 0; iv < m_num_vert; iv++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        m_system->AddBody(body);
        body->SetIdentifier(iv);
        body->SetMass(m_mass_p);
        body->SetInertiaXX(inertia_p);
        body->SetBodyFixed(false);
        body->SetCollide(true);
        body->SetMaterialSurface(m_material_tire);

        body->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(body.get(), m_radius_p, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true);
        body->GetCollisionModel()->BuildModel();

        m_proxies.push_back(ProxyNodeBody(body, iv));
    }
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive tire mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// -----------------------------------------------------------------------------
void TerrainNode::Synchronize(int step_number, double time) {
    // Receive tire mesh vertex locations and velocities.
    MPI_Status status;
    double* vert_data = new double[2 * 3 * m_num_vert];
    int* tri_data = new int[3 * m_num_tri];
    MPI_Recv(vert_data, 2 * 3 * m_num_vert, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(tri_data, 3 * m_num_tri, MPI_INT, RIG_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    // Set position, rotation, and velocity of proxy bodies.
    for (unsigned int iv = 0; iv < m_num_vert; iv++) {
        unsigned int offset = 3 * iv;
        m_proxies[iv].m_body->SetPos(ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]));
        offset += 3 * m_num_vert;
        m_proxies[iv].m_body->SetPos_dt(
            ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]));
        m_proxies[iv].m_body->SetRot(ChQuaternion<>(1, 0, 0, 0));
        m_proxies[iv].m_body->SetRot_dt(ChQuaternion<>(0, 0, 0, 0));
    }

    delete[] vert_data;
    delete[] tri_data;

    // Display information on lowest proxy.
    PrintLowestVertex();

    // Calculate cumulative contact forces for all bodies in system.
    m_system->CalculateContactForces();

    // Collect contact forces on proxy bodies (only if in contact).
    // Note that no forces are collected at the first step.
    std::vector<double> vert_forces;
    std::vector<int> vert_indeces;
    if (step_number > 0) {
        for (unsigned int iv = 0; iv < m_num_vert; iv++) {
            real3 force = m_system->GetBodyContactForce(m_proxies[iv].m_body);

            if (!IsZero(force)) {
                vert_forces.push_back(force.x);
                vert_forces.push_back(force.y);
                vert_forces.push_back(force.z);
                vert_indeces.push_back(m_proxies[iv].m_index);
            }
        }
    }

    // Send vertex indeces and forces.
    int num_vert = (int)vert_indeces.size();
    MPI_Send(vert_indeces.data(), num_vert, MPI_INT, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);
    MPI_Send(vert_forces.data(), 3 * num_vert, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);

    std::cout << "[Terrain node] step number: " << step_number << "  num contacts: " << m_system->GetNcontacts()
              << "  vertices in contact: " << num_vert << std::endl;
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void TerrainNode::Advance(double step_size) {
    m_system->DoStepDynamics(step_size);
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (gl_window.Active()) {
        gl_window.Render();
    } else {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
#endif

    PrintContactData();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::OutputData() {
    if (!m_outf.is_open())
        return;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::PrintContactData() {
    // Information on all contacts.
    // Note that proxy body identifiers match the index of the associated mesh vertex.
    auto bodies = m_system->Get_bodylist();
    auto dm = m_system->data_manager;
    auto& bids = dm->host_data.bids_rigid_rigid;
    auto& cpta = dm->host_data.cpta_rigid_rigid;
    auto& cptb = dm->host_data.cptb_rigid_rigid;
    auto& dpth = dm->host_data.dpth_rigid_rigid;
    auto& norm = dm->host_data.norm_rigid_rigid;
    std::set<int> vertices_in_contact;
    std::cout << "[Terrain node] contact information (" << dm->num_rigid_contacts << ")" << std::endl;
    for (uint ic = 0; ic < dm->num_rigid_contacts; ic++) {
        int idA = bids[ic].x;
        int idB = bids[ic].y;
        int indexA = (*bodies)[idA]->GetIdentifier();
        int indexB = (*bodies)[idB]->GetIdentifier();
        if (indexA > 0)
            vertices_in_contact.insert(indexA);
        if (indexB > 0)
            vertices_in_contact.insert(indexB);

        std::cout << "  id1 = " << indexA << "  id2 = " << indexB << "   dpth = " << dpth[ic]
                  << "  normal = " << norm[ic].x << "  " << norm[ic].y << "  " << norm[ic].z << std::endl;
    }

    // Cumulative contact forces on proxy bodies.
    m_system->CalculateContactForces();
    std::cout << "[Terrain node] vertex forces (" << vertices_in_contact.size() << ")" << std::endl;
    for (unsigned int iv = 0; iv < m_num_vert; iv++) {
        if (vertices_in_contact.find(iv) != vertices_in_contact.end()) {
            real3 force = m_system->GetBodyContactForce(m_proxies[iv].m_body);
            std::cout << "  id = " << m_proxies[iv].m_index << "  force = " << force.x << "  " << force.y << "  "
                      << force.z << std::endl;
        }
    }

    ////auto container = std::static_pointer_cast<ChContactContainerParallel>(m_system->GetContactContainer());
    ////auto contacts = container->GetContactList();

    ////for (auto it = contacts.begin(); it != contacts.end(); ++it) {
    ////    ChBody* bodyA = static_cast<ChBody*>((*it)->GetObjA());
    ////    ChBody* bodyB = static_cast<ChBody*>((*it)->GetObjA());

    ////    std::cout << " id1 = " << bodyA->GetIdentifier() << "  id2 = " << bodyB->GetIdentifier() << std::endl;
    ////}
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNode::PrintLowestVertex() {
    auto lowest = std::min_element(
        m_proxies.begin(), m_proxies.end(),
        [](const ProxyNodeBody& a, const ProxyNodeBody& b) { return a.m_body->GetPos().z < b.m_body->GetPos().z; });
    const ChVector<>& vel = (*lowest).m_body->GetPos_dt();
    double height = (*lowest).m_body->GetPos().z;
    std::cout << "[Terrain node] lowest vertex:  index = " << (*lowest).m_index << "  height = " << height
              << "  velocity = " << vel.x << "  " << vel.y << "  " << vel.z << std::endl;
}

// =============================================================================
// MAIN DRIVER
// =============================================================================

int main() {
    // Initialize MPI.
    int num_procs;
    int rank;
    MPI_Init(NULL, NULL);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

#ifdef _DEBUG
    if (rank == 0) {
        int foo;
        std::cout << "Enter something to continue..." << std::endl;
        std::cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);
#endif

    // Create the two systems and run settling phase for terrain.
    // Data exchange:
    //   rig => terrain (tire contact material properties)
    RigNode* my_rig = NULL;
    TerrainNode* my_terrain = NULL;

    switch (rank) {
        case RIG_NODE_RANK:
            my_rig = new RigNode(nthreads_rignode);
            my_rig->SetOutputFile("TestRigCosim_RigNode.txt");
            break;
        case TERRAIN_NODE_RANK:
            my_terrain = new TerrainNode(TerrainNode::RIGID, ChMaterialSurfaceBase::DEM, nthreads_terrainnode);
            ////my_terrain->SetOutputFile("TestRigCosim_TerrainNode.txt");
            my_terrain->Settle();
            break;
    }

    // Initialize systems.
    // Data exchange:
    //   terrain => rig (terrain height)
    //   rig => terrain (tire mesh topology information)
    switch (rank) {
        case RIG_NODE_RANK:
            my_rig->Initialize();
            break;
        case TERRAIN_NODE_RANK:
            my_terrain->Initialize();
            break;
    }

    // Perform co-simulation.
    // At synchronization, there is bi-directional data exchange:
    //     rig => terrain (position information)
    //     terrain => rig (force information)
    int num_steps = 125000;
    double step_size = 7e-5;

    for (int is = 0; is < num_steps; is++) {
        double time = is * step_size;

        MPI_Barrier(MPI_COMM_WORLD);

        switch (rank) {
            case RIG_NODE_RANK: {
                std::cout << " ---------------------------- " << std::endl;
                my_rig->Synchronize(is, time);
                std::cout << " --- " << std::endl;

                my_rig->Advance(step_size);
                my_rig->OutputData();

                break;
            }
            case TERRAIN_NODE_RANK: {
                my_terrain->Synchronize(is, time);
                my_terrain->Advance(step_size);
                my_terrain->OutputData();

                break;
            }
        }
    }

    // Cleanup.
    delete my_rig;
    delete my_terrain;

    MPI_Finalize();

    return 0;
}