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
// Definition of the RIG NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO:
////    mesh connectivity doesn't need to be communicated every time (modify Chrono?)

#include <omp.h>
#include <algorithm>
#include <set>
#include <vector>
#include "mpi.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_fea/ChElementShellANCF.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#include "RigNode.h"

using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

class ChFunction_SlipAngle : public chrono::ChFunction {
  public:
    ChFunction_SlipAngle(double max_angle) : m_max_angle(max_angle) {}

    virtual ChFunction_SlipAngle* Clone() const override { return new ChFunction_SlipAngle(m_max_angle); }

    virtual double Get_y(double t) const override {
        // Ramp for 1 second and stay at that value (scale)
        double delay = 0.2;
        double scale = -m_max_angle / 180 * CH_C_PI;
        if (t <= delay)
            return 0;
        double t1 = t - delay;
        if (t1 >= 1)
            return scale;
        return t1 * scale;
    }

  private:
    double m_max_angle;
};

// =============================================================================

// -----------------------------------------------------------------------------
// Construction of the rig node:
// - create the (sequential) Chrono system and set solver parameters
// -----------------------------------------------------------------------------
RigNode::RigNode(double init_vel, double slip, int num_threads)
    : BaseNode("RIG"), m_init_vel(init_vel), m_slip(slip), m_constructed(false) {
    cout << "[Rig node    ] init_vel = " << init_vel << " slip = " << slip << " num_threads = " << num_threads << endl;

    // ------------------------
    // Default model parameters
    // ------------------------

    m_chassis_mass = 1;
    m_set_toe_mass = 1;
    m_upright_mass = 450;
    m_rim_mass = 15;

    m_tire_pressure = true;

    // -----------------------------------
    // Default integrator and solver types
    // -----------------------------------
    m_int_type = ChSystem::INT_HHT;
    m_slv_type = ChSystem::SOLVER_CUSTOM;

    // ----------------------------------
    // Create the (sequential) DEM system
    // ----------------------------------

    m_system = new ChSystemDEM;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number threads
    m_system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigNode::~RigNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::SetIntegratorType(ChSystem::eCh_integrationType int_type, ChSystem::eCh_solverType slv_type) {
    m_int_type = int_type;
    m_slv_type = slv_type;
#ifndef CHRONO_MKL
    m_slv_type = Chsystem::SOLVER_SOR;
#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::SetBodyMasses(double chassis_mass, double set_toe_mass, double upright_mass, double rim_mass) {
    m_chassis_mass = chassis_mass;
    m_set_toe_mass = set_toe_mass;
    m_upright_mass = upright_mass;
    m_rim_mass = rim_mass;
}

void RigNode::SetTireJSONFile(const std::string& filename) {
    m_tire_json = filename;
}

void RigNode::EnableTirePressure(bool val) {
    m_tire_pressure = val;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Initialize.
// - create (but do not initialize) the rig mechanism bodies and joints
// - create (but do not initialize) the tire
// -----------------------------------------------------------------------------
void RigNode::Construct() {
    if (m_constructed)
        return;

    // -------------------------------
    // Change solver and integrator
    // -------------------------------

    if (m_slv_type == ChSystem::SOLVER_CUSTOM) {
#ifdef CHRONO_MKL
        ChSolverMKL<>* mkl_solver_stab = new ChSolverMKL<>;
        ChSolverMKL<>* mkl_solver_speed = new ChSolverMKL<>;
        m_system->ChangeSolverStab(mkl_solver_stab);
        m_system->ChangeSolverSpeed(mkl_solver_speed);
        mkl_solver_speed->SetSparsityPatternLock(true);
        mkl_solver_stab->SetSparsityPatternLock(true);
#endif
    } else {
        m_system->SetSolverType(m_slv_type);
        m_system->SetMaxItersSolverSpeed(100);
        m_system->SetMaxItersSolverStab(100);
        m_system->SetTol(1e-10);
        m_system->SetTolForce(1e-8);
    }

    switch (m_int_type) {
        case ChSystem::INT_HHT:
            m_system->SetIntegrationType(ChSystem::INT_HHT);
            m_integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
            m_integrator->SetAlpha(-0.2);
            m_integrator->SetMaxiters(50);
            m_integrator->SetAbsTolerances(5e-05, 1.8e00);
            m_integrator->SetMode(ChTimestepperHHT::POSITION);
            m_integrator->SetScaling(true);
            m_integrator->SetVerbose(true);
            m_integrator->SetMaxItersSuccess(5);

            break;
    }

    // -------------------------------
    // Create the rig mechanism bodies
    // -------------------------------

    ChVector<> chassis_inertia(0.1, 0.1, 0.1);
    ChVector<> set_toe_inertia(0.1, 0.1, 0.1);
    ChVector<> upright_inertia(1, 1, 1);
    ChVector<> rim_inertia(1, 1, 1);

    // Create ground body.
    m_ground = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    m_ground->SetBodyFixed(true);
    m_system->AddBody(m_ground);

    // Create the chassis body.
    m_chassis = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    m_chassis->SetMass(m_chassis_mass);
    m_chassis->SetInertiaXX(chassis_inertia);
    m_system->AddBody(m_chassis);

    // Create the set toe body.
    m_set_toe = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    m_set_toe->SetMass(m_set_toe_mass);
    m_set_toe->SetInertiaXX(set_toe_inertia);
    m_system->AddBody(m_set_toe);

    // Create the rim body.
    m_rim = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    m_rim->SetMass(m_rim_mass);
    m_rim->SetInertiaXX(rim_inertia);
    m_system->AddBody(m_rim);

    // Create the upright body.
    m_upright = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);
    m_upright->SetMass(m_upright_mass);
    m_upright->SetInertiaXX(upright_inertia);
    m_system->AddBody(m_upright);

    // -------------------------------
    // Create the rig mechanism joints
    // -------------------------------

    // Connect chassis to set_toe body through an actuated revolute joint.
    m_slip_motor = std::make_shared<ChLinkEngine>();
    m_slip_motor->SetName("engine_set_slip");
    m_slip_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    m_system->AddLink(m_slip_motor);

    // Prismatic constraint on the toe
    m_prism_vel = std::make_shared<ChLinkLockPrismatic>();
    m_prism_vel->SetName("Prismatic_chassis_ground");
    m_system->AddLink(m_prism_vel);

    // Impose velocity actuation on the prismatic joint
    m_lin_actuator = std::make_shared<ChLinkLinActuator>();
    m_lin_actuator->SetName("Prismatic_actuator");
    m_lin_actuator->Set_lin_offset(1);  // Set actuator distance offset
    m_system->AddLink(m_lin_actuator);

    // Prismatic constraint on the toe-axle: Connects chassis to axle
    m_prism_axl = std::make_shared<ChLinkLockPrismatic>();
    m_prism_axl->SetName("Prismatic_vertical");
    m_system->AddLink(m_prism_axl);

    // Connect rim to axle: Impose rotation on the rim
    m_rev_motor = std::make_shared<ChLinkEngine>();
    m_rev_motor->SetName("Motor_ang_vel");
    m_rev_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    m_system->AddLink(m_rev_motor);

    // ---------------
    // Create the tire
    // ---------------

    ConstructTire();

    // ---------------------------------
    // Write file with rig node settings
    // ---------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);

    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Tire specification" << endl;
    outf << "   JSON file: " << m_tire_json << endl;
    outf << "   Pressure enabled? " << (m_tire_pressure ? "YES" : "NO") << endl;
    outf << "Rig body masses" << endl;
    outf << "   chassis = " << m_chassis_mass << endl;
    outf << "   set_toe = " << m_set_toe_mass << endl;
    outf << "   upright = " << m_upright_mass << endl;
    outf << "   rim     = " << m_rim_mass << endl;

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------
// Construct an ANCF tire
// -----------------------------------------------------------------------------
void RigNodeDeformableTire::ConstructTire() {
    m_tire = std::make_shared<ANCFTire>(m_tire_json);
    m_tire->EnablePressure(m_tire_pressure);
    m_tire->EnableContact(true);
    m_tire->EnableRimConnection(true);
    m_tire->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);
}

// -----------------------------------------------------------------------------
// Construct a Rigid tire with mesh contact
// -----------------------------------------------------------------------------
void RigNodeRigidTire::ConstructTire() {
    m_tire = std::make_shared<RigidTire>(m_tire_json);
    assert(m_tire->UseContactMesh());
}

// -----------------------------------------------------------------------------
// Initialization of the rig node:
// - complete system construction
// - receive terrain height and container half-length
// - initialize the mechanism bodies
// - initialize the mechanism joints
// - call the virtual method InitializeTire which does the following:
//   - initialize the tire and extract contact surface
//   - send information on tire mesh topology (number verices and triangles)
//   - send information on tire contact material
// -----------------------------------------------------------------------------
void RigNode::Initialize() {
    Construct();

    // --------------------------------------
    // Initialize the rig bodies and the tire
    // --------------------------------------

    // Receive initial terrain dimensions: terrain height and container half-length
    double init_dim[2];
    MPI_Status status;
    MPI_Recv(init_dim, 2, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    cout << "[Rig node    ] Received initial terrain height = " << init_dim[0] << endl;
    cout << "[Rig node    ] Received container half-length = " << init_dim[1] << endl;

    // Slighlty perturb terrain height to ensure there is no initial contact
    double init_height = init_dim[0] + 1e-5;
    double half_length = init_dim[1];

    double tire_radius = GetTireRadius();
    ChVector<> origin(-half_length + 1.5 * tire_radius, 0, init_height + tire_radius);
    ChVector<> init_vel(m_init_vel * (1.0 - m_slip), 0, 0);

    // Initialize chassis body
    m_chassis->SetPos(origin);
    m_chassis->SetRot(QUNIT);
    m_chassis->SetPos_dt(init_vel);

    // Initialize the set_toe body
    m_set_toe->SetPos(origin);
    m_set_toe->SetRot(QUNIT);
    m_set_toe->SetPos_dt(init_vel);

    // Initialize rim body.
    m_rim->SetPos(origin);
    m_rim->SetRot(QUNIT);
    m_rim->SetPos_dt(init_vel);
    m_rim->SetWvel_loc(ChVector<>(0, m_init_vel / tire_radius, 0));

    // Initialize axle body
    m_upright->SetPos(origin);
    m_upright->SetRot(QUNIT);
    m_upright->SetPos_dt(init_vel);

    // -----------------------------------
    // Initialize the rig mechanism joints
    // -----------------------------------

    // Revolute engine on set_toe
    m_slip_motor->Set_rot_funct(std::make_shared<ChFunction_SlipAngle>(0));
    m_slip_motor->Initialize(m_set_toe, m_chassis, ChCoordsys<>(m_set_toe->GetPos(), QUNIT));

    // Prismatic constraint on the toe
    m_prism_vel->Initialize(m_ground, m_chassis, ChCoordsys<>(m_chassis->GetPos(), Q_from_AngY(CH_C_PI_2)));

    // Impose velocity actuation on the prismatic joint
    m_lin_actuator->Set_dist_funct(std::make_shared<ChFunction_Ramp>(0.0, m_init_vel * (1.0 - m_slip)));
    m_lin_actuator->Initialize(m_ground, m_chassis, false, ChCoordsys<>(m_chassis->GetPos(), QUNIT),
        ChCoordsys<>(m_chassis->GetPos() + ChVector<>(1, 0, 0), QUNIT));

    // Prismatic constraint on the toe-upright: Connects chassis to axle
    m_prism_axl->Initialize(m_set_toe, m_upright, ChCoordsys<>(m_set_toe->GetPos(), QUNIT));

    // Connect rim to upright: Impose rotation on the rim
    m_rev_motor->Set_rot_funct(std::make_shared<ChFunction_Ramp>(0, -m_init_vel / tire_radius));
    m_rev_motor->Initialize(m_rim, m_upright, ChCoordsys<>(m_rim->GetPos(), Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));

    // -----------------------------------
    // Initialize the tire
    // -----------------------------------

    // Let the derived class initialize the tire and send information to the terrain node
    InitializeTire();

    // Mark completion of system construction
    m_system->SetupInitial();
}

void RigNodeDeformableTire::InitializeTire() {
    // Initialize the ANCF tire
    m_tire->Initialize(m_rim, LEFT);

    // Create a mesh load for contact forces and add it to the tire's load container.
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire->GetContactSurface());
    m_contact_load = std::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire->GetLoadContainer()->Add(m_contact_load);

    // Preprocess the tire mesh and store neighbor element information for each vertex
    // and vertex indices for each element. This data is used in output.
    auto mesh = m_tire->GetMesh();
    m_adjElements.resize(mesh->GetNnodes());
    m_adjVertices.resize(mesh->GetNelements());

    int nodeOrder[] = { 0, 1, 2, 3 };
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ie++) {
        auto element = mesh->GetElement(ie);
        for (int in = 0; in < 4; in++) {
            auto node = element->GetNodeN(nodeOrder[in]);
            auto node_itr = std::find(mesh->GetNodes().begin(), mesh->GetNodes().end(), node);
            auto iv = std::distance(mesh->GetNodes().begin(), node_itr);
            m_adjElements[iv].push_back(ie);
            m_adjVertices[ie].push_back(iv);
        }
    }

    // Send tire contact surface specification
    unsigned int surf_props[2];
    surf_props[0] = contact_surface->GetNumVertices();
    surf_props[1] = contact_surface->GetNumTriangles();
    MPI_Send(surf_props, 2, MPI_UNSIGNED, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << "[Rig node    ] vertices = " << surf_props[0] << "  triangles = " << surf_props[1] << endl;

    // Send tire contact material properties
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

    cout << "[Rig node    ] friction = " << mat_props[0] << endl;
}

void RigNodeRigidTire::InitializeTire() {
    // Initialize the rigid tire
    m_tire->Initialize(m_rim, LEFT);

    // Preprocess the tire mesh and store neighbor element information for each vertex.
    // Calculate mesh triangle areas.
    m_adjElements.resize(m_tire->GetNumVertices());
    std::vector<double> triArea(m_tire->GetNumTriangles());
    const std::vector<ChVector<>>& vertices = m_tire->GetMeshVertices();
    const std::vector<ChVector<int>>& triangles = m_tire->GetMeshConnectivity();
    for (unsigned int ie = 0; ie < m_tire->GetNumTriangles(); ie++) {
        int iv1 = triangles[ie].x;
        int iv2 = triangles[ie].y;
        int iv3 = triangles[ie].z;
        ChVector<> v1 = vertices[iv1];
        ChVector<> v2 = vertices[iv2];
        ChVector<> v3 = vertices[iv3];
        triArea[ie] = 0.5 * Vcross(v2-v1, v3-v1).Length();
        m_adjElements[iv1].push_back(ie);
        m_adjElements[iv2].push_back(ie);
        m_adjElements[iv3].push_back(ie);
    }

    // Preprocess the tire mesh and store representative area for each vertex.
    m_vertexArea.resize(m_tire->GetNumVertices());
    for (unsigned int in = 0; in < m_tire->GetNumVertices(); in++) {
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            area += triArea[m_adjElements[in][ie]];
        }
        m_vertexArea[in] = area / m_adjElements[in].size();
    }

    // Send tire contact surface specification
    unsigned int surf_props[2];
    surf_props[0] = m_tire->GetNumVertices();
    surf_props[1] = m_tire->GetNumTriangles();
    MPI_Send(surf_props, 2, MPI_UNSIGNED, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << "[Rig node    ] vertices = " << surf_props[0] << "  triangles = " << surf_props[1] << endl;

    // Send tire contact material properties
    auto contact_mat = m_rim->GetMaterialSurfaceDEM();
    float mat_props[8] = {contact_mat->GetSfriction(),    contact_mat->GetRestitution(), contact_mat->GetYoungModulus(),
                          contact_mat->GetPoissonRatio(), contact_mat->GetKn(),          contact_mat->GetGn(),
                          contact_mat->GetKt(),           contact_mat->GetGt()};

    MPI_Send(mat_props, 8, MPI_FLOAT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << "[Rig node    ] friction = " << mat_props[0] << endl;
}

// -----------------------------------------------------------------------------
// Synchronization of the rig node:
// - extract and send tire mesh vertex states
// - receive and apply vertex contact forces
// -----------------------------------------------------------------------------
void RigNodeDeformableTire::Synchronize(int step_number, double time) {
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

    // Receive terrain forces.
    // Note that we use MPI_Probe to figure out the number of indices and forces received.
    MPI_Status status;
    int count;
    MPI_Probe(TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &count);
    int* index_data = new int[count];
    double* force_data = new double[3 * count];
    MPI_Recv(index_data, count, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(force_data, 3 * count, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    cout << "[Rig node    ] step number: " << step_number << "  vertices in contact: " << count << endl;

    // Repack data and apply forces to the mesh vertices
    m_vert_indices.resize(count);
    m_vert_pos.resize(count);
    m_vert_forces.resize(count);
    for (int iv = 0; iv < count; iv++) {
        int index = index_data[iv];
        m_vert_indices[iv] = index;
        m_vert_pos[iv] = vert_pos[index];
        m_vert_forces[iv] = ChVector<>(force_data[3 * iv + 0], force_data[3 * iv + 1], force_data[3 * iv + 2]);
    }
    m_contact_load->InputSimpleForces(m_vert_forces, m_vert_indices);

    PrintContactData(m_vert_forces, m_vert_indices);

    delete[] vert_data;
    delete[] tri_data;

    delete[] index_data;
    delete[] force_data;
}

void RigNodeRigidTire::Synchronize(int step_number, double time) {
    // Get contact mesh connectivity.
    std::vector<ChVector<int>> triangles = m_tire->GetMeshConnectivity();

    // Get current contact mesh vertex positions and velocities.
    std::vector<ChVector<>> vert_pos;
    std::vector<ChVector<>> vert_vel;
    m_tire->GetMeshVertexStates(vert_pos, vert_vel);

    // Display information on lowest contact vertex.
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

    // Receive terrain forces.
    // Note that we use MPI_Probe to figure out the number of indices and forces received.
    MPI_Status status;
    int count;
    MPI_Probe(TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &count);
    int* index_data = new int[count];
    double* force_data = new double[3 * count];
    MPI_Recv(index_data, count, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(force_data, 3 * count, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    cout << "[Rig node    ] step number: " << step_number << "  vertices in contact: " << count << endl;

    // Repack data and apply forces to the rim body
    m_vert_indices.resize(count);
    m_vert_pos.resize(count);
    m_vert_forces.resize(count);
    for (int iv = 0; iv < count; iv++) {
        int index = index_data[iv];
        m_vert_indices[iv] = index;
        m_vert_pos[iv] = vert_pos[index];
        m_vert_forces[iv] = ChVector<>(force_data[3 * iv + 0], force_data[3 * iv + 1], force_data[3 * iv + 2]);
    }

    m_rim->Empty_forces_accumulators();
    for (size_t i = 0; i < count; ++i) {
        m_rim->Accumulate_force(m_vert_forces[i], m_vert_pos[i], false);
    }

    PrintContactData(m_vert_forces, m_vert_indices);

    delete[] vert_data;
    delete[] tri_data;

    delete[] index_data;
    delete[] force_data;
}

// -----------------------------------------------------------------------------
// Advance simulation of the rig node by the specified duration
// -----------------------------------------------------------------------------
void RigNodeDeformableTire::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_tire->GetMesh()->ResetCounters();
        m_tire->GetMesh()->ResetTimers();
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

void RigNodeRigidTire::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& rim_pos = m_rim->GetPos();
        const ChVector<>& chassis_pos = m_chassis->GetPos();
        const ChVector<>& rim_vel = m_rim->GetPos_dt();
        const ChVector<>& rim_angvel = m_rim->GetWvel_loc();

        const ChVector<>& rfrc_prsm = m_prism_vel->Get_react_force();
        const ChVector<>& rtrq_prsm = m_prism_vel->Get_react_torque();
        const ChVector<>& rfrc_act = m_lin_actuator->Get_react_force();  // drawbar pull
        const ChVector<>& rtrq_act = m_lin_actuator->Get_react_torque();
        const ChVector<>& rfrc_motor = m_rev_motor->Get_react_force();
        const ChVector<>& rtrq_motor = m_rev_motor->Get_react_torque();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << rim_pos.x << del << rim_pos.y << del << rim_pos.z << del;
        m_outf << rim_vel.x << del << rim_vel.y << del << rim_vel.z << del;
        m_outf << rim_angvel.x << del << rim_angvel.y << del << rim_angvel.z << del;
        m_outf << chassis_pos.x << del << chassis_pos.y << del << chassis_pos.z << del;
        // Joint reactions
        m_outf << rfrc_prsm.x << del << rfrc_prsm.y << del << rfrc_prsm.z << del;
        m_outf << rtrq_prsm.x << del << rtrq_prsm.y << del << rtrq_prsm.z << del;
        m_outf << rfrc_act.x << del << rfrc_act.y << del << rfrc_act.z << del;
        m_outf << rtrq_act.x << del << rtrq_act.y << del << rtrq_act.z << del;
        m_outf << rfrc_motor.x << del << rfrc_motor.y << del << rfrc_motor.z << del;
        m_outf << rtrq_motor.x << del << rtrq_motor.y << del << rtrq_motor.z << del;
        // Solver statistics (for last integration step)
        m_outf << m_system->GetTimerStep() << del << m_system->GetTimerSetup() << del << m_system->GetTimerSolver()
               << del << m_system->GetTimerUpdate() << del;
        if (m_int_type == ChSystem::INT_HHT) {
            m_outf << m_integrator->GetNumIterations() << del << m_integrator->GetNumSetupCalls() << del
                   << m_integrator->GetNumSolveCalls() << del;
        }
        // Tire statistics
        OutputTireData(del);
        m_outf << endl;
    }

    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_node_out_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    WriteBodyInformation(csv);             // rig body states
    WriteTireInformation(csv);             // tire-related data
    csv.write_to_file(filename);

    cout << "[Rig node    ] write output file ==> " << filename << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNodeDeformableTire::OutputTireData(const std::string& del) {
    auto mesh = m_tire->GetMesh();

    m_outf << mesh->GetTimeInternalForces() << del << mesh->GetTimeJacobianLoad();
    m_outf << mesh->GetNumCallsInternalForces() << del << mesh->GetNumCallsJacobianLoad();
}

void RigNodeRigidTire::OutputTireData(const std::string& del) {
    //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::WriteBodyInformation(utils::CSV_writer& csv) {
    // Write number of bodies
    csv << "4" << endl;

    // Write body state information
    csv << m_chassis->GetIdentifier() << m_chassis->GetPos() << m_chassis->GetRot() << m_chassis->GetPos_dt()
        << m_chassis->GetRot_dt() << endl;
    csv << m_set_toe->GetIdentifier() << m_set_toe->GetPos() << m_set_toe->GetRot() << m_set_toe->GetPos_dt()
        << m_set_toe->GetRot_dt() << endl;
    csv << m_rim->GetIdentifier() << m_rim->GetPos() << m_rim->GetRot() << m_rim->GetPos_dt() << m_rim->GetRot_dt()
        << endl;
    csv << m_upright->GetIdentifier() << m_upright->GetPos() << m_upright->GetRot() << m_upright->GetPos_dt()
        << m_upright->GetRot_dt() << endl;
}

void RigNodeDeformableTire::WriteTireInformation(utils::CSV_writer& csv) {
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);
    WriteTireContactInformation(csv);
}

void RigNodeRigidTire::WriteTireInformation(utils::CSV_writer& csv) {
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);
    WriteTireContactInformation(csv);
}

void RigNodeDeformableTire::WriteTireStateInformation(utils::CSV_writer& csv) {
    // Extract vertex states from mesh
    auto mesh = m_tire->GetMesh();
    ChState x(mesh->GetDOF(), NULL);
    ChStateDelta v(mesh->GetDOF_w(), NULL);
    unsigned int offset_x = 0;
    unsigned int offset_v = 0;
    double t;
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        auto node = mesh->GetNode(in);
        node->NodeIntStateGather(offset_x, x, offset_v, v, t);
        offset_x += node->Get_ndof_x();
        offset_v += node->Get_ndof_w();
    }

    // Write number of vertices, number of DOFs
    csv << mesh->GetNnodes() << mesh->GetDOF() << mesh->GetDOF_w() << endl;

    // Write mesh vertex positions and velocities
    for (int ix = 0; ix < x.GetLength(); ix++)
        csv << x(ix) << endl;
    for (int iv = 0; iv < v.GetLength(); iv++)
        csv << v(iv) << endl;
}

void RigNodeDeformableTire::WriteTireMeshInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto mesh = m_tire->GetMesh();

    // Print tire mesh connectivity
    csv << "\n Connectivity " << mesh->GetNelements() << 5 * mesh->GetNelements() << endl;

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ie++) {
        for (unsigned int in = 0; in < m_adjVertices[ie].size(); in++) {
            csv << m_adjVertices[ie][in];
        }
        csv << endl;
    }

    // Print strain information: eps_xx, eps_yy, eps_xy averaged over surrounding elements
    csv << "\n Vectors of Strains \n";
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        double areaX = 0, areaY = 0, areaZ = 0;
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            auto element = std::static_pointer_cast<fea::ChElementShellANCF>(mesh->GetElement(m_adjElements[in][ie]));
            ChVector<> StrainVector = element->EvaluateSectionStrains();
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            area += dx * dy / 4;
            areaX += StrainVector.x * dx * dy / 4;
            areaY += StrainVector.y * dx * dy / 4;
            areaZ += StrainVector.z * dx * dy / 4;
        }
        csv << areaX / area << " " << areaY / area << " " << areaZ / area << endl;
    }
}

void RigNodeDeformableTire::WriteTireContactInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto mesh = m_tire->GetMesh();

    // Write the number of vertices in contact
    csv << m_vert_indices.size() << endl;

    // For each vertex in contact, calculate a representative area by averaging
    // the areas of its adjacent elements.
    for (unsigned int iv = 0; iv < m_vert_indices.size(); iv++) {
        int in = m_vert_indices[iv];
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(mesh->GetNode(in));
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            auto element = std::static_pointer_cast<fea::ChElementShellANCF>(mesh->GetElement(m_adjElements[in][ie]));
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            area += dx * dy / 4;
        }
        area /= m_adjElements[in].size();
        // Output vertex index, position, contact force, normal, and area.
        csv << in << m_vert_pos[iv] << m_vert_forces[iv] << node->GetD().GetNormalized() << area << endl;
    }
}

void RigNodeRigidTire::WriteTireStateInformation(utils::CSV_writer& csv) {
    // Write number of vertices
    unsigned int num_vertices = m_tire->GetNumVertices();
    csv << num_vertices << endl;

    // Write mesh vertex positions and velocities
    std::vector<ChVector<>> pos;
    std::vector<ChVector<>> vel;
    m_tire->GetMeshVertexStates(pos, vel);
    for (unsigned int in = 0; in < num_vertices; in++)
        csv << pos[in] << endl;
    for (unsigned int in = 0; in < num_vertices; in++)
        csv << vel[in] << endl;
}

void RigNodeRigidTire::WriteTireMeshInformation(utils::CSV_writer& csv) {
    // Print tire mesh connectivity
    csv << "\n Connectivity " << m_tire->GetNumTriangles() << endl;

    const std::vector<ChVector<int>>& triangles = m_tire->GetMeshConnectivity();
    for (unsigned int ie = 0; ie < m_tire->GetNumTriangles(); ie++) {
        csv << triangles[ie] << endl;
    }
}

void RigNodeRigidTire::WriteTireContactInformation(utils::CSV_writer& csv) {
    // Write the number of vertices in contact
    csv << m_vert_indices.size() << endl;

    // For each vertex in contact, output vertex index, contact force, normal, and area.
    const std::vector<ChVector<>>& normals = m_tire->GetMeshNormals();
    for (unsigned int iv = 0; iv < m_vert_indices.size(); iv++) {
        int in = m_vert_indices[iv];
        ChVector<> nrm = m_rim->TransformDirectionLocalToParent(normals[in]);
        csv << in << m_vert_pos[iv] << m_vert_forces[iv] << nrm << m_vertexArea[in] << endl;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::PrintLowestVertex(const std::vector<ChVector<>>& vert_pos, const std::vector<ChVector<>>& vert_vel) {
    auto lowest = std::min_element(vert_pos.begin(), vert_pos.end(),
                                   [](const ChVector<>& a, const ChVector<>& b) { return a.z < b.z; });
    int index = lowest - vert_pos.begin();
    const ChVector<>& vel = vert_vel[index];
    cout << "[Rig node    ] lowest vertex:  index = " << index << "  height = " << (*lowest).z
         << "  velocity = " << vel.x << "  " << vel.y << "  " << vel.z << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indices) {
    cout << "[Rig node    ] contact forces" << endl;
    for (int i = 0; i < indices.size(); i++) {
        cout << "  id = " << indices[i] << "  force = " << forces[i].x << "  " << forces[i].y << "  " << forces[i].z
             << endl;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNodeDeformableTire::PrintLowestNode() {
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
    cout << "[Rig node    ] lowest node:    index = " << index << "  height = " << zmin << "  velocity = " << vel.x
        << "  " << vel.y << "  " << vel.z << endl;
}

