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
////    complete OutputData() function

#include <omp.h>
#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <vector>
#include "mpi.h"

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_fea/ChElementShellANCF.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#include "Settings.h"
#include "RigNode.h"

using namespace chrono;
using namespace chrono::vehicle;


// =============================================================================

double ChFunction_SlipAngle::Get_y(double t) const {
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

// =============================================================================

const std::string RigNode::m_checkpoint_filename = rig_dir + "/checkpoint.dat";

// -----------------------------------------------------------------------------
// Construction of the rig node:
// - create the (sequential) Chrono system and set solver parameters
// - create (but do not initialize) the rig mechanism bodies and joints
// - create (but do not initialize) the tire
// - send information on tire contact material
// -----------------------------------------------------------------------------
RigNode::RigNode(int num_threads) : m_cumm_sim_time(0) {
    // ----------------
    // Model parameters
    // ----------------

    m_step_size = 1e-4;

    double chassis_mass = 0.1;
    double set_toe_mass = 0.1;
	double axle_mass = 0.1;
    double rim_mass = 100;

    ChVector<> chassis_inertia(1, 1, 1);
    ChVector<> set_toe_inertia(0.1, 0.1, 0.1);  
	ChVector<> axle_inertia(0.1, 0.1, 0.1);
    ChVector<> rim_inertia(1, 1, 1);  //// (1e-2, 1e-2, 1e-2);

    m_init_vel = 0; //// 20;
	m_slip = 0; // Enforced longitudinal slip

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
    m_ground->SetBodyFixed(true);
    m_system->AddBody(m_ground);

    // Create the chassis body.
    m_chassis = std::make_shared<ChBody>();
    m_chassis->SetMass(chassis_mass);
    m_chassis->SetInertiaXX(chassis_inertia);
    m_system->AddBody(m_chassis);

    // Create the set toe body.
    m_set_toe = std::make_shared<ChBody>();
    m_set_toe->SetMass(set_toe_mass);
    m_set_toe->SetInertiaXX(set_toe_inertia);
    m_system->AddBody(m_set_toe);

    // Create the rim body.
    m_rim = std::make_shared<ChBody>();
    m_rim->SetMass(rim_mass);
    m_rim->SetInertiaXX(rim_inertia);
    m_system->AddBody(m_rim);

	// Create the axle body.
	m_axle = std::make_shared<ChBody>();
	m_axle->SetMass(axle_mass);
	m_axle->SetInertiaXX(axle_inertia);
    m_system->AddBody(m_axle);

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
    m_prism_vel->SetName("Prismatic_settoe_ground");
    m_system->AddLink(m_prism_vel);

    // Impose velocity actuation on the prismatic joint
    m_lin_actuator = std::make_shared<ChLinkLinActuator>();
    m_lin_actuator->SetName("Prismatic_actuator");
    m_lin_actuator->Set_lin_offset(1); // Set actuator distance offset
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

    std::string ancftire_file("hmmwv/tire/HMMWV_ANCFTire.json");

    m_tire = std::make_shared<ANCFTire>(vehicle::GetDataFile(ancftire_file));
    m_tire->EnablePressure(false);
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
    m_outf.open(name, std::ios::out);
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
    // --------------------------------------
    // Initialize the rig bodies and the tire
    // --------------------------------------

    // Receive initial terrain dimensions: terrain height and container half-length
    double init_dim[2];
    MPI_Status status;
    MPI_Recv(init_dim, 2, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    std::cout << "[Rig node    ] Received initial terrain height = " << init_dim[0] << std::endl;
    std::cout << "[Rig node    ] Received container half-length = " << init_dim[1] << std::endl;

    // Slighlty perturb terrain height to ensure there is no initial contact
    init_dim[0] += 1e-5;

    // Set states, either at initial configuration or from checkpoint (depending on phase)
    switch (phase) {
        case SETTLING: {
			InitBodies(init_dim[0], init_dim[1]);
            break;
        }
        case TESTING: {
            InitBodies();
            break;
        }
    }

    // -----------------------------------
    // Initialize the rig mechanism joints
    // -----------------------------------

    // Revolute engine on set_toe
    auto slip_function = std::make_shared<ChFunction_SlipAngle>(0);
    m_slip_motor->Set_rot_funct(slip_function);
    m_slip_motor->Initialize(m_set_toe, m_chassis, ChCoordsys<>(m_set_toe->GetPos(), QUNIT));

    // Prismatic constraint on the toe
    m_prism_vel->Initialize(m_ground, m_set_toe, ChCoordsys<>(m_set_toe->GetPos(), Q_from_AngY(CH_C_PI_2)));

    // Impose velocity actuation on the prismatic joint
    auto actuator_fun = std::make_shared<ChFunction_Ramp>(0.0, m_init_vel * (1.0 - m_slip));
    m_lin_actuator->Set_dist_funct(actuator_fun);
    m_lin_actuator->Initialize(m_ground, m_set_toe, false, ChCoordsys<>(m_set_toe->GetPos(), QUNIT),
                               ChCoordsys<>(m_set_toe->GetPos() + ChVector<>(1, 0, 0), QUNIT));

    // Prismatic constraint on the toe-axle: Connects chassis to axle
    m_prism_axl->Initialize(m_set_toe, m_axle, ChCoordsys<>(m_set_toe->GetPos(), QUNIT));

    // Connect rim to axle: Impose rotation on the rim
    m_rev_motor->Initialize(m_rim, m_axle, ChCoordsys<>(m_rim->GetPos(), Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
    m_rev_motor->Set_rot_funct(std::make_shared<ChFunction_Ramp>(0, -m_init_vel / m_tire->GetRadius()));
    m_system->AddLink(m_rev_motor);

    // ----------------------
    // Create contact surface
    // ----------------------

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
// Initialization of the rig node bodies (initial configuration)
// - initialize the state of the mechanism bodies
// - initialize the tire
// -----------------------------------------------------------------------------
void RigNode::InitBodies(double init_height, double long_half_length) {
    double tire_radius = m_tire->GetRadius();
    ChVector<> origin(-long_half_length + 1.5 * tire_radius, 0, init_height + tire_radius);
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
    m_axle->SetPos(origin);
    m_axle->SetRot(QUNIT);
    m_axle->SetPos_dt(init_vel);

    // Initialize tire
    m_tire->Initialize(m_rim, LEFT);
}

// -----------------------------------------------------------------------------
// Initialization of the rig node bodies from a checkpoint data file:
// - initialize the mechanism bodies with states from checkpoint file
// - initialize the tire and overwrite mesh state from checkpoint file
// -----------------------------------------------------------------------------
void RigNode::InitBodies() {
    // Open input file stream
    std::ifstream ifile(m_checkpoint_filename);
    std::string line;

    // Initialize the rig mechanism bodies
    int identifier;
    ChVector<> pos;
    ChQuaternion<> rot;
    ChVector<> pos_dt;
    ChQuaternion<> rot_dt;

    // Read and discard line with number of bodies
    std::getline(ifile, line);

    // Initialize chassis body.
    {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> identifier >> pos.x >> pos.y >> pos.z >> rot.e0 >> rot.e1 >> rot.e2 >> rot.e3 >> pos_dt.x >> pos_dt.y >>
            pos_dt.z >> rot_dt.e0 >> rot_dt.e1 >> rot_dt.e2 >> rot_dt.e3;

        m_chassis->SetPos(ChVector<>(pos.x, pos.y, pos.z));
        m_chassis->SetRot(ChQuaternion<>(rot.e0, rot.e1, rot.e2, rot.e3));
        m_chassis->SetPos_dt(ChVector<>(pos_dt.x, pos_dt.y, pos_dt.z));
        m_chassis->SetRot_dt(ChQuaternion<>(rot_dt.e0, rot_dt.e1, rot_dt.e2, rot_dt.e3));
    }

    // Initialize set_toe body.
    {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> identifier >> pos.x >> pos.y >> pos.z >> rot.e0 >> rot.e1 >> rot.e2 >> rot.e3 >> pos_dt.x >> pos_dt.y >>
            pos_dt.z >> rot_dt.e0 >> rot_dt.e1 >> rot_dt.e2 >> rot_dt.e3;

        m_set_toe->SetPos(ChVector<>(pos.x, pos.y, pos.z));
        m_set_toe->SetRot(ChQuaternion<>(rot.e0, rot.e1, rot.e2, rot.e3));
        m_set_toe->SetPos_dt(ChVector<>(pos_dt.x, pos_dt.y, pos_dt.z));
        m_set_toe->SetRot_dt(ChQuaternion<>(rot_dt.e0, rot_dt.e1, rot_dt.e2, rot_dt.e3));
    }

    // Initialize rim body.
    {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> identifier >> pos.x >> pos.y >> pos.z >> rot.e0 >> rot.e1 >> rot.e2 >> rot.e3 >> pos_dt.x >> pos_dt.y >>
            pos_dt.z >> rot_dt.e0 >> rot_dt.e1 >> rot_dt.e2 >> rot_dt.e3;

        m_rim->SetPos(ChVector<>(pos.x, pos.y, pos.z));
        m_rim->SetRot(ChQuaternion<>(rot.e0, rot.e1, rot.e2, rot.e3));
        m_rim->SetPos_dt(ChVector<>(pos_dt.x, pos_dt.y, pos_dt.z));
        m_rim->SetRot_dt(ChQuaternion<>(rot_dt.e0, rot_dt.e1, rot_dt.e2, rot_dt.e3));
    }

	// Initialize axle body.
	{
		std::getline(ifile, line);
		std::istringstream iss(line);
		iss >> identifier >> pos.x >> pos.y >> pos.z >> rot.e0 >> rot.e1 >> rot.e2 >> rot.e3 >> pos_dt.x >> pos_dt.y >>
			pos_dt.z >> rot_dt.e0 >> rot_dt.e1 >> rot_dt.e2 >> rot_dt.e3;

		m_axle->SetPos(ChVector<>(pos.x, pos.y, pos.z));
		m_axle->SetRot(ChQuaternion<>(rot.e0, rot.e1, rot.e2, rot.e3));
		m_axle->SetPos_dt(ChVector<>(pos_dt.x, pos_dt.y, pos_dt.z));
		m_axle->SetRot_dt(ChQuaternion<>(rot_dt.e0, rot_dt.e1, rot_dt.e2, rot_dt.e3));
	}

    // Read and discard line with number of vertices and DOFs
    std::getline(ifile, line);

    // Initialize the tire, then overwrite the state of the underlying mesh.
    m_tire->Initialize(m_rim, LEFT);

    auto mesh = m_tire->GetMesh();
    ChState x(mesh->GetDOF(), NULL);
    ChStateDelta v(mesh->GetDOF_w(), NULL);

    for (int ix = 0; ix < x.GetLength(); ix++) {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> x(ix);
    }
    for (int iv = 0; iv < v.GetLength(); iv++) {
        std::getline(ifile, line);
        std::istringstream iss(line);
        iss >> v(iv);
    }

    unsigned int offset_x = 0;
    unsigned int offset_v = 0;
    double t = 0;
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        auto node = mesh->GetNode(in);
        node->NodeIntStateScatter(offset_x, x, offset_v, v, t);
        offset_x += node->Get_ndof_x();
        offset_v += node->Get_ndof_w();
    }
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
    // Note that we use MPI_Probe to figure out the number of indices and forces received.
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
    std::vector<int> vert_indices;
    for (int iv = 0; iv < count; iv++) {
        vert_forces.push_back(ChVector<>(force_data[3 * iv + 0], force_data[3 * iv + 1], force_data[3 * iv + 2]));
        vert_indices.push_back(index_data[iv]);
    }
    m_contact_load->InputSimpleForces(vert_forces, vert_indices);

    PrintContactData(vert_forces, vert_indices);

    delete[] index_data;
    delete[] force_data;
}

// -----------------------------------------------------------------------------
// Advance simulation of the rig node by the specified duration
// -----------------------------------------------------------------------------
void RigNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cumm_sim_time += m_timer();
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
        const ChVector<>& rfrc_prsm = m_prism_vel->Get_react_force();
        const ChVector<>& rtrq_prsm = m_prism_vel->Get_react_torque();
        const ChVector<>& rfrc_act = m_lin_actuator->Get_react_force();  // drawbar pull
        const ChVector<>& rtrq_act = m_lin_actuator->Get_react_torque();
        const ChVector<>& rfrc_motor = m_rev_motor->Get_react_force();
        const ChVector<>& rtrq_motor = m_rev_motor->Get_react_torque();

        m_outf << m_system->GetChTime() << del;
        m_outf << rim_pos.x << del << rim_pos.y << del << rim_pos.z << del;
        m_outf << rim_vel.x << del << rim_vel.y << del << rim_vel.z << del;
        m_outf << chassis_pos.x << del << chassis_pos.y << del << chassis_pos.z << del;
        m_outf << rfrc_prsm.x << del << rfrc_prsm.y << del << rfrc_prsm.z << del;
        m_outf << rtrq_prsm.x << del << rtrq_prsm.y << del << rtrq_prsm.z << del;
        m_outf << rfrc_act.x << del << rfrc_act.y << del << rfrc_act.z << del;
        m_outf << rtrq_act.x << del << rtrq_act.y << del << rtrq_act.z << del;
        m_outf << rfrc_motor.x << del << rfrc_motor.y << del << rfrc_motor.z << del;
        m_outf << std::endl;
    }

    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", rig_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    WriteStateInformation(csv);
	WriteMeshInformation(csv); // Connectivity and strain state
    csv.write_to_file(filename);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::WriteCheckpoint() {
    utils::CSV_writer csv(" ");
    WriteStateInformation(csv);
    csv.write_to_file(m_checkpoint_filename);

    std::cout << "[Rig node    ] write checkpoint ===> " << m_checkpoint_filename << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::WriteStateInformation(utils::CSV_writer& csv) {
    // Write number of bodies
    csv << "4" << std::endl;

    // Write body state information
    csv << m_chassis->GetIdentifier() << m_chassis->GetPos() << m_chassis->GetRot() << m_chassis->GetPos_dt()
        << m_chassis->GetRot_dt() << std::endl;
    csv << m_set_toe->GetIdentifier() << m_set_toe->GetPos() << m_set_toe->GetRot() << m_set_toe->GetPos_dt()
        << m_set_toe->GetRot_dt() << std::endl;
    csv << m_rim->GetIdentifier() << m_rim->GetPos() << m_rim->GetRot() << m_rim->GetPos_dt() << m_rim->GetRot_dt()
        << std::endl;
	csv << m_axle->GetIdentifier() << m_axle->GetPos() << m_axle->GetRot() << m_axle->GetPos_dt() << m_axle->GetRot_dt()
		<< std::endl;

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
    csv << mesh->GetNnodes() << mesh->GetDOF() << mesh->GetDOF_w() << std::endl;

    // Write mesh vertex positions and velocities
    for (int ix = 0; ix < x.GetLength(); ix++)
        csv << x(ix) << std::endl;
    for (int iv = 0; iv < v.GetLength(); iv++)
        csv << v(iv) << std::endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void RigNode::WriteMeshInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto my_mesh = m_tire->GetMesh();

    // Vector to roughly interpolate strain information
    std::vector<std::vector<int>> NodeNeighborElement;
    NodeNeighborElement.resize(my_mesh->GetNnodes());

    // Print tire mesh connectivity
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> myvector;
    myvector.resize(my_mesh->GetNnodes());
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<fea::ChNodeFEAbase>(my_mesh->GetNode(i));
    }
	csv << "\n Connectivity " << my_mesh->GetNelements() << 5 * my_mesh->GetNelements() << "\n";

	for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
		auto element = my_mesh->GetElement(iele);
		int nodeOrder[] = { 0, 1, 2, 3 };
		for (int myNodeN = 0; myNodeN < 4; myNodeN++) {
			auto nodeA = element->GetNodeN(nodeOrder[myNodeN]);
			std::vector<std::shared_ptr<fea::ChNodeFEAbase>>::iterator it;
			it = find(myvector.begin(), myvector.end(), nodeA);
			if (it != myvector.end()) {
				auto index = std::distance(myvector.begin(), it);
				csv << (unsigned int)index << " ";
				NodeNeighborElement[index].push_back(iele);
			}
		}
		csv << "\n";
	}

	// Print strain information: eps_xx, eps_yy, eps_xy averaged over 4 surrounding elements
	csv << "\n Vectors of Strains \n";
	for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
		double areaAve1 = 0, areaAve2 = 0, areaAve3 = 0;
		double myarea = 0;
        for (int j = 0; j < NodeNeighborElement[i].size(); j++) {
            int myelemInx = NodeNeighborElement[i][j];
            auto element = std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(myelemInx));
            ChVector<> StrainVector = element->EvaluateSectionStrains();
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
			myarea += dx * dy / 4;
			areaAve1 += StrainVector.x * dx * dy / 4;
			areaAve2 += StrainVector.y * dx * dy / 4;
			areaAve3 += StrainVector.z * dx * dy / 4;
		}
		csv << areaAve1 / myarea << " " << areaAve2 / myarea << " " << areaAve3 / myarea << "\n";
	}
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
void RigNode::PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indices) {
    std::cout << "[Rig node    ] contact forces" << std::endl;
    for (int i = 0; i < indices.size(); i++) {
        std::cout << "  id = " << indices[i] << "  force = " << forces[i].x << "  " << forces[i].y << "  "
                  << forces[i].z << std::endl;
    }
}
