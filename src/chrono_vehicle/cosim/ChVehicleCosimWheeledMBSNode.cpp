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
// Authors: Radu Serban
// =============================================================================
//
// Definition of the base vehicle co-simulation wheeled MBS NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <set>
#include <vector>

#include "chrono/solver/ChSolverBB.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// Construction of the base wheeled MBS node
ChVehicleCosimWheeledMBSNode::ChVehicleCosimWheeledMBSNode() : ChVehicleCosimBaseNode("MBS"), m_fix_chassis(false) {
    // Create the (sequential) SMC system with default collision system
    m_system = new ChSystemSMC;
    m_system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_system->SetGravitationalAcceleration(ChVector3d(0, 0, m_gacc));

    // Set default solver and integrator
    auto solver = chrono_types::make_shared<ChSolverBB>();
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-10);
    m_system->SetSolver(solver);

    m_system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Set default number of threads
    m_system->SetNumThreads(1);
}

ChVehicleCosimWheeledMBSNode::~ChVehicleCosimWheeledMBSNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimWheeledMBSNode::AttachDrawbarPullRig(std::shared_ptr<ChVehicleCosimDBPRig> rig) {
    m_DBP_rig = rig;
}

std::shared_ptr<ChVehicleCosimDBPRig> ChVehicleCosimWheeledMBSNode::GetDrawbarPullRig() const {
    return m_DBP_rig;
}

// -----------------------------------------------------------------------------
// Initialization of the wheeled MBS node:
// - receive terrain height and dimensions
// - receive tire mass and radius
// - construct and initialize MBS
// - send load mass on each wheel
// -----------------------------------------------------------------------------
void ChVehicleCosimWheeledMBSNode::Initialize() {
    // Invoke the base class method to figure out distribution of node types
    ChVehicleCosimBaseNode::Initialize();

    MPI_Status status;

    // Receive from TERRAIN node the initial terrain dimensions and the terrain height
    double init_dim[3];
    MPI_Recv(init_dim, 3, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    if (m_verbose) {
        cout << "[MBS node    ] Recv: initial terrain height = " << init_dim[0] << endl;
        cout << "[MBS node    ] Recv: terrain length =  " << init_dim[1] << endl;
        cout << "[MBS node    ] Recv: terrain width =  " << init_dim[2] << endl;
    }

    double terrain_height = init_dim[0];
    ChVector2d terrain_size(init_dim[1], init_dim[2]);

    // Receive from terrain node path information
    unsigned int num_path_points;
    MPI_Recv(&num_path_points, 1, MPI_INT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);
    if (num_path_points > 0) {
        std::vector<double> all_points(3 * num_path_points);
        MPI_Recv(all_points.data(), 3 * num_path_points, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);
        std::vector<ChVector3d> path_points;
        unsigned int start_idx = 0;
        for (unsigned int i = 0; i < num_path_points; i++) {
            path_points.push_back({all_points[start_idx + 0], all_points[start_idx + 1], all_points[start_idx + 2]});
            start_idx += 3;
        }
        m_path = chrono_types::make_shared<ChBezierCurve>(path_points);
    }

    if (m_verbose) 
        cout << "[MBS node    ] Recv: num. path points =  " << num_path_points << endl;

    // Let derived classes construct and initialize their multibody system
    InitializeMBS(terrain_size, terrain_height);
    auto num_spindles = GetNumSpindles();

    // There must be a number of TIRE nodes equal to the number of spindles.
    if (m_num_tire_nodes != (unsigned int)num_spindles) {
        std::cerr << "Error: number of TIRE nodes (" << m_num_tire_nodes << ") different from number of spindles ("
                  << num_spindles << ")." << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    GetChassisBody()->SetFixed(m_fix_chassis);

    // For each TIRE, send initial location
    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        // Send wheel state to the tire node
        BodyState state = GetSpindleState(i);
        double loc_data[] = {state.pos.x(), state.pos.y(), state.pos.z()};

        MPI_Send(loc_data, 3, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD);

        if (m_verbose)
            cout << "[MBS node    ] Send: spindle initial location (" << i << ") = " << state.pos << endl;
    }

    // For each TIRE, receive the tire mass, radius, and width
    std::vector<ChVector3d> tire_info;

    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        double tmp[3];
        MPI_Recv(tmp, 3, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);
        tire_info.push_back(ChVector3d(tmp[0], tmp[1], tmp[2]));
    }

    // Incorporate information on tire mass, radius, and width.
    // This is deferred to this point so that the MBS system could be initialized first and correct spindle locations
    // sent to the tire nodes. Only after their own initialization can the tire nodes send back here the tire info.
    ApplyTireInfo(tire_info);

    // Send to TERRAIN node the number of interacting objects (here, number of spindles)
    MPI_Send(&num_spindles, 1, MPI_INT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    // Send to TERRAIN node the initial locations of all interacting objects (here, tire/spindle initial locations)
    std::vector<double> all_locations(3 * num_spindles);
    unsigned int start_idx = 0;
    for (unsigned int i = 0; i < num_spindles; i++) {
        // Send wheel state to the tire node
        BodyState state = GetSpindleState(i);
        all_locations[start_idx + 0] = state.pos.x();
        all_locations[start_idx + 1] = state.pos.y();
        all_locations[start_idx + 2] = state.pos.z();
        start_idx += 3;
    }
    MPI_Send(all_locations.data(), 3 * num_spindles, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    // For each tire:
    // - cache the spindle body
    // - get the load on the wheel and send to TIRE node
    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        double load = GetSpindleLoad(i);
        MPI_Send(&load, 1, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD);
    }

    // Initialize the DBP rig if one is attached
    if (m_DBP_rig) {
        m_DBP_rig->m_verbose = m_verbose;
        m_DBP_rig->Initialize(GetChassisBody(), tire_info[0].y(), m_step_size);

        m_DBP_outf.open(m_node_out_dir + "/DBP.dat", std::ios::out);
        m_DBP_outf.precision(7);
        m_DBP_outf << std::scientific;

        OnInitializeDBPRig(m_DBP_rig->GetMotorFunction());
    }
}

// -----------------------------------------------------------------------------
// Synchronization of the MBS node:
// - extract and send tire mesh vertex states
// - receive and apply vertex contact forces
// -----------------------------------------------------------------------------
void ChVehicleCosimWheeledMBSNode::Synchronize(int step_number, double time) {
    MPI_Status status;

    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        // Send wheel state to the tire node
        BodyState state = GetSpindleState(i);
        double state_data[] = {
            state.pos.x(),     state.pos.y(),     state.pos.z(),                      //
            state.rot.e0(),    state.rot.e1(),    state.rot.e2(),    state.rot.e3(),  //
            state.lin_vel.x(), state.lin_vel.y(), state.lin_vel.z(),                  //
            state.ang_vel.x(), state.ang_vel.y(), state.ang_vel.z()                   //
        };

        MPI_Send(state_data, 13, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD);

        if (m_verbose)
            cout << "[MBS node    ] Send: spindle position (" << i << ") = " << state.pos << endl;

        // Receive spindle force as applied to the center of the spindle/wheel.
        // Note that we assume this is the resultant wrench at the wheel origin (expressed in absolute frame).
        double force_data[6];
        MPI_Recv(force_data, 6, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD, &status);

        TerrainForce spindle_force;
        spindle_force.point = GetSpindleBody(i)->GetPos();
        spindle_force.force = ChVector3d(force_data[0], force_data[1], force_data[2]);
        spindle_force.moment = ChVector3d(force_data[3], force_data[4], force_data[5]);
        ApplySpindleForce(i, spindle_force);

        if (m_verbose)
            cout << "[MBS node    ] Recv: spindle force (" << i << ") = " << spindle_force.force << endl;
    }

    // Send vehicle location to terrain node
    MPI_Send(GetChassisBody()->GetPos().data(), 3, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);
}

// -----------------------------------------------------------------------------
// Advance simulation of the MBS node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimWheeledMBSNode::Advance(double step_size) {
    // Advance state of the vehicle system for one step
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        PreAdvance(h);
        m_system->DoStepDynamics(h);
        if (m_DBP_rig) {
            m_DBP_rig->OnAdvance(step_size);
        }
        PostAdvance(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

    // Possible rendering
    Render(step_size);
}

void ChVehicleCosimWheeledMBSNode::OutputData(int frame) {
    double time = m_system->GetChTime();

    // If a DBP rig is attached, output its results
    if (m_DBP_rig && time >= m_DBP_rig->m_delay_time) {
        std::string del("  ");

        m_DBP_outf << time << del;
        m_DBP_outf << m_DBP_rig->GetLinVel() << del << m_DBP_rig->GetAngVel() << del;
        m_DBP_outf << m_DBP_rig->GetSlip() << del << m_DBP_rig->GetFilteredSlip() << del;
        m_DBP_outf << m_DBP_rig->GetDBP() << del << m_DBP_rig->GetFilteredDBP() << del;

        m_DBP_outf << endl;
    }

    // Let derived classes perform specific output
    OnOutputData(frame);
}

void ChVehicleCosimWheeledMBSNode::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    utils::WriteVisualizationAssets(m_system, filename, true);
}

}  // end namespace vehicle
}  // end namespace chrono
