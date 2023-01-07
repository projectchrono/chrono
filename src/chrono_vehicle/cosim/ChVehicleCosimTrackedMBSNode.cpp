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
// Definition of the base vehicle co-simulation tracked MBS NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <set>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_vehicle/cosim/ChVehicleCosimTrackedMBSNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// Construction of the base tracked MBS node
ChVehicleCosimTrackedMBSNode::ChVehicleCosimTrackedMBSNode() : ChVehicleCosimBaseNode("MBS"), m_fix_chassis(false) {
    // Default integrator and solver types
    m_int_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    m_slv_type = ChSolver::Type::BARZILAIBORWEIN;

    // Create the (sequential) SMC system
    m_system = new ChSystemSMC;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set default number of threads
    m_system->SetNumThreads(1, 1, 1);
}

ChVehicleCosimTrackedMBSNode::~ChVehicleCosimTrackedMBSNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTrackedMBSNode::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads, 1, 1);
}

void ChVehicleCosimTrackedMBSNode::SetIntegratorType(ChTimestepper::Type int_type, ChSolver::Type slv_type) {
    m_int_type = int_type;
    m_slv_type = slv_type;
#ifndef CHRONO_PARDISO_MKL
    if (m_slv_type == ChSolver::Type::PARDISO_MKL)
        m_slv_type = ChSolver::Type::BARZILAIBORWEIN;
#endif
#ifndef CHRONO_MUMPS
    if (m_slv_type == ChSolver::Type::MUMPS)
        m_slv_type = ChSolver::Type::BARZILAIBORWEIN;
#endif
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTrackedMBSNode::AttachDrawbarPullRig(std::shared_ptr<ChVehicleCosimDBPRig> rig) {
    m_DBP_rig = rig;
}

std::shared_ptr<ChVehicleCosimDBPRig> ChVehicleCosimTrackedMBSNode::GetDrawbarPullRig() const {
    return m_DBP_rig;
}

// -----------------------------------------------------------------------------
// Initialization of the tracked MBS node:
// - receive terrain height and dimensions
// - receive track shoe mass and size
// - construct and initialize MBS
// - send load mass on each wheel
// -----------------------------------------------------------------------------
void ChVehicleCosimTrackedMBSNode::Initialize() {
    // Invoke the base class method to figure out distribution of node types
    ChVehicleCosimBaseNode::Initialize();

    // There should be no TIRE nodes.
    if (m_num_tire_nodes > 0) {
        std::cerr << "Error: a tracked vehicle co-simulation should involve no TIRE nodes." << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Complete setup of the underlying ChSystem
    InitializeSystem();

    MPI_Status status;

    // Receive from TERRAIN node the initial terrain dimensions and the terrain height
    double init_dim[3];
    MPI_Recv(init_dim, 3, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    if (m_verbose) {
        cout << "[MBS node    ] Received initial terrain height = " << init_dim[0] << endl;
        cout << "[MBS node    ] Received terrain length =  " << init_dim[1] << endl;
        cout << "[MBS node    ] Received terrain width =  " << init_dim[2] << endl;
    }

    double terrain_height = init_dim[0];
    ChVector2<> terrain_size(init_dim[1], init_dim[2]);

    // Let derived classes construct and initialize their multibody system
    InitializeMBS(terrain_size, terrain_height);
    auto num_track_shoes = GetNumTrackShoes();

    GetChassisBody()->SetBodyFixed(m_fix_chassis);

    // Send to TERRAIN node the number of interacting objects (here, total number of track shoes)
    MPI_Send(&num_track_shoes, 1, MPI_INT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    // Send the communication interface type (rigid body) to the TERRAIN node
    char comm_type = 0;
    MPI_Send(&comm_type, 1, MPI_CHAR, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    // Send geometry for one track shoe
    SendGeometry(GetTrackShoeContactGeometry(), TERRAIN_NODE_RANK);

    // Send mass of one track shoe
    double mass = GetTrackShoeMass();
    MPI_Send(&mass, 1, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    // Initialize the DBP rig if one is attached
    if (m_DBP_rig) {
        m_DBP_rig->m_verbose = m_verbose;
        m_DBP_rig->Initialize(GetChassisBody(), GetSprocketAddendumRadius(), m_step_size);

        m_DBP_outf.open(m_node_out_dir + "/DBP.dat", std::ios::out);
        m_DBP_outf.precision(7);
        m_DBP_outf << std::scientific;

        OnInitializeDBPRig(m_DBP_rig->GetMotorFunction());
    }
}

// -----------------------------------------------------------------------------
// Complete setup of the underlying ChSystem based on any user-provided settings
// -----------------------------------------------------------------------------
void ChVehicleCosimTrackedMBSNode::InitializeSystem() {
    // Change solver
    switch (m_slv_type) {
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<ChSolverMumps>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::SPARSE_LU: {
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
            break;
        }
        case ChSolver::Type::SPARSE_QR: {
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
            break;
        }
        case ChSolver::Type::PSOR:
        case ChSolver::Type::PSSOR:
        case ChSolver::Type::PJACOBI:
        case ChSolver::Type::PMINRES:
        case ChSolver::Type::BARZILAIBORWEIN:
        case ChSolver::Type::APGD:
        case ChSolver::Type::GMRES:
        case ChSolver::Type::MINRES:
        case ChSolver::Type::BICGSTAB: {
            m_system->SetSolverType(m_slv_type);
            auto solver = std::dynamic_pointer_cast<ChIterativeSolver>(m_system->GetSolver());
            assert(solver);
            solver->SetMaxIterations(100);
            solver->SetTolerance(1e-10);
            break;
        }
        default: {
            cout << "Solver type not supported!" << endl;
            return;
        }
    }

    // Change integrator
    switch (m_int_type) {
        case ChTimestepper::Type::HHT:
            m_system->SetTimestepperType(ChTimestepper::Type::HHT);
            m_integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
            m_integrator->SetAlpha(-0.2);
            m_integrator->SetMaxiters(50);
            m_integrator->SetAbsTolerances(5e-05, 1.8e00);
            m_integrator->SetMode(ChTimestepperHHT::POSITION);
            m_integrator->SetScaling(true);
            m_integrator->SetVerbose(false);
            m_integrator->SetMaxItersSuccess(5);
            break;
        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Synchronization of the MBS node:
// - extract and send track shoe states
// - receive and apply vertex contact forces
// -----------------------------------------------------------------------------
void ChVehicleCosimTrackedMBSNode::Synchronize(int step_number, double time) {
    int num_shoes = (int)GetNumTrackShoes();
    std::vector<double> all_states(13 * num_shoes);
    std::vector<double> all_forces(6 * num_shoes);
    int start_idx;

    // Pack states of all track shoe bodies
    start_idx = 0;
    for (int i = 0; i < GetNumTracks(); i++) {
        for (int j = 0; j < GetNumTrackShoes(i); j++) {
            BodyState state = GetTrackShoeState(i, j);
            all_states[start_idx + 0] = state.pos.x();
            all_states[start_idx + 1] = state.pos.y();
            all_states[start_idx + 2] = state.pos.z();
            all_states[start_idx + 3] = state.rot.e0();
            all_states[start_idx + 4] = state.rot.e1();
            all_states[start_idx + 5] = state.rot.e2();
            all_states[start_idx + 6] = state.rot.e3();
            all_states[start_idx + 7] = state.lin_vel.x();
            all_states[start_idx + 8] = state.lin_vel.y();
            all_states[start_idx + 9] = state.lin_vel.z();
            all_states[start_idx + 10] = state.ang_vel.x();
            all_states[start_idx + 11] = state.ang_vel.y();
            all_states[start_idx + 12] = state.ang_vel.z();
            start_idx += 13;
        }
    }

    // Send track shoe states to the terrain node
    MPI_Send(all_states.data(), 13 * num_shoes, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);

    // Receive track shoe forces as applied to the center of the track shoe body.
    // Note that we assume this is the resultant wrench at the track shoe origin (expressed in absolute frame).
    MPI_Status status;
    MPI_Recv(all_forces.data(), 6 * num_shoes, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    // Apply track shoe forces on each individual track shoe body
    start_idx = 0;
    for (int i = 0; i < GetNumTracks(); i++) {
        for (int j = 0; j < GetNumTrackShoes(i); j++) {
            TerrainForce force;
            force.point = GetTrackShoeBody(i, j)->GetPos();
            force.force = ChVector<>(all_forces[start_idx + 0], all_forces[start_idx + 1], all_forces[start_idx + 2]);
            force.moment = ChVector<>(all_forces[start_idx + 3], all_forces[start_idx + 4], all_forces[start_idx + 5]);
            ApplyTrackShoeForce(i, j, force);
            start_idx += 6;
        }
    }
}

// -----------------------------------------------------------------------------
// Advance simulation of the MBS node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimTrackedMBSNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        PreAdvance();
        m_system->DoStepDynamics(h);
        if (m_DBP_rig) {
            m_DBP_rig->OnAdvance(step_size);
        }
        PostAdvance();
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

void ChVehicleCosimTrackedMBSNode::OutputData(int frame) {
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

void ChVehicleCosimTrackedMBSNode::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    utils::WriteVisualizationAssets(m_system, filename, true);
}

}  // end namespace vehicle
}  // end namespace chrono
