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

#include "chrono_vehicle/cosim/ChVehicleCosimWheeledMBSNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// Construction of the base wheeled MBS node
ChVehicleCosimWheeledMBSNode::ChVehicleCosimWheeledMBSNode() : ChVehicleCosimBaseNode("MBS"), m_fix_chassis(false) {
    // Default integrator and solver types
    m_int_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    m_slv_type = ChSolver::Type::BARZILAIBORWEIN;

    // Create the (sequential) SMC system
    m_system = new ChSystemSMC;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set default number of threads
    m_system->SetNumThreads(1, 1, 1);
}

ChVehicleCosimWheeledMBSNode::~ChVehicleCosimWheeledMBSNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimWheeledMBSNode::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads, 1, 1);
}

void ChVehicleCosimWheeledMBSNode::SetIntegratorType(ChTimestepper::Type int_type, ChSolver::Type slv_type) {
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

    // Complete setup of the underlying ChSystem
    InitializeSystem();

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
    ChVector2<> terrain_size(init_dim[1], init_dim[2]);

    // For each TIRE, receive the tire mass and radius
    std::vector<ChVector<>> tire_info;

    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        double tmp[3];
        MPI_Recv(tmp, 3, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);
        tire_info.push_back(ChVector<>(tmp[0], tmp[1], tmp[2]));
    }

    // Let derived classes construct and initialize their multibody system
    InitializeMBS(tire_info, terrain_size, terrain_height);
    auto num_spindles = GetNumSpindles();

    // There must be a number of TIRE nodes equal to the number of spindles.
    if (m_num_tire_nodes != num_spindles) {
        std::cerr << "Error: number of TIRE nodes (" << m_num_tire_nodes << ") different from number of spindles ("
                  << num_spindles << ")." << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    GetChassisBody()->SetBodyFixed(m_fix_chassis);

    // Send to TERRAIN node the number of interacting objects (here, number of spindles)
    MPI_Send(&num_spindles, 1, MPI_INT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

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
// Complete setup of the underlying ChSystem based on any user-provided settings
// -----------------------------------------------------------------------------
void ChVehicleCosimWheeledMBSNode::InitializeSystem() {
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
        spindle_force.force = ChVector<>(force_data[0], force_data[1], force_data[2]);
        spindle_force.moment = ChVector<>(force_data[3], force_data[4], force_data[5]);
        ApplySpindleForce(i, spindle_force);

        if (m_verbose)
            cout << "[MBS node    ] Recv: spindle force (" << i << ") = " << spindle_force.force << endl;
    }
}

// -----------------------------------------------------------------------------
// Advance simulation of the MBS node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimWheeledMBSNode::Advance(double step_size) {
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
