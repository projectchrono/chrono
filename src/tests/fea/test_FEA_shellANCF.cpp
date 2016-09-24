//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <algorithm>
#include <string>

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChMapMatrix.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#ifdef CHRONO_SUPERLUMT
#include "chrono_superlumt/ChSolverSuperLUMT.h"
#endif

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_OPENMP_ENABLED
#include <omp.h>
#endif

using namespace chrono;
using namespace chrono::fea;

using std::cout;
using std::endl;

enum class solver_type
{
    MINRES,
    MKL,
    SUPERLUMT,
    MUMPS
};

// -----------------------------------------------------------------------------

int num_threads = 4;      // default number of threads
double step_size = 1e-3;  // integration step size
int num_steps = 20;       // number of integration steps
int skip_steps = 0;       // initial number of steps excluded from timing

int numDiv_x = 100;  // mesh divisions in X direction
int numDiv_y = 100;  // mesh divisions in Y direction
int numDiv_z = 1;   // mesh divisions in Z direction

std::string out_dir = "../TEST_SHELL_ANCF";  // name of output directory
bool output = true;                         // generate output file?
bool verbose = true;                        // verbose output?

                                            // -----------------------------------------------------------------------------

void RunModel(solver_type solver,              // use MKL solver (if available)
              bool use_adaptiveStep,     // allow step size reduction
              bool use_modifiedNewton,   // use modified Newton method
              const std::string& suffix  // output filename suffix
) {

    cout << endl;
    cout << "===================================================================" << endl;
    cout << "Solver:          ";
    switch (solver)
    {
    case solver_type::MINRES:
        cout << "MINRES";
        break;
    case solver_type::MKL:
        cout << "MKL";
        break;
    case solver_type::SUPERLUMT:
        cout << "SUPERLUMT";
        break;
    case solver_type::MUMPS:
        cout << "MUMPS";
        break;
    default: break;
    }
    cout << endl;
    cout << "Adaptive step:   " << (use_adaptiveStep ? "Yes" : "No") << endl;
    cout << "Modified Newton: " << (use_modifiedNewton ? "Yes" : "No") << endl;
    cout << endl;
    cout << "Mesh divisions:  " << numDiv_x << " x " << numDiv_y << endl;
    cout << endl;

    // Create the physical system
    ChSystem my_system;

    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1.0;
    double plate_lenght_y = 1.0;
    double plate_lenght_z = 0.04;  // small thickness
                                   // Specification of the mesh
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    //(1+1) is the number of nodes in the z direction
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1);  // Or *(numDiv_z+1) for multilayer
                                                          // Element dimensions (uniform grid)
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Create and add the nodes
    for (int i = 0; i < TotalNumNodes; i++) {
        // Parametric location and direction of nodal coordinates
        double loc_x = (i % (numDiv_x + 1)) * dx;
        double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        double loc_z = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz;

        double dir_x = 0;
        double dir_y = 0;
        double dir_z = 1;

        // Create the node
        auto node = std::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));
        node->SetMass(0);
        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // Create an isotropic material
    // Only one layer
    double rho = 500;
    double E = 2.1e7;
    double nu = 0.3;
    auto mat = std::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Definition of nodes forming an element
        int node0 = (i / (numDiv_x)) * (N_x)+i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x)+i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x)+i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x)+i % numDiv_x + N_x;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

        // Element length is a fixed number in both direction. (uniform distribution of nodes in both directions)
        element->SetDimensions(dx, dy);
        // Single layer
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);  // Thickness: dy;  Ply angle: 0.
                                                          // Set other element properties
        element->SetAlphaDamp(0.0);   // Structural damping for this
        element->SetGravityOn(true);  // element calculates its own gravitational load
                                      // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Switch off mesh class gravity (ANCF shell elements have a custom implementation)
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Mark completion of system construction
    my_system.SetupInitial();

#ifdef CHRONO_MKL
    ChSolverMKL<>* mkl_solver_stab = nullptr;
    ChSolverMKL<>* mkl_solver_speed = nullptr;
    ////ChSolverMKL<ChMapMatrix>* mkl_solver_stab = nullptr;
    ////ChSolverMKL<ChMapMatrix>* mkl_solver_speed = nullptr;
#endif

#ifdef CHRONO_SUPERLUMT
    ChSolverSuperLUMT<>* superlumt_solver_stab = nullptr;
    ChSolverSuperLUMT<>* superlumt_solver_speed = nullptr;
    ////ChSolverSuperLUMT<ChMapMatrix>* superlumt_solver_stab = nullptr;
    ////ChSolverSuperLUMT<ChMapMatrix>* superlumt_solver_speed = nullptr;
#endif

#ifdef CHRONO_MUMPS
    ChSolverMumps* mumps_solver_stab = nullptr;
    ChSolverMumps* mumps_solver_speed = nullptr;
#endif


    // Set up solver
    switch (solver)
    {
    case solver_type::MINRES:
    {
        my_system.SetSolverType(ChSystem::SOLVER_MINRES);
        auto msolver = static_cast<ChSolverMINRES*>(my_system.GetSolverSpeed());
        msolver->SetDiagonalPreconditioning(true);
        my_system.SetMaxItersSolverSpeed(100);
        my_system.SetTolForce(1e-10);
    }
    break;
    case solver_type::MKL:
#ifdef CHRONO_MKL
        mkl_solver_stab = new ChSolverMKL<>;
        mkl_solver_speed = new ChSolverMKL<>;
        ////mkl_solver_stab = new ChSolverMKL<ChMapMatrix>;
        ////mkl_solver_speed = new ChSolverMKL<ChMapMatrix>;
        my_system.ChangeSolverStab(mkl_solver_stab);
        my_system.ChangeSolverSpeed(mkl_solver_speed);
        mkl_solver_speed->SetSparsityPatternLock(true);
        mkl_solver_stab->SetSparsityPatternLock(true);
        mkl_solver_speed->SetVerbose(verbose);
        mkl_solver_speed->ForceSparsityPatternUpdate();
#endif
        break;
    case solver_type::SUPERLUMT:
#ifdef CHRONO_SUPERLUMT
        superlumt_solver_stab = new ChSolverSuperLUMT<>;
        superlumt_solver_speed = new ChSolverSuperLUMT<>;
        ////superlumt_solver_stab = new ChSolverSuperLUMT<ChMapMatrix>;
        ////superlumt_solver_speed = new ChSolverSuperLUMT<ChMapMatrix>;
        my_system.ChangeSolverStab(superlumt_solver_stab);
        my_system.ChangeSolverSpeed(superlumt_solver_speed);
        superlumt_solver_speed->SetSparsityPatternLock(true);
        superlumt_solver_stab->SetSparsityPatternLock(true);
        superlumt_solver_speed->SetVerbose(verbose);
        superlumt_solver_speed->ForceSparsityPatternUpdate();
#endif
        break;
    case solver_type::MUMPS:
#ifdef CHRONO_MUMPS
        mumps_solver_stab = new ChSolverMumps;
        mumps_solver_speed = new ChSolverMumps;
        my_system.ChangeSolverStab(mumps_solver_stab);
        my_system.ChangeSolverSpeed(mumps_solver_speed);
        mumps_solver_speed->SetVerbose(verbose);
#endif
        break;
    default:
        std::cout << "No solver set up" << std::endl;
        break;
    }

    // Set up integrator
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(100);
    mystepper->SetAbsTolerances(1e-5);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetStepControl(use_adaptiveStep);
    mystepper->SetModifiedNewton(use_modifiedNewton);
    mystepper->SetScaling(true);
    mystepper->SetVerbose(verbose);

    // Initialize the output stream and set precision.
    utils::CSV_writer out("\t");
    out.stream().setf(std::ios::scientific | std::ios::showpos);
    out.stream().precision(6);

    // Get handle to tracked node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes - 1));

    // Simulation loop
    double time_total = 0;
    double time_setup = 0;
    double time_setup_assembly = 0;
    double time_setup_solvercall = 0;
    double time_solve = 0;
    double time_solve_assembly = 0;
    double time_solve_solvercall = 0;
    double time_update = 0;
    double time_force = 0;
    double time_jacobian = 0;
    double time_skipped = 0;

    int num_iterations = 0;
    int num_setup_calls = 0;
    int num_solver_calls = 0;
    int num_force_calls = 0;
    int num_jacobian_calls = 0;

    for (int istep = 0; istep < num_steps; istep++) {
        if (verbose) {
            cout << "-------------------------------------------------------------------" << endl;
            cout << "STEP: " << istep << endl;
        }

        my_mesh->ResetCounters();
        my_mesh->ResetTimers();

#ifdef CHRONO_MKL
        if (solver == solver_type::MKL)
            mkl_solver_speed->ResetTimers();
#endif

#ifdef CHRONO_SUPERLUMT
        if (solver == solver_type::SUPERLUMT)
            superlumt_solver_speed->ResetTimers();
#endif

#ifdef CHRONO_MUMPS
        if (solver == solver_type::MUMPS)
            mumps_solver_speed->ResetTimers();
#endif

        my_system.DoStepDynamics(step_size);

        if (istep == 3 && (solver == solver_type::MKL || solver == solver_type::SUPERLUMT))
        {
#ifdef CHRONO_MKL
            mkl_solver_speed->SetSparsityPatternLock(true);
            mkl_solver_stab->SetSparsityPatternLock(true);
#endif
#ifdef CHRONO_SUPERLUMT
            superlumt_solver_speed->SetSparsityPatternLock(true);
            superlumt_solver_stab->SetSparsityPatternLock(true);
#endif
        }

        if (istep == skip_steps) {
            if (verbose)
                cout << "Resetting counters at step = " << istep << endl;
            time_skipped = time_total;
            time_total = 0;
            time_setup = 0;
            time_setup_assembly = 0;
            time_setup_solvercall = 0;
            time_solve = 0;
            time_solve_assembly = 0;
            time_solve_solvercall = 0;
            time_update = 0;
            time_force = 0;
            time_jacobian = 0;
            num_iterations = 0;
            num_setup_calls = 0;
            num_solver_calls = 0;
            num_force_calls = 0;
            num_jacobian_calls = 0;
        }

        time_total += my_system.GetTimerStep();
        time_setup += my_system.GetTimerSetup();
        time_solve += my_system.GetTimerSolver();
        time_update += my_system.GetTimerUpdate();

        // TODO: if it is OK to move timer in ChSolver we can avoid this switch
#ifdef CHRONO_MKL
        if (solver == solver_type::MKL) {
            time_setup_assembly += mkl_solver_speed->GetTimeSetup_Assembly();
            time_setup_solvercall += mkl_solver_speed->GetTimeSetup_SolverCall();
            time_solve_assembly += mkl_solver_speed->GetTimeSolve_Assembly();
            time_solve_solvercall += mkl_solver_speed->GetTimeSolve_SolverCall();
        }
#endif
#ifdef CHRONO_SUPERLUMT
        if (solver == solver_type::SUPERLUMT) {
            time_setup_assembly += superlumt_solver_speed->GetTimeSetup_Assembly();
            time_setup_solvercall += superlumt_solver_speed->GetTimeSetup_SolverCall();
            time_solve_assembly += superlumt_solver_speed->GetTimeSolve_Assembly();
            time_solve_solvercall += superlumt_solver_speed->GetTimeSolve_SolverCall();
        }
#endif
#ifdef CHRONO_MUMPS
        if (solver == solver_type::MUMPS) {
            time_setup_assembly += mumps_solver_speed->GetTimeSetup_Assembly();
            time_setup_solvercall += mumps_solver_speed->GetTimeSetup_SolverCall();
            time_solve_assembly += mumps_solver_speed->GetTimeSolve_Assembly();
            time_solve_solvercall += mumps_solver_speed->GetTimeSolve_SolverCall();
        }
#endif
        time_force += my_mesh->GetTimeInternalForces();
        time_jacobian += my_mesh->GetTimeJacobianLoad();

        num_iterations += mystepper->GetNumIterations();
        num_setup_calls += mystepper->GetNumSetupCalls();
        num_solver_calls += mystepper->GetNumSolveCalls();

        num_force_calls += my_mesh->GetNumCallsInternalForces();
        num_jacobian_calls += my_mesh->GetNumCallsJacobianLoad();

        const ChVector<>& p = nodetip->GetPos();

        if (verbose) {
            cout << endl;
            cout << "t = " << my_system.GetChTime() << "  ";
            cout << "node: [ " << p.x << " " << p.y << " " << p.z << " ]  " << endl;
            cout << "step:  " << my_system.GetTimerStep() << endl;
            cout << "setup: " << my_system.GetTimerSetup();
#ifdef CHRONO_MKL
            if (solver == solver_type::MKL) {
                cout << "  [assembly: " << mkl_solver_speed->GetTimeSetup_Assembly();
                cout << "  pardiso: " << mkl_solver_speed->GetTimeSetup_SolverCall() <<"]";
            }
#endif
#ifdef CHRONO_SUPERLUMT
            if (solver == solver_type::SUPERLUMT) {
                cout << "  [assembly: " << superlumt_solver_speed->GetTimeSetup_Assembly();
                cout << "  superlu_mt: " << superlumt_solver_speed->GetTimeSetup_SolverCall() << "]";
            }
#endif
            cout << endl;
            cout << "solve: " << my_system.GetTimerSolver() << "  ";
#ifdef CHRONO_MUMPS
            if (solver == solver_type::MUMPS) {
                cout << "  [assembly: " << mumps_solver_speed->GetTimeSolve_Assembly();
                cout << "  mumps: " << mumps_solver_speed->GetTimeSolve_SolverCall() << "]";
            }
#endif
            cout << endl << endl;
        }

        if (output) {
            out << my_system.GetChTime() << my_system.GetTimerStep() << nodetip->GetPos() << endl;
        }
    }

    double time_other = time_total - time_setup - time_solve - time_update - time_force - time_jacobian;

    cout << "-------------------------------------------------------------------" << endl;
    cout << "Total number of steps:        " << num_steps - skip_steps << endl;
    cout << "Total number of iterations:   " << num_iterations << endl;
    cout << "Total number of setup calls:  " << num_setup_calls << endl;
    cout << "Total number of solver calls: " << num_solver_calls << endl;
    cout << "Total number of internal force calls: " << num_force_calls << endl;
    cout << "Total number of Jacobian calls:       " << num_jacobian_calls << endl;
    cout << endl;
    cout << std::setprecision(3) << std::fixed;
    cout << "Total time: " << time_total << endl;
    cout << "  Setup:    " << time_setup << "\t (" << (time_setup / time_total) * 100 << "%)" << endl;
    if (solver == solver_type::MKL || solver == solver_type::SUPERLUMT || solver == solver_type::MUMPS) {
        cout << "    Assembly: " << time_setup_assembly << "\t (" << (time_setup_assembly / time_setup) * 100
            << "% setup)" << endl;
        cout << "    SolverCall:  " << time_setup_solvercall << "\t (" << (time_setup_solvercall / time_setup) * 100
            << "% setup)" << endl;
    }
    cout << "  Solve:    " << time_solve << "\t (" << (time_solve / time_total) * 100 << "%)" << endl;
    if (solver == solver_type::MKL || solver == solver_type::SUPERLUMT || solver == solver_type::MUMPS) {
        cout << "    Assembly: " << time_solve_assembly << "\t (" << (time_solve_assembly / time_solve) * 100
            << "% solve)" << endl;
        cout << "    SolverCall:  " << time_solve_solvercall << "\t (" << (time_solve_solvercall / time_solve) * 100
            << "% solve)" << endl;
    }
    if (solver == solver_type::MKL || solver == solver_type::SUPERLUMT || solver == solver_type::MUMPS) {
        cout << "  [TOT Assembly: " << time_setup_assembly+time_solve_assembly << "\t (" << ((time_setup_assembly + time_solve_assembly) / time_total) * 100
            << "% total)]" << endl;
        cout << "  [TOT SolverCall:  " << time_setup_solvercall + time_solve_solvercall << "\t (" << ((time_setup_solvercall + time_solve_solvercall) / time_total) * 100
            << "% total)]" << endl;
    }
    cout << "  Forces:   " << time_force << "\t (" << (time_force / time_total) * 100 << "%)" << endl;
    cout << "  Jacobian: " << time_jacobian << "\t (" << (time_jacobian / time_total) * 100 << "%)" << endl;
    cout << "  Update:   " << time_update << "\t (" << (time_update / time_total) * 100 << "%)" << endl;
    cout << "  Other:    " << time_other << "\t (" << (time_other / time_total) * 100 << "%)" << endl;
    cout << endl;
    cout << "Time for skipped steps (" << skip_steps << "): " << time_skipped << endl;

    if (output) {
        char name[100];
        std::sprintf(name, "%s/out_%s_%d.txt", out_dir.c_str(), suffix.c_str(), num_threads);
        cout << "Write output to: " << name << endl;
        out.write_to_file(name);
    }
}

int main(int argc, char* argv[]) {
    // Create output directory (if it does not already exist).
    if (output) {
        if (ChFileutils::MakeDirectory("../TEST_SHELL_ANCF") < 0) {
            GetLog() << "Error creating directory ../TEST_SHELL_ANCF\n";
            return 1;
        }
    }

#ifdef CHRONO_OPENMP_ENABLED
    // Set number of threads
    if (argc > 1)
        num_threads = std::stoi(argv[1]);
    num_threads = std::min(num_threads, CHOMPfunctions::GetNumProcs());
    CHOMPfunctions::SetNumThreads(num_threads);
    GetLog() << "Using " << num_threads << " thread(s)\n";
#else
    GetLog() << "No OpenMP\n";
#endif

    // Run simulations.
#ifdef CHRONO_SUPERLUMT
    RunModel(solver_type::SUPERLUMT, true, false, "SUPERLUMT_adaptive_full");     // MKL, adaptive step, full Newton
    RunModel(solver_type::SUPERLUMT, true, true, "SUPERLUMT_adaptive_modified");  // MKL, adaptive step, modified Newton
#endif

#ifdef CHRONO_MKL
    RunModel(solver_type::MKL, true, false, "MKL_adaptive_full");     // MKL, adaptive step, full Newton
    RunModel(solver_type::MKL, true, true, "MKL_adaptive_modified");  // MKL, adaptive step, modified Newton
#endif

#ifdef CHRONO_MUMPS
    RunModel(solver_type::MUMPS, true, false, "MUMPS_adaptive_full");     // MUMPS, adaptive step, full Newton
    RunModel(solver_type::MUMPS, true, true, "MUMPS_adaptive_modified");  // MUMPS, adaptive step, modified Newton
#endif

    RunModel(solver_type::MINRES, true, false, "MINRES_adaptive_full");     // MINRES, adaptive step, full Newton
    RunModel(solver_type::MINRES, true, true, "MINRES_adaptive_modified");  // MINRES, adaptive step, modified Newton

    return 0;
}
