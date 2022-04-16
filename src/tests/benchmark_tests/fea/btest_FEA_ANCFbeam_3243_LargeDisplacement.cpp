// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Mike Taylor and Radu Serban
// =============================================================================
//
// Large Displacement, Large Deformation, Linear Isotropic Benchmark test for
// ANCF beam elements - Spinning 3D Pendulum with Initial Conditions and a
// Square cross section
//
// With Modifications from:
// J. Gerstmayr and A.A. Shabana. Analysis of thin beams and cables using the
// absolute nodal co-ordinate formulation.Nonlinear Dynamics, 45(1) : 109{130,
// 2006.
//
// =============================================================================

#include <string>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_PARDISOPROJECT
    #include "chrono_pardisoproject/ChSolverPardisoProject.h"
#endif

using namespace chrono;
using namespace chrono::fea;

enum class SolverType { MINRES, SparseLU, SparseQR, MKL, MUMPS, PARDISO_PROJECT };

// =============================================================================

#define NUM_SKIP_STEPS 10  // number of steps for hot start
#define NUM_SIM_STEPS 100  // number of simulation steps for each benchmark
#define REPEATS 10

// =============================================================================

class ANCFBeamTest {
  public:
    ANCFBeamTest(int num_elements, SolverType solver_type, int NumThreads, bool useContInt);

    ~ANCFBeamTest() { delete m_system; }

    ChSystem* GetSystem() { return m_system; }

    void ExecuteStep() { m_system->DoStepDynamics(1e-3); }

    void SimulateVis();

    ChVector<> GetBeamEndPointPos() { return m_nodeEndPoint->GetPos(); }

    void RunTimingTest(ChMatrixNM<double, 4, 19>& timing_stats, const std::string& test_name);

  protected:
    ChSystemSMC* m_system;
    std::shared_ptr<ChNodeFEAxyzDDD> m_nodeEndPoint;
    SolverType m_SolverType;
    int m_NumElements;
    int m_NumThreads;
};

ANCFBeamTest::ANCFBeamTest(int num_elements, SolverType solver_type, int NumThreads, bool useContInt) {
    m_SolverType = solver_type;
    m_NumElements = num_elements;
    m_NumThreads = NumThreads;
    m_system = new ChSystemSMC();
    m_system->Set_G_acc(ChVector<>(0, 0, -9.80665));
    m_system->SetNumThreads(NumThreads, 1, NumThreads);

    // Set solver parameters
#ifndef CHRONO_PARDISO_MKL
    if (solver_type == SolverType::MKL) {
        solver_type = SolverType::SparseLU;
        std::cout << "WARNING! Chrono::MKL not enabled. Forcing use of SparseLU solver" << std::endl;
    }
#endif

#ifndef CHRONO_MUMPS
    if (solver_type == SolverType::MUMPS) {
        solver_type = SolverType::SparseLU;
        std::cout << "WARNING! Chrono::MUMPS not enabled. Forcing use of SparseLU solver" << std::endl;
    }
#endif

#ifndef CHRONO_PARDISOPROJECT
    if (solver_type == SolverType::PARDISO_PROJECT) {
        solver_type = SolverType::SparseLU;
        std::cout << "WARNING! Chrono::PARDISO_PROJECT not enabled. Forcing use of SparseLU solver" << std::endl;
    }
#endif

    switch (solver_type) {
        case SolverType::MINRES: {
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            m_system->SetSolver(solver);
            solver->SetMaxIterations(100);
            solver->SetTolerance(1e-10);
            solver->EnableDiagonalPreconditioner(true);
            solver->SetVerbose(false);
            m_system->SetSolverForceTolerance(1e-10);
            break;
        }
        case SolverType::MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>(NumThreads);
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<ChSolverMumps>(NumThreads);
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::PARDISO_PROJECT: {
#ifdef CHRONO_PARDISOPROJECT
            auto solver = chrono_types::make_shared<ChSolverPardisoProject>(NumThreads);
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::SparseLU: {
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
            break;
        }
        case SolverType::SparseQR: {
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
            break;
        }
    }

    // Set up integrator
    m_system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-5);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetScaling(true);
    integrator->SetVerbose(false);
    integrator->SetModifiedNewton(true);

    // Mesh properties
    double f = 5.0;
    double length = 1;                   // m
    double area = 10e-6 * (f * f);       // m^2
    double width = std::sqrt(area);      // m
    double thickness = std::sqrt(area);  // m
    double rho = 8000 / (f * f);         // kg/m^3
    double E = 10e7 / (std::pow(f, 4));  // Pa
    double nu = 0;                       // Poisson effect neglected for this model
    double k1 =
        10 * (1 + nu) / (12 + 11 * nu);  // Timoshenko shear correction coefficient for a rectangular cross-section
    double k2 = k1;                      // Timoshenko shear correction coefficient for a rectangular cross-section

    double omega_z = -4.0;  // rad/s initial angular velocity about the Z axis

    auto material = chrono_types::make_shared<ChMaterialBeamANCF>(rho, E, nu, k1, k2);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    m_system->Add(mesh);

    // Setup visualization
    auto vis_surf = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_surf->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_surf->SetWireframe(true);
    vis_surf->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(vis_surf);

    auto vis_node = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_node->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_node->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_node->SetSymbolsThickness(0.01);
    mesh->AddVisualShapeFEA(vis_node);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    int num_nodes = num_elements + 1;
    double dx = length / (num_nodes - 1);

    // Setup beam cross section gradients to initially align with the global y and z directions
    ChVector<> dir1(1, 0, 0);
    ChVector<> dir2(0, 1, 0);
    ChVector<> dir3(0, 0, 1);

    // Create a grounded body to connect the 3D pendulum to
    auto grounded = chrono_types::make_shared<ChBody>();
    grounded->SetBodyFixed(true);
    m_system->Add(grounded);

    // Create the first node and fix only its position to ground (Spherical Joint constraint)
    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector<>(0, 0, 0.0), dir1, dir2, dir3);
    mesh->AddNode(nodeA);
    auto pos_constraint = chrono_types::make_shared<ChLinkPointFrame>();
    pos_constraint->Initialize(nodeA, grounded);  // body to be connected to
    m_system->Add(pos_constraint);

    for (int i = 1; i <= num_elements; i++) {
        auto nodeB = chrono_types::make_shared<ChNodeFEAxyzDDD>(ChVector<>(dx * i, 0, 0), dir1, dir2, dir3);
        nodeB->SetPos_dt(ChVector<>(0, omega_z * (dx * (2 * i)), 0));
        mesh->AddNode(nodeB);

        auto element = chrono_types::make_shared<ChElementBeamANCF_3243>();
        element->SetNodes(nodeA, nodeB);
        element->SetDimensions(dx, thickness, width);
        element->SetMaterial(material);
        element->SetAlphaDamp(0.01);

        // By default the "continuous" integration style of calculation method is used since it is typically faster.
        // Switch to the alternative "pre-integration" style of internal force calculation if selected by the user.
        if (!useContInt)
            element->SetIntFrcCalcMethod(ChElementBeamANCF_3243::IntFrcMethod::PreInt);

        mesh->AddElement(element);

        nodeA = nodeB;
    }

    m_nodeEndPoint = nodeA;
}

void ANCFBeamTest::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    m_system->SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ANCF Beam 3242");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(-0.4, 0.4, 0.4), ChVector<>(0, 0, 0));

    while (vis->Run()) {
        std::cout << "Time(s): " << this->m_system->GetChTime() << "  Tip Pos(m): " << this->GetBeamEndPointPos()
                  << std::endl;
        vis->BeginScene();
        vis->DrawAll();
        irrlicht::tools::drawSegment(vis->GetVideoDriver(), ChVector<>(0), ChVector<>(1, 0, 0),
                                     irr::video::SColor(255, 255, 0, 0));
        irrlicht::tools::drawSegment(vis->GetVideoDriver(), ChVector<>(0), ChVector<>(0, 1, 0),
                                     irr::video::SColor(255, 0, 255, 0));
        irrlicht::tools::drawSegment(vis->GetVideoDriver(), ChVector<>(0), ChVector<>(0, 0, 1),
                                     irr::video::SColor(255, 0, 0, 255));
        ExecuteStep();
        vis->EndScene();
    }
#endif
}

void ANCFBeamTest::RunTimingTest(ChMatrixNM<double, 4, 19>& timing_stats, const std::string& test_name) {
    // Timing Results entries (in seconds)
    //  - "Step_Total"
    //  - "Step_Advance"
    //  - "Step_Update"
    //  - "LS_Jacobian"
    //  - "LS_Setup"
    //  - "LS_Setup_Asm"
    //  - "LS_Setup_Solver"
    //  - "LS_Solve"
    //  - "LS_Solve_Asm"
    //  - "LS_Solve_Solver"
    //  - "CD_Total"
    //  - "CD_Broad"
    //  - "CD_Narrow"
    //  - "FEA_InternalFrc"
    //  - "FEA_Jacobian"
    //  - "FEA_InternalFrc_Calls"
    //  - "FEA_Jacobian_Calls"

    // Reset timing results since the results will be accumulated into this vector
    ChMatrixNM<double, REPEATS, 19> timing_results;
    timing_results.setZero();

    // Run the requested number of steps to warm start the system, but do not collect any timing information
    for (int i = 0; i < NUM_SKIP_STEPS; i++) {
        ExecuteStep();
    }

    // Time the requested number of steps, collecting timing information (system is not restarted between collections)
    auto LS = std::dynamic_pointer_cast<ChDirectSolverLS>(GetSystem()->GetSolver());
    auto MeshList = GetSystem()->Get_meshlist();
    for (int r = 0; r < REPEATS; r++) {
        for (int i = 0; i < NUM_SIM_STEPS; i++) {
            for (auto& Mesh : MeshList) {
                Mesh->ResetTimers();
                Mesh->ResetCounters();
            }
            if (LS != NULL) {  // Direct Solver
                LS->ResetTimers();
            }
            GetSystem()->ResetTimers();

            ExecuteStep();

            timing_results(r, 0) += GetSystem()->GetTimerStep();
            timing_results(r, 1) += GetSystem()->GetTimerAdvance();
            timing_results(r, 2) += GetSystem()->GetTimerUpdate();

            timing_results(r, 3) += GetSystem()->GetTimerJacobian();
            timing_results(r, 4) += GetSystem()->GetTimerLSsetup();
            timing_results(r, 7) += GetSystem()->GetTimerLSsolve();
            timing_results(r, 10) += GetSystem()->GetTimerCollision();
            timing_results(r, 11) += GetSystem()->GetTimerCollisionBroad();
            timing_results(r, 12) += GetSystem()->GetTimerCollisionNarrow();

            if (LS != NULL) {  // Direct Solver
                timing_results(r, 5) += LS->GetTimeSetup_Assembly();
                timing_results(r, 6) += LS->GetTimeSetup_SolverCall();
                timing_results(r, 8) += LS->GetTimeSolve_Assembly();
                timing_results(r, 9) += LS->GetTimeSolve_SolverCall();
            }

            // Accumulate the internal force and Jacobian timers across all the FEA mesh containers
            for (auto& Mesh : MeshList) {
                timing_results(r, 13) += Mesh->GetTimeInternalForces();
                timing_results(r, 14) += Mesh->GetTimeJacobianLoad();
                timing_results(r, 15) += Mesh->GetNumCallsInternalForces();
                timing_results(r, 16) += Mesh->GetNumCallsJacobianLoad();
            }
        }
        timing_results(r, 17) = (timing_results(r, 13) * 1e6) / (timing_results(r, 15) * double(m_NumElements));
        timing_results(r, 18) = (timing_results(r, 14) * 1e6) / (timing_results(r, 16) * double(m_NumElements));
    }

    // Scale times from s to ms
    timing_results.block(0, 0, REPEATS, 15) *= 1e3;

    // Compute statistics (min, max, median, mean, std deviation)
    timing_stats.row(0) = timing_results.colwise().minCoeff();
    timing_stats.row(1) = timing_results.colwise().maxCoeff();
    timing_stats.row(2) = timing_results.colwise().mean();
    for (auto c = 0; c < timing_stats.cols(); c++) {  // compute the standard deviation column by column
        timing_stats(3, c) = std::sqrt((timing_results.col(c).array() - timing_results.col(c).mean()).square().sum() /
                                       (timing_results.col(c).size() - 1));
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << test_name << " - Num_Elements: " << m_NumElements << " - Linear_Solver: ";
    switch (m_SolverType) {
        case SolverType::MINRES:
            std::cout << "MINRES";
            ;
            break;
        case SolverType::MKL:
            std::cout << "MKL";
            ;
            break;
        case SolverType::MUMPS:
            std::cout << "MUMPS";
            ;
            break;
        case SolverType::PARDISO_PROJECT:
            std::cout << "PARDISO_PROJECT";
            ;
            break;
        case SolverType::SparseLU:
            std::cout << "SparseLU";
            ;
            break;
        case SolverType::SparseQR:
            std::cout << "SparseQR";
            ;
            break;
    }
    std::cout << " - Requested_Threads: " << m_NumThreads;
    std::cout << " - Tip_Displacement_End = " << GetBeamEndPointPos().z() << std::endl;

    std::cout << "Step_Total "
              << "Step_Advance "
              << "Step_Update "
              << "LS_Jacobian "
              << "LS_Setup "
              << "LS_Setup_Asm "
              << "LS_Setup_Solver "
              << "LS_Solve "
              << "LS_Solve_Asm "
              << "LS_Solve_Solver "
              << "CD_Total "
              << "CD_Broad "
              << "CD_Narrow "
              << "FEA_InternalFrc "
              << "FEA_Jacobian "
              << "FEA_InternalFrc_Calls "
              << "FEA_Jacobian_Calls "
              << "FEA_InternalFrc_AvgFunctionCall_us "
              << "FEA_Jacobian_AvgFunctionCall_us" << std::endl;
    for (int r = 0; r < REPEATS; r++) {
        std::cout << "Run_" << r << ":\t" << timing_results.row(r) << std::endl;
    }
    std::cout << "Min:\t" << timing_stats.row(0) << std::endl;
    std::cout << "Max:\t" << timing_stats.row(1) << std::endl;
    std::cout << "Mean:\t" << timing_stats.row(2) << std::endl;
    std::cout << "StdDev:\t" << timing_stats.row(3) << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc > 1) {
        // If any input arguments are passed into the program, visualize the test system using a default set of inputs
#ifdef CHRONO_IRRLICHT
        ANCFBeamTest test(8, SolverType::SparseLU, 1, true);
        test.SimulateVis();
#endif
    } else {
        // Run the batch timing studies using the settings below

        // Setup the vector containing the numbers of elements to test
        ChVectorN<int, 1> num_els;
        num_els << 32;

        // Setup the vector containing the specific linear solvers to test
        std::vector<SolverType> Solver = {SolverType::MINRES, SolverType::SparseLU, SolverType::SparseQR};
#ifdef CHRONO_PARDISO_MKL
        Solver.push_back(SolverType::MKL);
#endif

#ifdef CHRONO_MUMPS
        Solver.push_back(SolverType::MUMPS);
#endif

#ifdef CHRONO_PARDISOPROJECT
        Solver.push_back(SolverType::PARDISO_PROJECT);
#endif

        // Set the limit on the number of OpenMP threads to test up to.
        int MaxThreads = ChOMP::GetMaxThreads();
        std::cout << "GetNumProcs:\t" << ChOMP::GetNumProcs() << " Max Threads = " << MaxThreads << std::endl;

        ChMatrixNM<double, 4, 19> timing_stats;

        // Run timing studies on all of the combinations of test conditions specified
        for (const auto& ls : Solver) {
            for (auto i = 0; i < num_els.size(); i++) {
                int NumThreads = 1;
                bool run = true;
                while (run) {
                    {
                        ANCFBeamTest test(num_els(i), ls, NumThreads, true);
                        test.RunTimingTest(timing_stats, "ChElementBeamANCF_3243_ContInt");
                    }
                    {
                        ANCFBeamTest test(num_els(i), ls, NumThreads, false);
                        test.RunTimingTest(timing_stats, "ChElementBeamANCF_3243_PreInt");
                    }

                    if (NumThreads == MaxThreads)
                        run = false;

                    // Double the number of threads for the next run up to the specified maximum number of OpenMP
                    // threads
                    NumThreads *= 2;
                    if (NumThreads > MaxThreads)
                        NumThreads = MaxThreads;
                }
            }
        }
    }

    return (0);
}
