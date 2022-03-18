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
// ANCF shell elements - Simple Plate Pendulum
//
// With Modifications from:
// Aki M Mikkola and Ahmed A Shabana. A non-incremental finite element procedure
// for the analysis of large deformation of plates and shells in mechanical
// system applications. Multibody System Dynamics, 9(3) : 283�309, 2003.
//
// =============================================================================

#include <string>

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/fea/ChElementShellANCF_3833.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChIrrApp.h"
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

class ANCFShellTest {
  public:
    ANCFShellTest(int num_elements, SolverType solver_type, int NumThreads, bool useContInt);

    ~ANCFShellTest() { delete m_system; }

    ChSystem* GetSystem() { return m_system; }

    void ExecuteStep() { m_system->DoStepDynamics(1e-3); }

    void SimulateVis();

    ChVector<> GetCornerPointPos() { return m_nodeCornerPoint->GetPos(); }

    void RunTimingTest(ChMatrixNM<double, 4, 19>& timing_stats, const std::string& test_name);

  protected:
    ChSystemSMC* m_system;
    std::shared_ptr<ChNodeFEAxyzDD> m_nodeCornerPoint;
    SolverType m_SolverType;
    int m_NumElements;
    int m_NumThreads;
};

ANCFShellTest::ANCFShellTest(int num_elements, SolverType solver_type, int NumThreads, bool useContInt) {
    m_SolverType = solver_type;
    m_NumElements = 2 * num_elements * num_elements;
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
    double length = 0.6;      // m
    double width = 0.3;       // m
    double thickness = 0.01;  // m
    double rho = 7810;        // kg/m^3
    double E = 1.0e5;         // Pa
    double nu = 0.3;          // Poisson's Ratio

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    m_system->Add(mesh);

    // Setup visualization
    auto mvisualizemesh = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemesh->SetSmoothFaces(true);
    mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshlines = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    mvisualizemeshlines->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshlines->SetWireframe(true);
    mvisualizemeshlines->SetDrawInUndeformedReference(false);
    mesh->AddAsset(mvisualizemeshlines);

    auto mvisualizemeshnode = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    mvisualizemeshnode->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshnode->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshnode->SetSymbolsThickness(0.004);
    mesh->AddAsset(mvisualizemeshnode);

    // Populate the mesh container with a the nodes and elements for the meshed beam
    double dx = length / (4 * num_elements);
    double dy = width / (2 * num_elements);

    // Setup shell normals to initially align with the global z direction with no curvature
    ChVector<> dir1(0, 0, 1);
    ChVector<> Curv1(0, 0, 0);

    // Create a grounded body to connect the 3D pendulum to
    auto grounded = chrono_types::make_shared<ChBody>();
    grounded->SetBodyFixed(true);
    m_system->Add(grounded);

    // Create and add the nodes
    for (auto i = 0; i <= 4 * num_elements; i++) {
        for (auto j = 0; j <= 2 * num_elements; j++) {
            // Fix only the first node's position to ground (Spherical Joint constraint)
            if ((i == 0) && (j == 0)) {
                auto node = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(dx * i, dy * j, 0.0), dir1, Curv1);
                mesh->AddNode(node);

                auto pos_constraint = chrono_types::make_shared<ChLinkPointFrame>();
                pos_constraint->Initialize(node, grounded);  // body to be connected to
                m_system->Add(pos_constraint);
            } else if (((i % 2) == 0) || ((j % 2) == 0)) {
                auto node = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(dx * i, dy * j, 0.0), dir1, Curv1);
                mesh->AddNode(node);
            }
        }
    }

    // Create and add the elements
    for (auto i = 0; i < 2 * num_elements; i++) {
        for (auto j = 0; j < num_elements; j++) {
            int nodeA_idx = 2 * j + i * (3 * num_elements + 2);
            int nodeH_idx = nodeA_idx + 1;
            int nodeD_idx = nodeA_idx + 2;

            int nodeE_idx = j + i * (3 * num_elements + 2) + (2 * num_elements + 1);
            int nodeG_idx = nodeE_idx + 1;

            int nodeB_idx = 2 * j + (i + 1) * (3 * num_elements + 2);
            int nodeF_idx = nodeB_idx + 1;
            int nodeC_idx = nodeB_idx + 2;

            auto element = chrono_types::make_shared<ChElementShellANCF_3833>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeA_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeB_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeC_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeD_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeE_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeF_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeG_idx)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeH_idx)));
            element->SetDimensions(dx, dy);
            element->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, material);
            element->SetAlphaDamp(0.01);

            // By default the "continuous" integration style of calculation method is used since it is typically faster.
            // Switch to the alternative "pre-integration" style of internal force calculation if selected by the user.
            if (!useContInt)
                element->SetIntFrcCalcMethod(ChElementShellANCF_3833::IntFrcMethod::PreInt);

            mesh->AddElement(element);

            m_nodeCornerPoint = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(nodeC_idx));
        }
    }
}

void ANCFShellTest::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    irrlicht::ChIrrApp application(m_system, L"ANCF Shells 3833", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(irr::core::vector3df(-0.4f, 0.4f, 0.4f), irr::core::vector3df(0, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (application.GetDevice()->run()) {
        std::cout << "Time(s): " << this->m_system->GetChTime() << "  Corner Pos(m): " << this->GetCornerPointPos()
                  << std::endl;
        application.BeginScene();
        application.DrawAll();
        irrlicht::tools::drawSegment(application.GetVideoDriver(), ChVector<>(0), ChVector<>(1, 0, 0),
                                     irr::video::SColor(255, 255, 0, 0));
        irrlicht::tools::drawSegment(application.GetVideoDriver(), ChVector<>(0), ChVector<>(0, 1, 0),
                                     irr::video::SColor(255, 0, 255, 0));
        irrlicht::tools::drawSegment(application.GetVideoDriver(), ChVector<>(0), ChVector<>(0, 0, 1),
                                     irr::video::SColor(255, 0, 0, 255));
        ExecuteStep();
        application.EndScene();
    }
#endif
}

void ANCFShellTest::RunTimingTest(ChMatrixNM<double, 4, 19>& timing_stats, const std::string& test_name) {
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

    // Time the requested number of steps, collecting timing information (systems is not restarted between collections)
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
    std::cout << " - Corner_Displacement_End = " << GetCornerPointPos().z() << std::endl;

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
        ANCFShellTest test(2, SolverType::SparseLU, 1, true);
        test.SimulateVis();
#endif
    } else {
        // Run the batch timing studies using the settings below

        // Setup the vector containing the numbers of elements to test
        ChVectorN<int, 1> num_els;
        num_els << 2;  // 2=>2x4=8

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
                        ANCFShellTest test(num_els(i), ls, NumThreads, true);
                        test.RunTimingTest(timing_stats, "ChElementShellANCF_3833_ContInt");
                    }
                    {
                        ANCFShellTest test(num_els(i), ls, NumThreads, false);
                        test.RunTimingTest(timing_stats, "ChElementShellANCF_3833_PreInt");
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
