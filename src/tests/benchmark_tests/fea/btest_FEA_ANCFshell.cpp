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
// Authors: Radu Serban
// =============================================================================
//
// Benchmark test for ANCF shell elements.
//
// Note that the MKL Pardiso and Mumps solvers are set to lock the sparsity
// pattern, but not to use the sparsity pattern learner.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
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

enum class SolverType {MINRES, MKL, MUMPS, PARDISO_PROJECT, SparseQR};

template <int N>
class ANCFshell : public utils::ChBenchmarkTest {
  public:
    virtual ~ANCFshell() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(1e-4); }

    void SimulateVis();

  protected:
    ANCFshell(SolverType solver_type);

    ChSystemSMC* m_system;
};

template <int N>
class ANCFshell_MINRES : public ANCFshell<N> {
  public:
    ANCFshell_MINRES() : ANCFshell<N>(SolverType::MINRES) {}
};

template <int N>
class ANCFshell_SparseQR : public ANCFshell<N> {
  public:
    ANCFshell_SparseQR() : ANCFshell<N>(SolverType::SparseQR) {}
};

template <int N>
class ANCFshell_MKL : public ANCFshell<N> {
  public:
    ANCFshell_MKL() : ANCFshell<N>(SolverType::MKL) {}
};

template <int N>
class ANCFshell_MUMPS : public ANCFshell<N> {
  public:
    ANCFshell_MUMPS() : ANCFshell<N>(SolverType::MUMPS) {}
};

template <int N>
class ANCFshell_PARDISOPROJECT : public ANCFshell<N> {
  public:
    ANCFshell_PARDISOPROJECT() : ANCFshell<N>(SolverType::PARDISO_PROJECT) {}
};

template <int N>
ANCFshell<N>::ANCFshell(SolverType solver_type) {
    m_system = new ChSystemSMC();
    m_system->Set_G_acc(ChVector<>(0, -9.8, 0));
    m_system->SetNumThreads(4);

    // Set solver parameters
#ifndef CHRONO_PARDISO_MKL
    if (solver_type == SolverType::MKL) {
        solver_type = SolverType::MINRES;
        std::cout << "WARNING! Chrono::MKL not enabled. Forcing use of MINRES solver" << std::endl;
    }
#endif

#ifndef CHRONO_MUMPS
    if (solver_type == SolverType::MUMPS) {
        solver_type = SolverType::MINRES;
        std::cout << "WARNING! Chrono::MUMPS not enabled. Forcing use of MINRES solver" << std::endl;
    }
#endif

#ifndef CHRONO_PARDISOPROJECT
    if (solver_type == SolverType::PARDISO_PROJECT) {
        solver_type = SolverType::MINRES;
        std::cout << "WARNING! Chrono::PARDISO_PROJECT not enabled. Forcing use of MINRES solver" << std::endl;
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
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>(4);
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<ChSolverMumps>(4);
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::PARDISO_PROJECT: {
#ifdef CHRONO_PARDISOPROJECT
            auto solver = chrono_types::make_shared<ChSolverPardisoProject>();
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::SparseQR: {
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
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
    integrator->SetMode(ChTimestepperHHT::ACCELERATION);
    integrator->SetScaling(true);
    integrator->SetVerbose(false);

    // Mesh properties
    double length = 1;
    double width = 0.1;
    double thickness = 0.01;

    double rho = 500;
    ChVector<> E(2.1e7, 2.1e7, 2.1e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create mesh nodes and elements
    auto mesh = chrono_types::make_shared<ChMesh>();
    m_system->Add(mesh);

    auto vis_surf = chrono_types::make_shared<ChVisualizationFEAmesh>(*mesh);
    vis_surf->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    vis_surf->SetWireframe(true);
    vis_surf->SetDrawInUndeformedReference(true);
    mesh->AddAsset(vis_surf);

    auto vis_node = chrono_types::make_shared<ChVisualizationFEAmesh>(*mesh);
    vis_node->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    vis_node->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    vis_node->SetSymbolsThickness(0.004);
    mesh->AddAsset(vis_node);

    double dx = length / N;
    ChVector<> dir(0, 1, 0);

    auto nodeA = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, -width / 2), dir);
    auto nodeB = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, +width / 2), dir);
    nodeA->SetFixed(true);
    nodeB->SetFixed(true);
    mesh->AddNode(nodeA);
    mesh->AddNode(nodeB);

    for (int i = 1; i < N; i++) {
        auto nodeC = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(i * dx, 0, -width / 2), dir);
        auto nodeD = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(i * dx, 0, +width / 2), dir);
        mesh->AddNode(nodeC);
        mesh->AddNode(nodeD);

        auto element = chrono_types::make_shared<ChElementShellANCF>();
        element->SetNodes(nodeA, nodeB, nodeD, nodeC);
        element->SetDimensions(dx, width);
        element->AddLayer(thickness, 0 * CH_C_DEG_TO_RAD, mat);
        element->SetAlphaDamp(0.0);
        element->SetGravityOn(false);
        mesh->AddElement(element);

        nodeA = nodeC;
        nodeB = nodeD;
    }
}

template <int N>
void ANCFshell<N>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    irrlicht::ChIrrApp application(m_system, L"ANCF shells", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(-0.2f, 0.2f, 0.2f), irr::core::vector3df(0, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    while (application.GetDevice()->run()) {
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

// =============================================================================

#define NUM_SKIP_STEPS 10  // number of steps for hot start
#define NUM_SIM_STEPS 100  // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(ANCFshell08_MINRES, ANCFshell_MINRES<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_MINRES, ANCFshell_MINRES<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_MINRES, ANCFshell_MINRES<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_MINRES, ANCFshell_MINRES<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

CH_BM_SIMULATION_LOOP(ANCFshell08_SparseQR, ANCFshell_SparseQR<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_SparseQR, ANCFshell_SparseQR<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_SparseQR, ANCFshell_SparseQR<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_SparseQR, ANCFshell_SparseQR<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

#ifdef CHRONO_PARDISO_MKL
CH_BM_SIMULATION_LOOP(ANCFshell08_MKL, ANCFshell_MKL<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_MKL, ANCFshell_MKL<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_MKL, ANCFshell_MKL<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_MKL, ANCFshell_MKL<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
#endif

#ifdef CHRONO_MUMPS
CH_BM_SIMULATION_LOOP(ANCFshell08_MUMPS, ANCFshell_MUMPS<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_MUMPS, ANCFshell_MUMPS<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_MUMPS, ANCFshell_MUMPS<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_MUMPS, ANCFshell_MUMPS<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
#endif

#ifdef CHRONO_PARDISOPROJECT
CH_BM_SIMULATION_LOOP(ANCFshell08_PARDISOPROJECT, ANCFshell_PARDISOPROJECT<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell16_PARDISOPROJECT, ANCFshell_PARDISOPROJECT<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell32_PARDISOPROJECT, ANCFshell_PARDISOPROJECT<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(ANCFshell64_PARDISOPROJECT, ANCFshell_PARDISOPROJECT<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
#endif

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        ANCFshell_MINRES<16> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
