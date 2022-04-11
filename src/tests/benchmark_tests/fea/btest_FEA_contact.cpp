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
// Benchmark test for FEA contacts.
//
// Note that the MKL Pardiso and Mumps solvers are set to lock the sparsity
// pattern, but not to use the sparsity pattern learner.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
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

using namespace chrono;
using namespace chrono::fea;

enum class SolverType { MINRES, MKL, MUMPS };

class FEAcontactTest : public utils::ChBenchmarkTest {
  public:
    virtual ~FEAcontactTest() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(1e-3); }

    void SimulateVis();

  protected:
    FEAcontactTest(SolverType solver_type);

  private:
    void CreateFloor(std::shared_ptr<ChMaterialSurfaceSMC> cmat);
    void CreateBeams(std::shared_ptr<ChMaterialSurfaceSMC> cmat);
    void CreateCables(std::shared_ptr<ChMaterialSurfaceSMC> cmat);

    ChSystemSMC* m_system;
};

class FEAcontactTest_MINRES : public FEAcontactTest {
  public:
    FEAcontactTest_MINRES() : FEAcontactTest(SolverType::MINRES) {}
};

class FEAcontactTest_MKL : public FEAcontactTest {
  public:
    FEAcontactTest_MKL() : FEAcontactTest(SolverType::MKL) {}
};

class FEAcontactTest_MUMPS : public FEAcontactTest {
  public:
    FEAcontactTest_MUMPS() : FEAcontactTest(SolverType::MUMPS) {}
};

FEAcontactTest::FEAcontactTest(SolverType solver_type) {
    m_system = new ChSystemSMC();

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

    switch (solver_type) {
        case SolverType::MINRES: {
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            m_system->SetSolver(solver);
            solver->SetMaxIterations(40);
            solver->SetTolerance(1e-10);
            solver->EnableDiagonalPreconditioner(true);
            solver->SetVerbose(false);

            m_system->SetSolverForceTolerance(1e-10);
            
            m_system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        }
        case SolverType::MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case SolverType::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<ChSolverMumps>();
            solver->UseSparsityPatternLearner(false);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            m_system->SetSolver(solver);
#endif
            break;
        }
    }

    collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.006);

    auto cmat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    cmat->SetYoungModulus(6e4);
    cmat->SetFriction(0.3f);
    cmat->SetRestitution(0.2f);
    cmat->SetAdhesion(0);

    CreateFloor(cmat);
    CreateBeams(cmat);
    CreateCables(cmat);
}

void FEAcontactTest::CreateFloor(std::shared_ptr<ChMaterialSurfaceSMC> cmat) {
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true, true, cmat);
    mfloor->SetBodyFixed(true);
    mfloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    m_system->Add(mfloor);
}

void FEAcontactTest::CreateBeams(std::shared_ptr<ChMaterialSurfaceSMC> cmat) {
    auto mesh = chrono_types::make_shared<ChMesh>();
    m_system->Add(mesh);

    auto emat = chrono_types::make_shared<ChContinuumElastic>();
    emat->Set_E(0.01e9);
    emat->Set_v(0.3);
    emat->Set_RayleighDampingK(0.003);
    emat->Set_density(1000);

    double angles[4] = {0.15, 0.3, 0.0, 0.7};

    for (int i = 0; i < 4; ++i) {
        ChCoordsys<> cdown(ChVector<>(0, -0.4, 0));
        ChCoordsys<> crot(VNULL, Q_from_AngAxis(CH_C_2PI * angles[i], VECT_Y) * Q_from_AngAxis(CH_C_PI_2, VECT_X));
        ChCoordsys<> cydisp(ChVector<>(0.0, 0.1 + i * 0.1, -0.3));
        ChCoordsys<> ctot = cdown >> crot >> cydisp;
        ChMeshFileLoader::FromTetGenFile(mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                         GetChronoDataFile("fea/beam.ele").c_str(), emat, ctot.pos,
                                         ChMatrix33<>(ctot.rot));
    }

    auto surf = chrono_types::make_shared<ChContactSurfaceMesh>(cmat);
    mesh->AddContactSurface(surf);
    surf->AddFacesFromBoundary(0.002);

    auto vis_speed = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_speed->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_speed->SetColorscaleMinMax(0.0, 5.50);
    vis_speed->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(vis_speed);
}

void FEAcontactTest::CreateCables(std::shared_ptr<ChMaterialSurfaceSMC> cmat) {
    auto mesh = chrono_types::make_shared<ChMesh>();
    m_system->Add(mesh);

    auto section = chrono_types::make_shared<ChBeamSectionCable>();
    section->SetDiameter(0.05);
    section->SetYoungModulus(0.01e9);
    section->SetBeamRaleyghDamping(0.05);

    ChBuilderCableANCF builder;

    builder.BuildBeam(mesh, section, 10, ChVector<>(0, 0.1, -0.5), ChVector<>(0.5, 0.5, -0.5));

    auto cloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(cmat);
    mesh->AddContactSurface(cloud);
    cloud->AddAllNodes(0.025);

    auto vis_speed = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_speed->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    vis_speed->SetColorscaleMinMax(0.0, 5.50);
    vis_speed->SetSmoothFaces(true);
    vis_speed->SetWireframe(true);
    mesh->AddVisualShapeFEA(vis_speed);

    auto vis_nodes = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_nodes->SetSymbolsThickness(0.008);
    mesh->AddVisualShapeFEA(vis_nodes);
}

void FEAcontactTest::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    m_system->SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("FEA contacts");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 0.6, -1.0), ChVector<>(0, 0, 0));

    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        ExecuteStep();
        vis->EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 50  // number of steps for hot start
#define NUM_SIM_STEPS 500  // number of simulation steps for each benchmark

CH_BM_SIMULATION_ONCE(FEAcontact_MINRES, FEAcontactTest_MINRES, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

#ifdef CHRONO_PARDISO_MKL
CH_BM_SIMULATION_ONCE(FEAcontact_MKL, FEAcontactTest_MKL, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
#endif

#ifdef CHRONO_MUMPS
CH_BM_SIMULATION_ONCE(FEAcontact_MUMPS, FEAcontactTest_MUMPS, NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
#endif

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        FEAcontactTest_MINRES test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
