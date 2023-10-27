// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// Benchmark test for pendulum chain.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

using namespace chrono;

// =============================================================================

template <int N>
class ChainTest : public utils::ChBenchmarkTest {
  public:
    ChainTest();
    ~ChainTest() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis();

  private:
    ChSystem* m_system;
    double m_length;
    double m_step;
};

template <int N>
ChainTest<N>::ChainTest() : m_length(0.25), m_step(1e-3) {
    ChTimestepper::Type integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;

    // Create system
    m_system = new ChSystemNSC;
    m_system->Set_G_acc(ChVector<>(0, -1, 0));

    // Set solver parameters
    switch (solver_type) {
        case ChSolver::Type::PSOR: {
            auto solver = chrono_types::make_shared<ChSolverPSOR>();
            solver->SetMaxIterations(50);
            solver->SetOmega(0.8);
            solver->SetSharpnessLambda(1.0);
            m_system->SetSolver(solver);

            m_system->SetMaxPenetrationRecoverySpeed(1.5);
            m_system->SetMinBounceSpeed(2.0);

            break;
        }
        case ChSolver::Type::BARZILAIBORWEIN: {
            auto solver = chrono_types::make_shared<ChSolverBB>();
            solver->SetMaxIterations(50);
            m_system->SetSolver(solver);

            m_system->SetMaxPenetrationRecoverySpeed(1.5);
            m_system->SetMinBounceSpeed(2.0);

            break;
        }
        default:
            break;
    }

    // Set integrator parameters
    switch (integrator_type) {
        case ChTimestepper::Type::HHT: {
            m_system->SetTimestepperType(ChTimestepper::Type::HHT);
            auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
            integrator->SetAlpha(-0.2);
            integrator->SetMaxiters(50);
            integrator->SetAbsTolerances(1e-4, 1e2);
            integrator->SetStepControl(false);
            integrator->SetModifiedNewton(false);
            integrator->SetVerbose(false);
            break;
        }
        default:
            break;
    }

    // Create ground
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    m_system->AddBody(ground);

    // Create pendulums
    double width = 0.025;
    double density = 500;
    for (int ib = 0; ib < N; ib++) {
        auto prev = m_system->Get_bodylist().back();

        auto pend = chrono_types::make_shared<ChBodyEasyBox>(m_length, width, width, density, true, false);
        pend->SetPos(ChVector<>((ib + 0.5) * m_length, 0, 0));
        m_system->AddBody(pend);

        auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
        rev->Initialize(pend, prev, ChCoordsys<>(ChVector<>(ib * m_length, 0, 0)));
        m_system->AddLink(rev);
    }
}

template <int N>
void ChainTest<N>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    double offset = N * m_length;

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->AttachSystem(m_system);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Pendulum chain");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, -offset / 2, offset), ChVector<>(0, -offset / 2, 0));

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        m_system->DoStepDynamics(m_step);
        vis->EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(Chain04, ChainTest<4>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain08, ChainTest<8>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain16, ChainTest<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain32, ChainTest<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain64, ChainTest<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        ChainTest<4> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
