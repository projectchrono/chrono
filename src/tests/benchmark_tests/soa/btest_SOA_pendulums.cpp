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
#include "chrono/solver/ChSolverBB.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChSoaRevoluteBody.h"

#include "chrono/utils/ChBenchmark.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

using namespace chrono;
using namespace chrono::soa;

using std::cout;
using std::endl;

// =============================================================================

#define KINEMATICS

// =============================================================================

template <int N>
class ChainTest : public utils::ChBenchmarkTest {
  public:
    ChainTest();
    ~ChainTest() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
#ifdef KINEMATICS
    void ExecuteStep() override { m_system->DoStepKinematics(m_step); }
#else
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }
#endif

    void SimulateVis();

  private:
    ChSystemNSC* m_system;
    double m_length;
    double m_step;
};

template <int N>
ChainTest<N>::ChainTest() : m_length(1.0), m_step(1e-3) {
    // Create system
    m_system = new ChSystemNSC;
    m_system->SetGravitationalAcceleration(ChVector3d(0, -1, 0));

    // Set solver parameters
    auto solver = chrono_types::make_shared<ChSolverBB>();
    solver->SetMaxIterations(50);
    m_system->SetSolver(solver);

    /*
    // Use HHT integrator
    m_system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);
    integrator->SetStepControl(false);
    integrator->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
    integrator->SetVerbose(false);
    */

    // Create ground
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    m_system->AddBody(ground);

    // Create pendulums
    double width = 0.1;
    double density = 1000;
    for (int ib = 0; ib < N; ib++) {
        auto prev = m_system->GetBodies().back();

        auto pend = chrono_types::make_shared<ChBodyEasyBox>(m_length, width, width, density, true, false);
        pend->SetPos(ChVector3d((ib + 0.5) * m_length, 0, 0));
        m_system->AddBody(pend);

        auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
        rev->Initialize(pend, prev, ChFrame<>(ChVector3d(ib * m_length, 0, 0)));
        m_system->AddLink(rev);
    }
}

template <int N>
void ChainTest<N>::SimulateVis() {
#ifdef CHRONO_VSG
    double offset = N * m_length;

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(m_system);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(ChVector2i(1200, 800));
    vis->SetWindowPosition(ChVector2i(100, 300));
    vis->SetWindowTitle("Pendulum chain");
    vis->AddCamera(ChVector3d(2 * offset, -offset, offset / 2));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();

    ChRealtimeStepTimer rt_timer;
    while (vis->Run()) {
        vis->Render();
        ExecuteStep();
        rt_timer.Spin(m_step);
    }
#endif
}

// =============================================================================

template <int N>
class SOAChainTest : public utils::ChBenchmarkTest {
  public:
    SOAChainTest();
    ~SOAChainTest() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
#ifdef KINEMATICS
    void ExecuteStep() override { m_soa->DoForwardKinematics(); }
#else
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }
#endif

    void SimulateVis();

    double GetStep() const { return m_step; }
    std::shared_ptr<ChSoaAssembly> GetSOA() const { return m_soa; }
    const std::vector<std::shared_ptr<ChSoaRevoluteBody>>& GetPendulums() const { return m_pendulums; }

  private:
    ChSystemNSC* m_system;
    std::shared_ptr<ChSoaAssembly> m_soa;
    std::vector<std::shared_ptr<ChSoaRevoluteBody>> m_pendulums;
    double m_length;
    double m_step;
};

template <int N>
SOAChainTest<N>::SOAChainTest() : m_length(1.0), m_step(1e-3) {
    // Create system
    m_system = new ChSystemNSC;
    m_system->SetGravitationalAcceleration(ChVector3d(0, -1, 0));

    // Set solver parameters
    auto solver = chrono_types::make_shared<ChSolverBB>();
    solver->SetMaxIterations(50);
    m_system->SetSolver(solver);

    /*
    // Use HHT integrator
    m_system->SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxIters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);
    integrator->SetStepControl(false);
    integrator->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
    integrator->SetVerbose(false);
    */

    // Create SOA assembly and pendulums
    m_soa = chrono_types::make_shared<ChSoaAssembly>();

    double width = 0.1;
    double mass = 10;
    ChMatrix33d inertia(ChVector3d(0.02, 0.84, 0.84));
    ChSoaMassProperties mprops(mass, ChVector3d(m_length / 2, 0, 0), inertia);

    for (int i = 0; i < N; i++) {
        std::shared_ptr<ChSoaMobilizedBody> parent;
        if (i == 0)
            parent = m_soa->getGroundBody();
        else
            parent = m_pendulums[i - 1];

        auto pendulum = chrono_types::make_shared<ChSoaRevoluteBody>(parent, mprops,                                         //
                                                                     ChFramed(ChVector3d(m_length, 0, 0), Q_ROTATE_Z_TO_Y),  // X_PF
                                                                     ChFramed(VNULL, Q_ROTATE_Z_TO_Y),                       // X_BM
                                                                     "pendulum_" + std::to_string(i));                       //

        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(m_length, width, width);
        vis_shape->SetColor(ChColor(0, 0, 0.6f));
        pendulum->AddVisualShape(vis_shape, ChFramed(ChVector3d(m_length / 2, 0, 0), QUNIT));

        pendulum->setRelPos(0.0);
        pendulum->setRelVel(0.5);
        m_soa->AddBody(pendulum);

        m_pendulums.push_back(pendulum);
    }

    // Attach SOA assembly to Chrono system
    m_system->Add(m_soa);

    // Initialize assembly (perform position- and velocity-level traversal)
    m_soa->Initialize();
}

template <int N>
void SOAChainTest<N>::SimulateVis() {
#ifdef CHRONO_VSG
    double offset = N * m_length;

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(m_system);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowSize(ChVector2i(1200, 800));
    vis->SetWindowPosition(ChVector2i(100, 300));
    vis->SetWindowTitle("SOA pendulum chain");
    vis->AddCamera(ChVector3d(2 * offset, -offset, offset / 2));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();

    ChFunctionSine dof(CH_PI, 1 / 4.0, 0.0);
    ChRealtimeStepTimer rt_timer;
    double time = 0;
    while (vis->Run()) {
        vis->Render();
    #ifdef KINEMATICS
        for (auto& p : m_pendulums) {
            p->setRelPos(dof(time));
        }
    #endif
        ExecuteStep();
        rt_timer.Spin(m_step);
        time += m_step;
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(Chain04, ChainTest<4>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain08, ChainTest<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain16, ChainTest<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain32, ChainTest<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain64, ChainTest<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(Chain128, ChainTest<128>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);

CH_BM_SIMULATION_LOOP(SOAChain04, SOAChainTest<4>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(SOAChain08, SOAChainTest<8>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(SOAChain16, SOAChainTest<16>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(SOAChain32, SOAChainTest<32>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(SOAChain64, SOAChainTest<64>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);
CH_BM_SIMULATION_LOOP(SOAChain128, SOAChainTest<128>, NUM_SKIP_STEPS, NUM_SIM_STEPS, 20);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_VSG
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        int which = 1;
        cout << "Options:" << endl;
        cout << "  1. Chain" << endl;
        cout << "  2. SOA Chain" << endl;
        std::cout << "\nSelect formulation: ";
        std::cin >> which;
        std::cout << std::endl;
        ChClampValue(which, 1, 2);

        switch (which) {
            case 1: {
                ChainTest<4> test;
                test.SimulateVis();
                break;
            }
            case 2: {
                SOAChainTest<4> test;
                test.SimulateVis();
                break;
            }
        }

        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
