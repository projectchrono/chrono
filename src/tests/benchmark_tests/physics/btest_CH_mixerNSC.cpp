// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
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
// Benchmark test for contact simulation using NSC contact.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChBenchmark.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

using namespace chrono;

// =============================================================================

template <int N>
class MixerTestNSC : public utils::ChBenchmarkTest {
  public:
    MixerTestNSC();
    ~MixerTestNSC() { delete m_system; }

    ChSystem* GetSystem() override { return m_system; }
    void ExecuteStep() override { m_system->DoStepDynamics(m_step); }

    void SimulateVis();

  private:
    ChSystemNSC* m_system;
    double m_step;
};

template <int N>
MixerTestNSC<N>::MixerTestNSC() : m_system(new ChSystemNSC()), m_step(0.02) {
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    for (int bi = 0; bi < N; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(1.0, 1000, true, true, mat);
        sphereBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        m_system->Add(sphereBody);

        auto boxBody = chrono_types::make_shared<ChBodyEasyBox>(1.25, 1.25, 1.25, 1000, true, true, mat);
        boxBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        m_system->Add(boxBody);

        auto cylBody =
            chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, 0.8, 1.0, 1000, true, true, mat);
        cylBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        m_system->Add(cylBody);
    }

    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true, mat);
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);
    m_system->Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, true, true, mat);
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);
    m_system->Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, true, true, mat);
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);
    m_system->Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, true, true, mat);
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);
    m_system->Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, true, true, mat);
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);
    m_system->Add(wallBody4);

    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1, 4000, true, true, mat);
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    m_system->Add(rotatingBody);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto fun = chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 3.0);
    motor->SetSpeedFunction(fun);
    m_system->AddLink(motor);
}

template <int N>
void MixerTestNSC<N>::SimulateVis() {
#ifdef CHRONO_IRRLICHT
    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
    vis->AttachSystem(m_system);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Rigid contacts");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 14, -20), ChVector<>(0, 0, 0));

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        ExecuteStep();
        vis->EndScene();
    }
#endif
}

// =============================================================================

#define NUM_SKIP_STEPS 2000  // number of steps for hot start
#define NUM_SIM_STEPS 1000   // number of simulation steps for each benchmark

CH_BM_SIMULATION_LOOP(MixerNSC032, MixerTestNSC<32>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);
CH_BM_SIMULATION_LOOP(MixerNSC064, MixerTestNSC<64>,  NUM_SKIP_STEPS, NUM_SIM_STEPS, 10);

// =============================================================================

int main(int argc, char* argv[]) {
    ::benchmark::Initialize(&argc, argv);

#ifdef CHRONO_IRRLICHT
    if (::benchmark::ReportUnrecognizedArguments(argc, argv)) {
        MixerTestNSC<64> test;
        test.SimulateVis();
        return 0;
    }
#endif

    ::benchmark::RunSpecifiedBenchmarks();
}
