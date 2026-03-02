// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Benchmark test for comparing solver performance on a Chrono::FSI-TDPF model.
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/ChFsiBenchmark.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
    #include "chrono_mumps/ChSolverMumps.h"
#endif

#ifdef CHRONO_VSG
    #include "chrono_fsi/tdpf/visualization/ChTdpfVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::tdpf;

// -----------------------------------------------------------------------------
// Benchmark test parameters

// Number of steps for hot start (1s if time_step = 1e-2)
constexpr int NUM_SKIP_STEPS = 100;

// Number of simulation steps for each benchmark (10s if time_step = 1e-2)
constexpr int NUM_SIM_STEPS = 1000;

// Number of test repetitions to extract statistics
constexpr int REPEATS = 4;

// -----------------------------------------------------------------------------
// Simulation parameters

double time_step = 1.0e-2;
int solver_max_iterations = 100;
bool solver_use_diag_precond = true;

double wave_height = 1.0;
double wave_period = 10;

// -----------------------------------------------------------------------------

template <ChSolver::Type T>
class FsiTdpfSolverTest : public chrono::fsi::ChFsiBenchmarkTest {
  public:
    FsiTdpfSolverTest();
    ~FsiTdpfSolverTest() = default;

    ChFsiSystem* GetFsiSystem() override { return sysFSI.get(); }
    void ExecuteStep() override;

    void SimulateVis();

  private:
    double wave_amplitude;

    std::vector<std::shared_ptr<ChBody>> bodies;

    std::unique_ptr<ChFsiSystemTDPF> sysFSI;
    std::unique_ptr<ChFsiFluidSystemTDPF> sysTDPF;
    std::unique_ptr<ChSystem> sysMBS;
};

// -----------------------------------------------------------------------------

template <ChSolver::Type T>
FsiTdpfSolverTest<T>::FsiTdpfSolverTest() : wave_amplitude(0.5 * wave_height) {
    auto hydro_file = GetChronoDataFile("fsi-tdpf/cparray/cparray.h5");

    ChVector3d g_acc(0.0, 0.0, -9.81);

    // ----- Multibody system
    sysMBS = std::make_unique<ChSystemSMC>();

    switch (T) {
        case ChSolver::Type::APGD:
        case ChSolver::Type::BARZILAIBORWEIN:
        case ChSolver::Type::GMRES: {
            sysMBS->SetSolverType(T);
            auto iterative = sysMBS->GetSolver()->AsIterative();
            iterative->SetMaxIterations(solver_max_iterations);
            iterative->EnableDiagonalPreconditioner(solver_use_diag_precond);
            break;
        }
        case ChSolver::Type::SPARSE_LU:
        case ChSolver::Type::SPARSE_QR: {
            sysMBS->SetSolverType(T);
            auto direct = sysMBS->GetSolver()->AsDirect();
            direct->LockSparsityPattern(true);
            direct->UseSparsityPatternLearner(false);
            break;
        }
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<chrono::ChSolverPardisoMKL>(1);
            solver->LockSparsityPattern(true);
            solver->UseSparsityPatternLearner(false);
            sysMBS->SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<chrono::ChSolverMumps>();
            solver->LockSparsityPattern(true);
            solver->UseSparsityPatternLearner(false);
            solver->EnableNullPivotDetection(true);
            solver->GetMumpsEngine().SetICNTL(14, 50);
            sysMBS->SetSolver(solver);
#endif
            break;
        }
    }

    // Create bodies
    double D = 3;
    ChColormap cmap(ChColormap::Type::KINDLMANN);

    for (int ix = -1; ix <= +1; ix++) {
        for (int iy = -1; iy <= +1; iy++) {
            int b = (ix + 1) * 3 + (iy + 1);

            auto body = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, 1.0, 0.5, 1500.0);
            body->SetName("body" + std::to_string(b + 1));
            body->SetPos(ChVector3d(ix * D, iy * D, 0.0));
            body->SetFixed(false);
            body->GetVisualShape(0)->SetColor(cmap.Get((b + 1.0) / 9));
            sysMBS->Add(body);

            bodies.push_back(body);
        }
    }

    // Create joints (spherical)
    for (int ix = -1; ix <= +1; ix++) {
        for (int iy = -1; iy <= +1; iy++) {
            int b = (ix + 1) * 3 + (iy + 1);
            int b1 = (ix + 2) * 3 + (iy + 1);
            int b2 = (ix + 1) * 3 + (iy + 2);
            
            const auto& body = bodies[b];
            
            if (ix < +1) {
                const auto& body1 = bodies[b1];

                auto joint = chrono_types::make_shared<ChLinkDistance>();
                joint->SetName("joint_" + std::to_string(b) + "_" + std::to_string(b1));
                joint->Initialize(body, body1, true, VNULL, VNULL, false, D);
                joint->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
                sysMBS->AddLink(joint);
            
                auto rsda = chrono_types::make_shared<ChLinkRSDA>();
                rsda->Initialize(body, body1, ChFramed(0.5*(body->GetPos() + body1->GetPos()), Q_ROTATE_Z_TO_Y));
                rsda->SetDampingCoefficient(1e4);
                sysMBS->AddLink(rsda);
            }

            if (iy < +1) {
                auto body2 = bodies[b2];
                
                auto joint = chrono_types::make_shared<ChLinkDistance>();
                joint->SetName("joint_" + std::to_string(b) + "_" + std::to_string(b2));
                joint->Initialize(body, body2, true, VNULL, VNULL, false, D);
                joint->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
                sysMBS->AddLink(joint);
            
                auto rsda = chrono_types::make_shared<ChLinkRSDA>();
                rsda->Initialize(body, body2, ChFramed(0.5 * (body->GetPos() + body2->GetPos()), Q_ROTATE_Z_TO_X));
                rsda->SetDampingCoefficient(1e4);
                sysMBS->AddLink(rsda);            
            }
        }
    }

    // ----- TDPF fluid system
    sysTDPF = std::make_unique<ChFsiFluidSystemTDPF>();
    sysTDPF->SetHydroFilename(hydro_file);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.regular_wave_amplitude = wave_amplitude;
    reg_wave_params.regular_wave_omega = CH_2PI / wave_period;
    sysTDPF->AddWaves(reg_wave_params);

    // ----- FSI system
    sysFSI = std::make_unique<ChFsiSystemTDPF>(sysMBS.get(), sysTDPF.get());
    sysFSI->SetGravitationalAcceleration(g_acc);
    sysFSI->SetVerbose(false);
    sysFSI->SetStepSizeCFD(time_step);
    sysFSI->SetStepsizeMBD(time_step);

    // Add FSI bodies
    for (auto& body : bodies)
        sysFSI->AddFsiBody(body, nullptr, false);

    sysFSI->Initialize();
}

template <ChSolver::Type T>
void FsiTdpfSolverTest<T>::ExecuteStep() {
    sysFSI->DoStepDynamics(time_step);
}

template <ChSolver::Type T>
void FsiTdpfSolverTest<T>::SimulateVis() {
    double render_fps = 30;

#ifdef CHRONO_VSG
    auto visFSI = chrono_types::make_shared<ChTdpfVisualizationVSG>(sysFSI.get());
    visFSI->SetWaveMeshVisibility(true);
    visFSI->SetWaveMeshParams({0.0, 0.0}, {40.0, 10.0});
    visFSI->SetWaveMeshColormap(ChColormap::Type::BLUE, 0.95f);
    visFSI->SetWaveMeshColorMode(ChTdpfVisualizationVSG::ColorMode::HEIGHT, {-wave_amplitude, +wave_amplitude});
    visFSI->SetWaveMeshUpdateFrequency(render_fps);

    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(sysMBS.get());
    visVSG->SetWindowTitle("FSI-TDPF VLFP regular waves");
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(-12, -12, 4));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visVSG->SetModelScale(1);
    visVSG->ToggleAbsFrameVisibility();
    visVSG->ToggleCOMSymbolVisibility();

    visVSG->Initialize();

    double time = 0;
    int render_frame = 0;
    ChRealtimeStepTimer rt_timer;
    while (true) {
        if (time >= render_frame / render_fps) {
            if (!visVSG->Run())
                break;
            visVSG->Render();
            render_frame++;
        }

        ExecuteStep();

        time += time_step;
        rt_timer.Spin(time_step);
    }
#endif
}

// -----------------------------------------------------------------------------

CH_FSI_BM_SIMULATION_ONCE(BARZILAI_BORWEIN,
                          FsiTdpfSolverTest<ChSolver::Type::BARZILAIBORWEIN>,
                          NUM_SKIP_STEPS,
                          NUM_SIM_STEPS,
                          REPEATS);
////CH_FSI_BM_SIMULATION_ONCE(APGD, FsiTdpfSolverTest<ChSolver::Type::APGD>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_FSI_BM_SIMULATION_ONCE(GMRES, FsiTdpfSolverTest<ChSolver::Type::GMRES>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);
CH_FSI_BM_SIMULATION_ONCE(SPARSE_LU,
                          FsiTdpfSolverTest<ChSolver::Type::SPARSE_LU>,
                          NUM_SKIP_STEPS,
                          NUM_SIM_STEPS,
                          REPEATS);
CH_FSI_BM_SIMULATION_ONCE(SPARSE_QR,
                          FsiTdpfSolverTest<ChSolver::Type::SPARSE_QR>,
                          NUM_SKIP_STEPS,
                          NUM_SIM_STEPS,
                          REPEATS);

#ifdef CHRONO_PARDISO_MKL
CH_FSI_BM_SIMULATION_ONCE(PARDISO_MKL,
                          FsiTdpfSolverTest<ChSolver::Type::PARDISO_MKL>,
                          NUM_SKIP_STEPS,
                          NUM_SIM_STEPS,
                          REPEATS);

#endif

#ifdef CHRONO_MUMPS
CH_FSI_BM_SIMULATION_ONCE(MUMPS, FsiTdpfSolverTest<ChSolver::Type::MUMPS>, NUM_SKIP_STEPS, NUM_SIM_STEPS, REPEATS);

#endif

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Force tabular output for user counters
    std::string extra_arg = "--benchmark_counters_tabular=true";

    std::vector<char*> new_argv(argv, argv + argc);
    new_argv.push_back(extra_arg.data());
    int new_argc = argc + 1;

    // Initialize benchmark framework
    ::benchmark::Initialize(&new_argc, new_argv.data());

#ifdef CHRONO_VSG
    // Pass a dummy argument to run a single simulation with visualization
    if (::benchmark::ReportUnrecognizedArguments(new_argc, new_argv.data())) {
        FsiTdpfSolverTest<ChSolver::Type::GMRES> solver_test;
        solver_test.SimulateVis();
        return 0;
    }
#endif

    // Run all tests
    ::benchmark::RunSpecifiedBenchmarks();
}
