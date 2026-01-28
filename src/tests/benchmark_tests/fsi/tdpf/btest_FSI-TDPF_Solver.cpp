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
// Benchmark test for comparing solver performance on the VLFP Chrono::FSI-TDPF
// model.
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

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

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
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

// -----------------------------------------------------------------------------
// MBS configuration

// Number of pontoon segments
constexpr int NUM_BODIES = 6;

// Body properties
struct BodyConfig {
    std::string name;
    ChVector3d location;
    double mass;
    ChVector3d inertia_moments;
    std::string mesh_file;
};

// Joint configuration
struct JointConfig {
    std::string name;
    int body1_idx;
    int body2_idx;
    ChVector3d location;
    ChVector3d axis;
};

// RSDA (Rotational Spring-Damper-Actuator) configuration
struct RsdaConfig {
    std::string name;
    int body1_idx;
    int body2_idx;
    ChVector3d location;
    ChVector3d axis;
    double spring_coeff;
    double damping_coeff;
    double free_angle;
};

// Body configurations
const std::array<BodyConfig, NUM_BODIES> BODY_CONFIGS = {{
    {"body1", ChVector3d(14.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_1.obj"},
    {"body2", ChVector3d(43.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_2.obj"},
    {"body3", ChVector3d(72.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_3.obj"},
    {"body4", ChVector3d(101.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_4.obj"},
    {"body5", ChVector3d(130.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_5.obj"},
    {"body6", ChVector3d(159.5, 29.0, 1.6), 1732460.0, ChVector3d(751780112.0, 194629819.0, 929312321.0),
     "pontoon_6.obj"},
}};

// Joint configurations (from vlfp.model.yaml)
const std::array<JointConfig, NUM_BODIES - 1> JOINT_CONFIGS = {{
    {"hinge_1_2", 0, 1, ChVector3d(29.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_2_3", 1, 2, ChVector3d(58.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_3_4", 2, 3, ChVector3d(87.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_4_5", 3, 4, ChVector3d(116.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
    {"hinge_5_6", 4, 5, ChVector3d(145.0, 29.0, 1.6), ChVector3d(0, 1, 0)},
}};

// RSDA configurations (from vlfp.model.yaml)
const std::array<RsdaConfig, NUM_BODIES - 1> RSDA_CONFIGS = {{
    {"damper_1_2", 0, 1, ChVector3d(29.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_2_3", 1, 2, ChVector3d(58.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_3_4", 2, 3, ChVector3d(87.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_4_5", 3, 4, ChVector3d(116.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
    {"damper_5_6", 4, 5, ChVector3d(145.0, 29.0, 1.6), ChVector3d(0, 1, 0), 0.0, 0.0, 0.0},
}};

// Output setup
struct OutputData {
    std::vector<double> time;
    std::array<std::vector<double>, NUM_BODIES> heave;
    std::array<std::vector<double>, NUM_BODIES> surge;
};

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
    double wave_period;

    std::array<std::shared_ptr<ChBody>, NUM_BODIES> bodies;
    std::array<std::shared_ptr<ChLink>, NUM_BODIES - 1> joints;
    std::array<std::shared_ptr<ChLinkRSDA>, NUM_BODIES - 1> rsdas;

    std::unique_ptr<ChFsiSystemTDPF> sysFSI;
    std::unique_ptr<ChFsiFluidSystemTDPF> sysTDPF;
    std::unique_ptr<ChSystem> sysMBS;
};

// -----------------------------------------------------------------------------

template <ChSolver::Type T>
FsiTdpfSolverTest<T>::FsiTdpfSolverTest() : wave_amplitude(2.0), wave_period(10.0) {
    auto vlfp_hydrofile = GetChronoDataFile("fsi-tdpf/vlfp/vlfp_bemio.h5");

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

    // Create pontoon bodies
    ChColormap cmap(ChColormap::Type::KINDLMANN);
    for (int i = 0; i < NUM_BODIES; ++i) {
        const auto& cfg = BODY_CONFIGS[i];

        auto body = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("fsi-tdpf/vlfp/" + cfg.mesh_file),
                                                              1,      // density (not used since we set mass manually)
                                                              false,  // do not compute mass from mesh
                                                              true,   // create visualization asset
                                                              false   // no collision
        );
        body->SetName(cfg.name);
        body->SetPos(cfg.location);
        body->SetMass(cfg.mass);
        body->SetInertiaXX(cfg.inertia_moments);
        body->SetFixed(false);

        body->GetVisualShape(0)->SetColor(cmap.Get((i + 1.0) / NUM_BODIES));

        sysMBS->Add(body);
        bodies[i] = body;
    }

    // Create joints (revolute or fixed based on configuration)
    for (int i = 0; i < NUM_BODIES - 1; ++i) {
        const auto& cfg = JOINT_CONFIGS[i];

        auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
        joint->SetName(cfg.name);
        joint->Initialize(bodies[cfg.body1_idx], bodies[cfg.body2_idx], ChFramed(cfg.location, Q_ROTATE_Y_TO_Z));

        sysMBS->AddLink(joint);
        joints[i] = joint;
    }

    // Create RSDAs
    for (int i = 0; i < NUM_BODIES - 1; ++i) {
        const auto& cfg = RSDA_CONFIGS[i];

        auto rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName(cfg.name);
        rsda->Initialize(bodies[cfg.body1_idx], bodies[cfg.body2_idx], ChFramed(cfg.location, Q_ROTATE_Y_TO_Z));
        rsda->SetSpringCoefficient(cfg.spring_coeff);
        rsda->SetDampingCoefficient(cfg.damping_coeff);
        rsda->SetRestAngle(cfg.free_angle);

        sysMBS->AddLink(rsda);
        rsdas[i] = rsda;
    }

    // ----- TDPF fluid system
    sysTDPF = std::make_unique<ChFsiFluidSystemTDPF>();
    sysTDPF->SetHydroFilename(vlfp_hydrofile);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.num_bodies_ = NUM_BODIES;
    reg_wave_params.regular_wave_amplitude_ = wave_amplitude;
    reg_wave_params.regular_wave_omega_ = CH_2PI / wave_period;
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
    double t_end = 200;
    double render_fps = 30;

#ifdef CHRONO_VSG
    OutputData out_data;
    size_t estimated_steps = static_cast<size_t>(t_end / time_step) + 100;
    out_data.time.reserve(estimated_steps);
    for (int i = 0; i < NUM_BODIES; ++i) {
        out_data.heave[i].reserve(estimated_steps);
        out_data.surge[i].reserve(estimated_steps);
    }

    auto visFSI = chrono_types::make_shared<ChTdpfVisualizationVSG>(sysFSI.get());
    visFSI->SetWaveMeshVisibility(true);
    visFSI->SetWaveMeshParams({87.0, 29.0}, {300.0, 200.0});
    visFSI->SetWaveMeshColormap(ChColormap::Type::BLUE, 0.95f);
    visFSI->SetWaveMeshColorMode(ChTdpfVisualizationVSG::ColorMode::HEIGHT, {-wave_amplitude, +wave_amplitude});
    visFSI->SetWaveMeshUpdateFrequency(render_fps);

    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(sysMBS.get());
    visVSG->SetWindowTitle("FSI-TDPF VLFP regular waves");
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(-45.0, -45.0, 45.0), ChVector3d(80.0, 30.0, 0.0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visVSG->SetModelScale(10);
    visVSG->ToggleCOMFrameVisibility();
    visVSG->ToggleCOMSymbolVisibility();

    visVSG->Initialize();

    double time = 0;
    int render_frame = 0;
    while (time <= t_end) {
        if (time >= render_frame / render_fps) {
            if (!visVSG->Run())
                break;
            visVSG->Render();
            render_frame++;
        }

        ExecuteStep();

        out_data.time.push_back(sysFSI->GetSimTime());
        for (int i = 0; i < NUM_BODIES; ++i) {
            out_data.heave[i].push_back(bodies[i]->GetPos().z());
            out_data.surge[i].push_back(bodies[i]->GetPos().x());
        }

        time += time_step;
    }

    #ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot;
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("heave (m)");
    gplot.SetTitle("VLFP regular waves");
    for (int i = 0; i < NUM_BODIES; i++)
        gplot.Plot(out_data.time, out_data.heave[i], bodies[i]->GetName(), " with lines lw 2");
    #endif

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
