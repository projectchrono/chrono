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
// Author: Radu Serban, Dave Ogden
// =============================================================================

#include <iomanip>

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"

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

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::tdpf;

// -----------------------------------------------------------------------------

class FloatingPlatform {
  public:
    FloatingPlatform(double wave_height, double wave_period);
    ~FloatingPlatform() {}

    void SetStepSize(double step) { time_step = step; }
    void SetSolver(ChSolver::Type type) { solver_type = type; }
    void SetRenderFPS(double fps) { render_fps = 30; }

    void CreateModel();
    void CreateVisualization();
    void Simulate(double duration, bool real_time);
    void Plot();

  private:
    int N;
    double L;
    double H;

    double time_step;
    ChSolver::Type solver_type;

    double wave_amplitude;
    double wave_omega;

    std::vector<double> out_time;
    std::vector<std::vector<double>> out_heave;

    std::vector<std::shared_ptr<ChBody>> bodies;

    std::unique_ptr<ChFsiSystemTDPF> sysFSI;
    std::unique_ptr<ChFsiFluidSystemTDPF> sysTDPF;
    std::unique_ptr<ChSystem> sysMBS;

    std::shared_ptr<ChVisualSystem> vis;
    double render_fps;
};

FloatingPlatform::FloatingPlatform(double wave_height, double wave_period)
    : N(6),
      L(29.0),
      H(3.2),
      time_step(1e-2),
      render_fps(30),
      solver_type(ChSolver::Type::GMRES),
      wave_amplitude(wave_height / 2),
      wave_omega(CH_2PI / wave_period) {}

void FloatingPlatform::CreateModel() {
    auto hydro_file = GetChronoDataFile("fsi-tdpf/floating_platform/floating_platform.h5");
    ChVector3d g_acc(0.0, 0.0, -9.81);

    // ----- Multibody system
    sysMBS = std::make_unique<ChSystemSMC>();
    switch (solver_type) {
        case ChSolver::Type::APGD:
        case ChSolver::Type::BARZILAIBORWEIN:
        case ChSolver::Type::GMRES: {
            sysMBS->SetSolverType(solver_type);
            auto iterative = sysMBS->GetSolver()->AsIterative();
            iterative->SetMaxIterations(100);
            iterative->EnableDiagonalPreconditioner(true);
            break;
        }
        case ChSolver::Type::SPARSE_LU:
        case ChSolver::Type::SPARSE_QR: {
            sysMBS->SetSolverType(solver_type);
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
    ChColormap cmap(ChColormap::Type::KINDLMANN);
    for (int i = 0; i < N; ++i) {
        auto body = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("fsi-tdpf/floating_platform/body.obj"),
                                                              1.0, false);
        body->SetName("body" + std::to_string(i + 1));
        body->SetPos(ChVector3d((i + 0.5) * L, 0, 0.5 * H));
        body->SetMass(1732460.0);
        body->SetInertiaXX(ChVector3d(751780112.0, 194629819.0, 929312321.0));
        body->SetFixed(false);
        body->GetVisualShape(0)->SetColor(cmap.Get((i + 1.0) / N));
        sysMBS->Add(body);
        bodies.push_back(body);
    }

    // Create joints (revolute or fixed based on configuration)
    for (int i = 0; i < N - 1; ++i) {
        ChFramed frame(ChVector3d((i + 1.0) * L, 0, 0.5 * H), Q_ROTATE_Y_TO_Z);

        auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
        joint->SetName("joint" + std::to_string(i + 1));
        joint->Initialize(bodies[i], bodies[i + 1], frame);
        sysMBS->AddLink(joint);

        auto rsda = chrono_types::make_shared<ChLinkRSDA>();
        rsda->SetName("rsda" + std::to_string(i + 1));
        rsda->Initialize(bodies[i], bodies[i + 1], frame);
        rsda->SetSpringCoefficient(0);
        rsda->SetDampingCoefficient(0);
        rsda->SetRestAngle(0);
        sysMBS->AddLink(rsda);
    }

    // ----- TDPF fluid system
    sysTDPF = std::make_unique<ChFsiFluidSystemTDPF>();
    sysTDPF->SetHydroFilename(hydro_file);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.regular_wave_amplitude_ = wave_amplitude;
    reg_wave_params.regular_wave_omega_ = wave_omega;
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

void FloatingPlatform::CreateVisualization() {
#ifdef CHRONO_VSG
    auto visFSI = chrono_types::make_shared<ChTdpfVisualizationVSG>(sysFSI.get());
    visFSI->SetWaveMeshVisibility(true);
    visFSI->SetWaveMeshParams({(N / 2) * L, 0.0}, {(N + 4) * L, (N + 2) * L});
    visFSI->SetWaveMeshColormap(ChColormap::Type::BLUE, 0.95f);
    visFSI->SetWaveMeshColorMode(ChTdpfVisualizationVSG::ColorMode::HEIGHT, {-wave_amplitude, +wave_amplitude});
    visFSI->SetWaveMeshUpdateFrequency(render_fps);

    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(sysMBS.get());
    visVSG->SetWindowTitle("FSI-TDPF VLFP regular waves");
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(-2 * L, -2 * L, 2 * L), ChVector3d(80.0, 30.0, 0.0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visVSG->SetModelScale(10);
    visVSG->ToggleCOMFrameVisibility();
    visVSG->ToggleCOMSymbolVisibility();

    visVSG->Initialize();

    vis = visVSG;
#endif
}

void FloatingPlatform::Simulate(double duration, bool real_time) {
    out_heave.resize(N);

    ChRealtimeStepTimer realtime_timer;
    double time = 0;
    int render_frame = 0;

    while (time <= duration) {
#ifdef CHRONO_VSG
        if (time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
#endif

        sysFSI->DoStepDynamics(time_step);

        out_time.push_back(sysFSI->GetSimTime());
        for (int i = 0; i < N; ++i)
            out_heave[i].push_back(bodies[i]->GetPos().z());

        time += time_step;
        if (real_time)
            realtime_timer.Spin(time_step);
    }
}

void FloatingPlatform::Plot() {
#ifdef CHRONO_POSTPROCESS
    postprocess::ChGnuPlot gplot;
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("heave (m)");
    gplot.SetTitle("Body heave");
    for (int i = 0; i < N; i++)
        gplot.Plot(out_time, out_heave[i], bodies[i]->GetName(), " with lines lw 2");
#endif
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double duration = 200;
    double time_step = 1.0e-2;
    bool real_time = true;

    double render_fps = 30;

    double wave_height = 4.0;
    double wave_period = 10;

    ////std::string out_dir = GetChronoOutputPath() + "FSI-TDPF_vlfp";
    ////if (!filesystem::create_directory(filesystem::path(out_dir))) {
    ////    cerr << "Error creating directory " << out_dir << endl;
    ////    return 1;
    ////}

    FloatingPlatform platform(wave_height, wave_period);
    platform.SetStepSize(time_step);
    platform.SetSolver(ChSolver::Type::GMRES);
    platform.SetRenderFPS(render_fps);

    platform.CreateModel();
    platform.CreateVisualization();
    platform.Simulate(duration, real_time);
    platform.Plot();

    return 0;
}
