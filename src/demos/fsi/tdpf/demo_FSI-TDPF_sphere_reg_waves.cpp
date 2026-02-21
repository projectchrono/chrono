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
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

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

double t_end = 100;
double time_step = 1.5e-2;
bool enforce_realtime = true;

bool lock = false;
bool verbose = false;

bool render_waves = true;
double render_fps = 30;
bool snapshots = false;

bool debug_sys = false;

ChSolver::Type solver_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type solver_type = ChSolver::Type::APGD;
////ChSolver::Type solver_type = ChSolver::Type::GMRES;

bool use_diag_precond = true;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    auto sphere_meshfile = GetChronoDataFile("fsi-tdpf/sphere/sphere.obj");
    auto sphere_hydrofile = GetChronoDataFile("fsi-tdpf/sphere/sphere.h5");

    ChVector3d g_acc(0.0, 0.0, -9.81);

    double wave_amplitude = 2.5;
    double wave_period = 5;
    double spring_coefficient = 0;
    double damping_coefficient = 1e5;

    // HydroChrono test values
    ////double wave_amplitude = 2.374;
    ////double wave_period = 11;
    ////double spring_coefficient = 0;
    ////double damping_coefficient = 1077123.445;

    // ----- Multibody system
    ChSystemNSC sysMBS;
    sysMBS.SetSolverType(solver_type);
    if (sysMBS.GetSolver()->AsIterative()) {
        sysMBS.GetSolver()->AsIterative()->SetMaxIterations(300);
        sysMBS.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(use_diag_precond);
    }

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Sphere body
    auto sphere = chrono_types::make_shared<ChBodyEasyMesh>(  //
        sphere_meshfile,                                      // file name
        1000,                                                 // density
        false,                                                // do not evaluate mass automatically
        true,                                                 // create visualization asset
        false                                                 // do not collide
    );
    sphere->SetName("body1");  // must set body name correctly! (must match .h5 file)
    sphere->SetPos(ChVector3d(0, 0, -2));
    sphere->SetMass(261.8e3);
    sphere->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.1f, 0.1f));
    sysMBS.Add(sphere);

    // Prismatic joint between sphere and ground (limit to heave motion only)
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(sphere, ground, false, ChFramed(ChVector3d(0, 0, -2)), ChFramed(ChVector3d(0, 0, -5)));
    sysMBS.AddLink(prismatic);

    // TSDA between sphere and ground
    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(sphere, ground, false, ChVector3d(0, 0, -2),
                     ChVector3d(0, 0, -5));  // false means positions are in global frame
    // Note: rest length is calculated from initial position when not explicitly set
    tsda->SetSpringCoefficient(spring_coefficient);
    tsda->SetDampingCoefficient(damping_coefficient);
    sysMBS.AddLink(tsda);

    if (lock) {
        auto weld = chrono_types::make_shared<ChLinkLockLock>();
        weld->Initialize(ground, sphere, ChFramed());
        sysMBS.AddLink(weld);
    }

    // ----- TDPF fluid system
    ChFsiFluidSystemTDPF sysTDPF;
    sysTDPF.SetHydroFilename(sphere_hydrofile);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.regular_wave_amplitude = wave_amplitude;
    reg_wave_params.regular_wave_omega = CH_2PI / wave_period;
    sysTDPF.AddWaves(reg_wave_params);

    // ----- FSI system
    ChFsiSystemTDPF sysFSI(&sysMBS, &sysTDPF);
    sysFSI.SetGravitationalAcceleration(g_acc);
    sysFSI.SetVerbose(verbose);
    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    // Add FSI body
    sysFSI.AddFsiBody(sphere, nullptr, false);

    sysFSI.Initialize();

    // ----- Run-time visualization
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    auto visFSI = chrono_types::make_shared<ChTdpfVisualizationVSG>(&sysFSI);
    visFSI->SetWaveMeshVisibility(render_waves);
    visFSI->SetWaveMeshColormap(ChColormap::Type::BLUE, 0.95f);
    visFSI->SetWaveMeshColorMode(ChTdpfVisualizationVSG::ColorMode::HEIGHT, {-wave_amplitude, +wave_amplitude});
    visFSI->SetWaveMeshUpdateFrequency(render_fps);

    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&sysMBS);
    visVSG->SetWindowTitle("FSI-TDPF sphere regular waves");
    visVSG->SetWindowSize(1280, 800);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(30, -35, 15), ChVector3d(0, 0, 0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

    visVSG->Initialize();
    vis = visVSG;
#endif

    // ----- Create output directory
    std::string out_dir = GetChronoOutputPath() + "FSI-TDPF_sphere";
    std::string img_dir = out_dir + "/reg_waves_img";
    std::string dbg_dir = out_dir + "/reg_waves_dbg_" + sysMBS.GetSolver()->GetTypeAsString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    if (snapshots) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cerr << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }
    if (debug_sys) {
        if (!filesystem::create_directory(filesystem::path(dbg_dir))) {
            std::cerr << "Error creating directory " << dbg_dir << std::endl;
            return 1;
        }
    }
    std::string out_file = out_dir + "/reg_waves.txt";
    ChWriterCSV csv(" ");

    // ----- ChSystem debug log
    sysMBS.EnableSolverMatrixWrite(debug_sys, dbg_dir);

    // ----- Simulation loop
    cout << "Using solver: " << sysMBS.GetSolver()->GetTypeAsString() << endl;

    ChRealtimeStepTimer realtime_timer;
    double time = 0;
    int render_frame = 0;

    while (time <= t_end) {
#ifdef CHRONO_VSG
        if (time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            if (snapshots) {
                if (verbose)
                    cout << " -- Snapshot frame " << render_frame << " at t = " << time << endl;
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(5) << std::setfill('0') << render_frame << ".png";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }
#endif

        csv << time << sphere->GetPos().z() << endl;

        sysFSI.DoStepDynamics(time_step);

        time += time_step;
        if (enforce_realtime)
            realtime_timer.Spin(time_step);
    }

    csv.WriteToFile(out_file, "time(s)   heave(m)");

#ifdef CHRONO_POSTPROCESS
    // ----- Output plot
    postprocess::ChGnuPlot gplot(out_dir + "/reg_waves.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("heave (m)");
    gplot.SetTitle("Sphere regular waves");
    gplot.Plot(out_file, 1, 2, "", " with lines lt rgb '#FF5500' lw 2");
#endif

    cout << "Output in " << out_dir << endl;

    return 0;
}
