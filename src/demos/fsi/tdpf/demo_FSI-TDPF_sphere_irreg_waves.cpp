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

int main(int argc, char* argv[]) {
    auto sphere_meshfile = GetChronoDataFile("fsi-tdpf/sphere/sphere.obj");
    auto sphere_hydrofile = GetChronoDataFile("fsi-tdpf/sphere/sphere.h5");

    ChVector3d g_acc(0.0, 0.0, -9.81);

    double t_end = 100;
    double time_step = 1.5e-2;

    double spring_coefficient = 0;
    double damping_coefficient = 0;

    double wave_height = 4;
    double wave_period = 8;

    bool lock = false;
    bool verbose = false;

    // ----- Multibody system
    ChSystemNSC sysMBS;
    sysMBS.SetGravitationalAcceleration(g_acc);

    sysMBS.SetSolverType(ChSolver::Type::GMRES);
    sysMBS.GetSolver()->AsIterative()->SetMaxIterations(300);

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
    sysTDPF.SetGravitationalAcceleration(g_acc);

    // Set hydro input file
    sysTDPF.SetHydroFilename(sphere_hydrofile);

    // Add irregular waves
    IrregularWaveParams irreg_wave_params;
    irreg_wave_params.num_bodies_ = 1;
    irreg_wave_params.simulation_dt_ = time_step;
    irreg_wave_params.simulation_duration_ = t_end;
    irreg_wave_params.ramp_duration_ = 60.0;
    irreg_wave_params.wave_height_ = wave_height;
    irreg_wave_params.wave_period_ = wave_period;
    irreg_wave_params.frequency_min_ = 0.001;
    irreg_wave_params.frequency_max_ = 1.0;
    irreg_wave_params.nfrequencies_ = 1000;
    sysTDPF.AddWaves(irreg_wave_params);

    // ----- FSI system
    ChFsiSystemTDPF sysFSI(&sysMBS, &sysTDPF);
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
    visFSI->SetWaveMeshVisibility(true);
    visFSI->SetWaveMesh({0, 0}, {50, 50}, 64);
    visFSI->SetWavesColormap(ChColormap::Type::BLUE, 0.95f);
    visFSI->SetWavesColorMode(ChTdpfVisualizationVSG::ColorMode::HEIGHT, {-wave_height, +wave_height});

    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visFSI);
    visVSG->AttachSystem(&sysMBS);
    visVSG->SetWindowTitle("FSI-TDPF sphere irregular waves");
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(30, -35, 15), ChVector3d(0, 0, 0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

    visVSG->Initialize();
    vis = visVSG;
#endif

    // ----- Create output directory
    std::string out_dir = GetChronoOutputPath() + "FSI-TDPF_sphere";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }
    std::string out_file = out_dir + "/irreg_waves.txt";
    ChWriterCSV csv(" ");

    // ----- Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double time = 0;
    while (time <= t_end) {
        if (!vis->Run())
            break;
        vis->Render();

        csv << time << sphere->GetPos().z() << endl;

        sysFSI.DoStepDynamics(time_step);

        time += time_step;
        realtime_timer.Spin(time_step);
    }

    csv.WriteToFile(out_file, "time(s)   heave(m)");

#ifdef CHRONO_POSTPROCESS
    // ----- Output plot
    postprocess::ChGnuPlot gplot(out_dir + "/irreg_waves.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("heave (m)");
    gplot.SetTitle("Sphere irregular waves");
    gplot.Plot(out_file, 1, 2, "", " with lines lt rgb '#FF5500' lw 2");
#endif

    cout << "Output in " << out_dir << endl;

    return 0;
}
