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
    auto float_meshfile = GetChronoDataFile("fsi-tdpf/rm3/float_cog.obj");
    auto plate_meshfile = GetChronoDataFile("fsi-tdpf/rm3/plate_cog.obj");
    auto rm3_hydrofile = GetChronoDataFile("fsi-tdpf/rm3/rm3.h5");

    ChVector3d g_acc(0.0, 0.0, -9.81);

    double t_end = 100;
    double time_step = 1.0e-2;
    bool enforce_realtime = true;

    bool verbose = false;

    bool render_waves = true;
    double render_fps = 30;
    bool snapshots = false;

    double wave_amplitude = 1.0;
    double wave_period = 3.0;

    // ----- Multibody system
    ChSystemNSC sysMBS;
    sysMBS.SetGravitationalAcceleration(g_acc);

    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    sysMBS.SetSolverType(ChSolver::Type::GMRES);
    sysMBS.GetSolver()->AsIterative()->SetMaxIterations(300);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Float body
    std::shared_ptr<ChBody> float_body = chrono_types::make_shared<ChBodyEasyMesh>(  //
        float_meshfile,                                                              // file name
        0,                                                                           // density
        false,  // do not evaluate mass automatically
        true,   // create visualization asset
        false   // collisions
    );
    float_body->SetName("body1");
    float_body->SetPos(ChVector3d(0, 0, -0.72));
    float_body->SetMass(725834);
    float_body->SetInertiaXX(ChVector3d(20907301.0, 21306090.66, 37085481.11));
    float_body->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.1f, 0.1f));
    sysMBS.Add(float_body);

    float_body->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.1f, 0.1f));

    // Plate body
    std::shared_ptr<ChBody> plate_body = chrono_types::make_shared<ChBodyEasyMesh>(  //
        plate_meshfile,                                                              // file name
        0,                                                                           // density
        false,  // do not evaluate mass automatically
        true,   // create visualization asset
        false   // collisions
    );
    plate_body->SetName("body2");
    plate_body->SetPos(ChVector3d(0, 0, -21.29));
    plate_body->SetMass(886691);
    plate_body->SetInertiaXX(ChVector3d(94419614.57, 94407091.24, 28542224.82));
    plate_body->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.1f, 0.6f));
    sysMBS.Add(plate_body);

    // Prismatic joint between the two bodies
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(float_body, plate_body, false, ChFramed(ChVector3d(0, 0, -0.72)),
                          ChFramed(ChVector3d(0, 0, -21.29)));
    sysMBS.AddLink(prismatic);

    // PTO TSDA
    auto prismatic_pto = chrono_types::make_shared<ChLinkTSDA>();
    prismatic_pto->Initialize(float_body, plate_body, false, ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.29));
    prismatic_pto->SetDampingCoefficient(0.0);
    sysMBS.AddLink(prismatic_pto);

    // ----- TDPF fluid system
    ChFsiFluidSystemTDPF sysTDPF;
    sysTDPF.SetGravitationalAcceleration(g_acc);

    // Set hydro input file
    sysTDPF.SetHydroFilename(rm3_hydrofile);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.num_bodies_ = 2;
    reg_wave_params.regular_wave_amplitude_ = wave_amplitude;
    reg_wave_params.regular_wave_omega_ = CH_2PI / wave_period;
    sysTDPF.AddWaves(reg_wave_params);

    // ----- FSI system
    ChFsiSystemTDPF sysFSI(&sysMBS, &sysTDPF);
    sysFSI.SetVerbose(verbose);
    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    // Add FSI body
    sysFSI.AddFsiBody(float_body, nullptr, false);
    sysFSI.AddFsiBody(plate_body, nullptr, false);

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
    visVSG->SetWindowTitle("FSI-TDPF RM3 regular waves");
    visVSG->SetWindowSize(1280, 720);
    visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
    visVSG->AddCamera(ChVector3d(58, -65, 18), ChVector3d(2, -1, -6));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);
    visVSG->SetModelScale(15);
    visVSG->ToggleRefFrameVisibility();
    visVSG->ToggleCOMSymbolVisibility();

    visVSG->Initialize();
    vis = visVSG;
#endif

    // ----- Create output directory
    std::string out_dir = GetChronoOutputPath() + "FSI-TDPF_rm3";
    std::string img_dir = out_dir + "/reg_waves_img";
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
    std::string out_file = out_dir + "/reg_waves.txt";
    ChWriterCSV csv(" ");

    // ----- Simulation loop
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

        csv << time << float_body->GetPos().z() << plate_body->GetPos().z() << float_body->GetPos().x() << endl;

        sysFSI.DoStepDynamics(time_step);

        time += time_step;
        if (enforce_realtime)
            realtime_timer.Spin(time_step);
    }

    csv.WriteToFile(out_file, "time(s)  Float_heave(m)  Plate_heave(m)  Float_drift(m)");

#ifdef CHRONO_POSTPROCESS
    // ----- Output plot
    postprocess::ChGnuPlot gplot(out_dir + "/reg_waves.gpl");
    gplot.SetGrid();
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("heave float (m)");
    gplot.SetLabelY2("heave plate (m)");
    gplot.SetCommand("set ytics 0.05 nomirror");
    gplot.SetCommand("set y2tics 0.05 nomirror");
    gplot.SetTitle("RM3 regular waves");
    gplot.Plot(out_file, 1, 2, "float", " axes x1y1 with lines lt rgb '#FF5500' lw 2");
    gplot.Plot(out_file, 1, 3, "plate", " axes x1y2 with lines lt rgb '#0055FF' lw 2");
#endif

    cout << "Output in " << out_dir << endl;

    return 0;
}
