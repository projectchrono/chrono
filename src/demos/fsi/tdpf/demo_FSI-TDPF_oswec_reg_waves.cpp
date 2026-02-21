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
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
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

double t_end = 100;
double time_step = 1.0e-2;
bool enforce_realtime = true;

bool verbose = false;

bool render_waves = true;
double render_fps = 30;
bool snapshots = false;

////ChSolver::Type solver_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type solver_type = ChSolver::Type::SPARSE_LU;
ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;
////ChSolver::Type solver_type = ChSolver::Type::GMRES;

bool use_diag_precond = true;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    auto base_meshfile = GetChronoDataFile("fsi-tdpf/oswec/base.obj");
    auto flap_meshfile = GetChronoDataFile("fsi-tdpf/oswec/flap.obj");
    auto oswec_hydrofile = GetChronoDataFile("fsi-tdpf/oswec/oswec.h5");

    ChVector3d g_acc(0.0, 0.0, -9.81);

    double wave_amplitude = 0.1;
    double wave_period = 6.0;

    // ----- Multibody system
    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(g_acc);

#ifndef CHRONO_PARDISO_MKL
    if (solver_type == ChSolver::Type::PARDISO_MKL) {
        std::cerr << "Pardiso MKL not available. Switching to GMRES" << std::endl;
        solver_type = ChSolver::Type::GMRES;
    }
#endif

    if (solver_type == ChSolver::Type::PARDISO_MKL) {
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        sysMBS.SetSolver(mkl_solver);
    } else {
        sysMBS.SetSolverType(solver_type);
    }

    if (sysMBS.GetSolver()->AsIterative()) {
        sysMBS.GetSolver()->AsIterative()->SetMaxIterations(300);
        sysMBS.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(use_diag_precond);
    }

    // Set scaling factor (for generalized mass and force) to improve system matrix condition number
    sysMBS.GetSolver()->SetConditioningFactor(1e-3);

    // Flap body
    auto flap_body = chrono_types::make_shared<ChBodyEasyMesh>(flap_meshfile,
                                                               1000,   // density
                                                               false,  // do not evaluate mass automatically
                                                               true,   // create visualization asset
                                                               false   // no collisions
    );

    sysMBS.Add(flap_body);
    flap_body->SetName("body1");
    flap_body->SetPos(ChVector3d(0.0, 0.0, -3.9));
    flap_body->SetMass(1.27e5);
    flap_body->SetInertiaXX(ChVector3d(0.65e6, 1.85e6, 0.31e6));

    // Base body
    auto base_body = chrono_types::make_shared<ChBodyEasyMesh>(base_meshfile,
                                                               1000,   // density
                                                               false,  // do not evaluate mass automatically
                                                               true,   // create visualization asset
                                                               false   // no collisions
    );
    sysMBS.Add(base_body);
    base_body->SetName("body2");
    base_body->SetPos(ChVector3d(0, 0, -10.15));
    base_body->SetMass(1e6);
    base_body->SetInertiaXX(ChVector3d(1e6, 1e6, 1e6));
    base_body->GetVisualShape(0)->SetColor(ChColor(0.96f, 0.56f, 0.23f));

    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(ground);
    ground->SetFixed(true);

    // Weld base to ground
    auto weld = chrono_types::make_shared<ChLinkLockLock>();
    weld->Initialize(base_body, ground, ChFramed());
    sysMBS.Add(weld);

    // Connect flap to base
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(base_body, flap_body, ChFramed(ChVector3d(0.0, 0.0, -8.9), Q_ROTATE_Y_TO_Z));
    sysMBS.AddLink(revolute);

    // ----- TDPF fluid system
    ChFsiFluidSystemTDPF sysTDPF;
    sysTDPF.SetGravitationalAcceleration(g_acc);

    // Set hydro input file
    sysTDPF.SetHydroFilename(oswec_hydrofile);

    // Add regular wave
    RegularWaveParams reg_wave_params;
    reg_wave_params.regular_wave_amplitude = wave_amplitude;
    reg_wave_params.regular_wave_omega = CH_2PI / wave_period;
    sysTDPF.AddWaves(reg_wave_params);

    // ----- FSI system
    ChFsiSystemTDPF sysFSI(&sysMBS, &sysTDPF);
    sysFSI.SetVerbose(verbose);
    sysFSI.SetStepSizeCFD(time_step);
    sysFSI.SetStepsizeMBD(time_step);

    // Add FSI body
    sysFSI.AddFsiBody(flap_body, nullptr, false);
    sysFSI.AddFsiBody(base_body, nullptr, false);

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
    std::string out_dir = GetChronoOutputPath() + "FSI-TDPF_oswec";
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
        double flap_rot = flap_body->GetRot().GetCardanAnglesXYZ().y();
        csv << time << flap_rot * CH_RAD_TO_DEG << endl;

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
    gplot.SetLabelY("flap rot (deg)");
    gplot.SetTitle("OSWEC regular waves");
    gplot.Plot(out_file, 1, 2, "", " with lines lt rgb '#FF5500' lw 2");
#endif

    cout << "Output in " << out_dir << endl;

    return 0;
}
