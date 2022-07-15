// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
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
// RoboSimian on granular terrain
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_models/robot/robosimian/RoboSimian.h"

#include "chrono_vehicle/terrain/GranularTerrain.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "granular.h"

using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     robosimian::LocomotionMode& mode,
                     ChContactMethod& method,
                     double& sim_time,
                     double& step_size,
                     double& out_fps,
                     double& pov_fps,
                     int& nthreads,
                     bool& drop,
                     bool& output,
                     bool& pov_output,
                     bool& render,
                     std::string& suffix);

// =============================================================================

// Granular terrain parameters
double r_g = 0.0075;
double rho_g = 2000;
double coh_g = 40e3;
float mu_g = 0.4f;

// Terrain patch
double patch_length = 3;  //// 5
double patch_width = 2.5;
unsigned int num_layers = 5;  //// 10

// =============================================================================

int main(int argc, char* argv[]) {
    // ----------------------------
    // Parse command line arguments
    // ----------------------------

    robosimian::LocomotionMode mode = robosimian::LocomotionMode::WALK;
    ChContactMethod contact_method = ChContactMethod::NSC;
    double step_size = 1e-4;
    double duration_sim = 10;
    double out_fps = 100;
    double pov_fps = 60;
    int nthreads = 2;
    bool drop = true;
    bool render = true;
    bool pov_output = true;
    bool output = true;
    std::string suffix = "";

    if (!GetProblemSpecs(argc, argv, mode, contact_method, duration_sim, step_size, out_fps, pov_fps, nthreads, drop,
                         output, pov_output, render, suffix)) {
        return 1;
    }

    // ------------
    // Timed events
    // ------------

    double duration_pose = 0.5;            // Interval to assume initial pose
    double duration_settle_terrain = 1.0;  // Interval to allow granular material settling
    double duration_settle_robot = 0.5;    // Interval to allow robot settling on terrain
    double duration_hold = duration_settle_terrain + duration_settle_robot;

    double time_create_terrain = duration_pose;  // create terrain after robot assumes initial pose
    double time_release = time_create_terrain + duration_settle_terrain;  // release robot after terrain settling
    double time_start = time_release + duration_settle_robot;  // start actual simulation after robot settling
    double time_end = time_start + duration_sim;               // end simulation after specified duration

    // -----------------------------
    // Initialize output directories
    // -----------------------------

    const std::string dir = GetChronoOutputPath() + "ROBOSIMIAN_GRANULAR";
    std::string pov_dir = dir + "/POVRAY" + suffix;
    std::string out_dir = dir + "/RESULTS" + suffix;

    if (!filesystem::create_directory(filesystem::path(dir))) {
        cout << "Error creating directory " << dir << endl;
        return 1;
    }
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }
    }
    if (pov_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            cout << "Error creating directory " << pov_dir << endl;
            return 1;
        }
    }

    // ----------------------
    // Write problem settings
    // ----------------------

    std::ofstream outf;
    outf.open(dir + "/settings" + suffix + ".txt", std::ios::out);
    outf << "System type (contact method): ";
    switch (contact_method) {
        case ChContactMethod::NSC:
            outf << "NSC (non-smooth contact)" << endl;
            break;
        case ChContactMethod::SMC:
            outf << "SMC (smooth contact)" << endl;
            break;
    }
    outf << "Locomotion mode: ";
    switch (mode) {
        case robosimian::LocomotionMode::WALK:
            outf << "WALK" << endl;
            break;
        case robosimian::LocomotionMode::SCULL:
            outf << "SCULL" << endl;
            break;
        case robosimian::LocomotionMode::INCHWORM:
            outf << "INCHWORM" << endl;
            break;
        case robosimian::LocomotionMode::DRIVE:
            outf << "DRIVE" << endl;
            break;
    }
    outf << "Release robot? " << (drop ? "YES" : "NO") << endl;
    outf << endl;
    outf << "Time terrain settling: " << duration_settle_terrain << endl;
    outf << "Time robot settling: " << duration_settle_robot << endl;
    outf << "Time locomotion: " << duration_sim << endl;
    outf << "Total time: " << time_end << endl;
    outf << endl;
    outf << "Step size: " << step_size << endl;
    outf << "Number threads: " << nthreads << endl;
    outf << endl;
    outf << "Result output?" << (output ? "YES" : "NO") << endl;
    if (output) {
        outf << "  Output frequency (FPS): " << out_fps << endl;
        outf << "  Output directory:       " << out_dir << endl;
    }
    outf << "POV-Ray output? " << (pov_output ? "YES" : "NO") << endl;
    if (pov_output) {
        outf << "  Output frequency (FPS): " << pov_fps << endl;
        outf << "  Output directory:       " << pov_dir << endl;
    }
    outf.close();

    // -------------
    // Create system
    // -------------

    ChSystemMulticore* sys = nullptr;
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto my_sys = new ChSystemMulticoreNSC;
            cout << "System type: NSC" << endl;

            my_sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            my_sys->GetSettings()->solver.max_iteration_normal = 0;
            my_sys->GetSettings()->solver.max_iteration_sliding = 100;
            my_sys->GetSettings()->solver.max_iteration_spinning = 0;
            my_sys->GetSettings()->solver.compute_N = false;
            my_sys->GetSettings()->solver.alpha = 0;
            my_sys->GetSettings()->solver.contact_recovery_speed = 1000;
            my_sys->GetSettings()->collision.collision_envelope = 0.01;

            my_sys->ChangeSolverType(SolverType::APGD);

            sys = my_sys;
            break;
        }
        case ChContactMethod::SMC: {
            auto my_sys = new ChSystemMulticoreSMC;
            cout << "System type: SMC" << endl;

            my_sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            my_sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::OneStep;

            sys = my_sys;
            break;
        }
    }

    sys->Set_G_acc(ChVector<double>(0, 0, -9.8));
    ////sys->Set_G_acc(ChVector<double>(0, 0, 0));

    int max_threads = ChOMP::GetNumThreads();
    if (nthreads > max_threads)
        nthreads = max_threads;
    sys->SetNumThreads(nthreads);

    sys->GetSettings()->solver.tolerance = 1e-3;
    sys->GetSettings()->solver.max_iteration_bilateral = 100;
    sys->GetSettings()->solver.cache_step_length = true;
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.bilateral_clamp_speed = 1e8;

    sys->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    // -----------------------
    // Create RoboSimian robot
    // -----------------------

    robosimian::RoboSimian robot(sys, true, true);

    // Set output directory
    robot.SetOutputDirectory(out_dir);

    // Ensure wheels are actuated in ANGLE mode (required for Chrono::Multicore)
    robot.SetMotorActuationMode(robosimian::ActuationMode::ANGLE);

    ////robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    robot.Initialize(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI)));

    robot.SetVisualizationTypeChassis(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeSled(robosimian::VisualizationType::MESH);
    robot.SetVisualizationTypeLimbs(robosimian::VisualizationType::MESH);

    // -----------------------------------
    // Create a driver and attach to robot
    // -----------------------------------

    std::shared_ptr<robosimian::RS_Driver> driver;

    switch (mode) {
        case robosimian::LocomotionMode::WALK: {
            auto drv = chrono_types::make_shared<robosimian::RS_Driver>(
                "",                                                           // start input file
                GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt"),  // cycle input file
                "",                                                           // stop input file
                true);
            driver = drv;
            cout << "Locomotion mode: WALK" << endl;
            break;
        }
        case robosimian::LocomotionMode::SCULL: {
            auto drv = chrono_types::make_shared<robosimian::RS_Driver>(
                GetChronoDataFile("robot/robosimian/actuation/sculling_start.txt"),   // start input file
                GetChronoDataFile("robot/robosimian/actuation/sculling_cycle2.txt"),  // cycle input file
                GetChronoDataFile("robot/robosimian/actuation/sculling_stop.txt"),    // stop input file
                true);
            driver = drv;
            cout << "Locomotion mode: SCULL" << endl;
            break;
        }
        case robosimian::LocomotionMode::INCHWORM: {
            auto drv = chrono_types::make_shared<robosimian::RS_Driver>(
                GetChronoDataFile("robot/robosimian/actuation/inchworming_start.txt"),  // start input file
                GetChronoDataFile("robot/robosimian/actuation/inchworming_cycle.txt"),  // cycle input file
                GetChronoDataFile("robot/robosimian/actuation/inchworming_stop.txt"),   // stop input file
                true);
            driver = drv;
            cout << "Locomotion mode: INCHWORM" << endl;
            break;
        }
        case robosimian::LocomotionMode::DRIVE: {
            auto drv = chrono_types::make_shared<robosimian::RS_Driver>(
                GetChronoDataFile("robot/robosimian/actuation/driving_start.txt"),  // start input file
                GetChronoDataFile("robot/robosimian/actuation/driving_cycle.txt"),  // cycle input file
                GetChronoDataFile("robot/robosimian/actuation/driving_stop.txt"),   // stop input file
                true);
            driver = drv;
            cout << "Locomotion mode: DRIVE" << endl;
            break;
        }
    }

    robosimian::RS_DriverCallback cbk(&robot);
    driver->RegisterPhaseChangeCallback(&cbk);

    driver->SetTimeOffsets(duration_pose, duration_hold);
    robot.SetDriver(driver);

    // -----------------
    // Initialize OpenGL
    // -----------------

    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.AttachSystem(sys);
        gl_window.Initialize(1280, 720, "RoboSimian - Granular terrain");
        gl_window.SetCamera(ChVector<>(2, -2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::WIREFRAME);
    }

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    int out_steps = (int)std::ceil((1.0 / out_fps) / step_size);
    int pov_steps = (int)std::ceil((1.0 / pov_fps) / step_size);
    int sim_frame = 0;
    int pov_frame = 0;

    bool terrain_created = false;
    bool robot_released = false;
    double x_max = 0;

    double terrain_bottom_height = 0;
    std::pair<double, double> terrain_init_height;
    std::pair<double, double> terrain_settled_height;

    GroundGranularA ground(sys);
    ////GroundGranularB ground(sys);

    while (true) {
        double time = sys->GetChTime();
        double x = robot.GetChassisPos().x();

        if (time >= time_end) {
            cout << "Reached final time: " << time << endl;
            break;
        }

        if (drop) {
            if (!terrain_created && time > time_create_terrain) {
                // Find robot bottom point (below wheels)
                double z = robot.GetWheelPos(robosimian::FR).z() - 0.15;

                // Create granular terrain
                cout << "Time: " << time << "  CREATE TERRAIN" << endl;

                ground.SetParticleProperties(r_g, rho_g, mu_g, coh_g);
                ground.SetPatchProperties(patch_length, patch_width, num_layers);

                ground.Initialize(x - 1.0, z, step_size);

                terrain_init_height = ground.GetTopHeight();
                terrain_bottom_height = ground.GetBottomHeight();

                cout << "  Generated " << ground.GetNumParticles() << " particles" << endl;
                cout << "  Terrain bottom: " << terrain_bottom_height << endl;
                cout << "  Terrain top:    " << terrain_init_height.first << " " << terrain_init_height.second << endl;

                x_max = (x - 1.0) + patch_length - 1.0;

                terrain_created = true;
            }

            if (!robot_released && time > time_release) {
                terrain_settled_height = ground.GetTopHeight();

                cout << "Time: " << time << "  TRANSLATE & RELEASE ROBOT" << endl;
                cout << "  Terrain bottom: " << terrain_bottom_height << endl;
                cout << "  Terrain top:    " << terrain_settled_height.first << " " << terrain_settled_height.second << endl;

                robot.Translate(ChVector<>(0, 0, terrain_settled_height.first - terrain_init_height.first));
                robot.GetChassisBody()->SetBodyFixed(false);
                robot_released = true;
            }

            if (robot_released && x > x_max) {
                cout << "Time: " << time << "  Reached maximum distance" << endl;
                break;
            }
        }

        // Output results
        if (output && sim_frame % out_steps == 0) {
            robot.Output();
        }

        // Output POV-Ray data
        if (pov_output && sim_frame % pov_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/data_%04d.dat", pov_dir.c_str(), pov_frame + 1);
            utils::WriteVisualizationAssets(sys, filename);
            pov_frame++;
        }

        ////double A = CH_C_PI / 6;
        ////double freq = 2;
        ////double val = 0.5 * A * (1 - std::cos(CH_C_2PI * freq * time));
        ////robot.Activate(robosimian::FR, "joint2", time, val);
        ////robot.Activate(robosimian::RL, "joint5", time, val);
        ////robot.Activate(robosimian::FL, "joint8", time, -0.4 * time);

        robot.DoStepDynamics(step_size);

        ////if (sys->GetNcontacts() > 0) {
        ////    robot.ReportContacts();
        ////}

        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active()) {
                gl_window.Render();
            } else {
                break;
            }
        }

        sim_frame++;
    }

    return 0;
}

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     robosimian::LocomotionMode& mode,
                     ChContactMethod& method,
                     double& sim_time,
                     double& step_size,
                     double& out_fps,
                     double& pov_fps,
                     int& nthreads,
                     bool& drop,
                     bool& output,
                     bool& pov_output,
                     bool& render,
                     std::string& suffix) {
    // Default values
    mode = robosimian::LocomotionMode::WALK;
    method = ChContactMethod::NSC;
    step_size = 1e-4;
    sim_time = 10;
    out_fps = 100;
    pov_fps = 60;
    nthreads = 2;
    drop = true;
    render = true;
    pov_output = true;
    output = true;
    suffix = "";

    ChCLI cli(argv[0]);

    cli.AddOption<int>("Demo", "mode", "Locomotion mode (0:walk, 1:scull, 2:inchworm, 3:drive)", "0");
    cli.AddOption<int>("Demo", "method", "Contact method (0: NSC, 1:SMC)", "0");
    cli.AddOption<double>("Demo", "sim_time", "Simulation length after robot release [s]", "10");
    cli.AddOption<double>("Demo", "step_size", "Integration step size [s]", "1e-4");
    cli.AddOption<double>("Demo", "out_fps", "Output frequency [FPS]", "100");
    cli.AddOption<double>("Demo", "pov_fps", "POV-Ray output frequency [FPS]", "100");
    cli.AddOption<bool>("Demo", "drop", "Release robot?", "true");
    cli.AddOption<bool>("Demo", "render", "OpenGL rendering?", "true");
    cli.AddOption<bool>("Demo", "output", "Generate result output files", "true");
    cli.AddOption<bool>("Demo", "pov_output", "Generate POV-Ray output files", "true");
    cli.AddOption<std::string>("Demo", "suffix", "Suffix for output directory names", "");
    cli.AddOption<int>("Demo", "threads", "Number of OpenMP threads", "2");

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    switch (cli.GetAsType<int>("mode")) {
        case 0:
            mode = robosimian::LocomotionMode::WALK;
            break;
        case 1:
            mode = robosimian::LocomotionMode::SCULL;
            break;
        case 2:
            mode = robosimian::LocomotionMode::INCHWORM;
            break;
        case 3:
            mode = robosimian::LocomotionMode::DRIVE;
            break;
        default:
            cout << "Invalid locomotion mode" << endl;
            cli.Help();
            return false;
    }

    switch (cli.GetAsType<int>("method")) {
        case 0:
            method = ChContactMethod::NSC;
            break;
        case 1:
            method = ChContactMethod::SMC;
            break;
        default:
            cout << "Invalid contact method" << endl;
            cli.Help();
            return false;
    }

    sim_time = cli.GetAsType<double>("sim_time");
    step_size = cli.GetAsType<double>("step_size");
    out_fps = cli.GetAsType<double>("out_fps");
    pov_fps = cli.GetAsType<double>("pov_fps");
    drop = cli.GetAsType<bool>("drop");    
    render = cli.GetAsType<bool>("render");
    output = cli.GetAsType<bool>("output");    
    pov_output = cli.GetAsType<bool>("pov_output");
    suffix = cli.GetAsType<std::string>("suffix");
    nthreads = cli.GetAsType<int>("threads");

    return true;
}
