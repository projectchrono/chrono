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
// Authors: Radu Serban
// =============================================================================
//
// Application for running Chrono simulations specified through YAML files.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#ifdef CHRONO_VEHICLE
    #include "chrono_parsers/yaml/ChParserVehicleYAML.h"
    #include "chrono_vehicle/driver/ChInteractiveDriver.h"
#endif
#ifdef CHRONO_FSI
    #include "chrono_parsers/yaml/ChParserFsiYAML.h"
#endif

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
    #ifdef CHRONO_VEHICLE
        #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
        #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
    #endif
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::parsers;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

bool ParseArgs(int argc, char** argv, std::string& yaml_filename, std::string& out_dir, bool& disable_output);
bool RunMBS(const std::string& yaml_filename, std::string& out_dir, bool disable_output);
bool RunVEHICLE(const std::string& yaml_filename, std::string& out_dir, bool disable_output);
bool RunFSI(const std::string& yaml_filename, std::string& out_dir, bool disable_output);

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Proces command line arguments
    bool disable_output = false;
    std::string yaml_filename = "";
    std::string out_dir = GetChronoOutputPath() + "YAML_CHRONO/";
    if (!ParseArgs(argc, argv, yaml_filename, out_dir, disable_output))
        return 1;

    cout << endl;
    cout << "YAML specification file: " << yaml_filename << endl;
    cout << "Output directory:        " << out_dir << endl;
    cout << "Disable output?          " << (disable_output ? "yes" : "no") << endl;

    // Peek in file, read type, and call appropriate function for processing the YAML file
    auto type = ChParserYAML::ReadYamlFileType(yaml_filename);
    switch (type) {
        case ChParserYAML::YamlFileType::MBS:
            RunMBS(yaml_filename, out_dir, disable_output);
            break;
        case ChParserYAML::YamlFileType::VEHICLE:
#ifdef CHRONO_VEHICLE
            RunVEHICLE(yaml_filename, out_dir, disable_output);
#else
            cerr << "The Chrono::Vehicle module is not available. Cannot process a vehicle YAML specification." << endl;
#endif
            break;
        case ChParserYAML::YamlFileType::FSI:
#ifdef CHRONO_FSI
            RunFSI(yaml_filename, out_dir, disable_output);
#else
            cerr << "The Chrono::FSI module is not available. Cannot process an FSI YAML specification." << endl;
#endif
            break;

        default:
            cerr << "\nError: Unsupported YAML file type.\n" << endl;
    }

    return 0;
}

// -----------------------------------------------------------------------------

bool ParseArgs(int argc, char** argv, std::string& yaml_filename, std::string& out_dir, bool& disable_output) {
    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "s,sim_file", "simulation specification file (YAML)");
    cli.AddOption<std::string>("", "o,out_dir", "output directory", out_dir);
    cli.AddOption<bool>("", "no_output", "Disable output");

    if (!cli.Parse(argc, argv, true))
        return false;

    try {
        yaml_filename = cli.Get("sim_file").as<std::string>();
    } catch (std::domain_error&) {
        cerr << "\nError: Missing YAML specification file." << endl;
        cli.Help();
        return false;
    }

    disable_output = cli.GetAsType<bool>("no_output");

    out_dir = cli.Get("out_dir").as<std::string>();

    return true;
}

// -----------------------------------------------------------------------------

bool RunMBS(const std::string& yaml_filename, std::string& out_dir, bool disable_output) {
    // Create YAML parser object, load the YAML file, then create a Chrono system and populate it
    parsers::ChParserMbsYAML parser(yaml_filename, true);
    auto sys = parser.CreateSystem();
    parser.Populate(*sys, ChFramed());

    // Print hierarchy of modeling components in ChSystem
    ////std::cout << "Number of moidel instances: " << parser.GetNumInstances() << std::endl;
    ////sys->ShowHierarchy(std::cout);

    // Extract information from parsed YAML files
    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool real_time = parser.EnforceRealtime();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    CameraVerticalDir camera_vertical = parser.GetCameraVerticalDir();
    const ChVector3d& camera_location = parser.GetCameraLocation();
    const ChVector3d& camera_target = parser.GetCameraTarget();
    bool enable_shadows = parser.EnableShadows();
    bool output = parser.Output() && !disable_output;
    double output_fps = parser.GetOutputFPS();

    // Print system hierarchy
    ////sys->ShowHierarchy(std::cout);

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    if (render) {
        auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
        vis_vsg->AttachSystem(sys.get());
        vis_vsg->SetWindowTitle("YAML model - " + model_name);
        vis_vsg->AddCamera(camera_location, camera_target);
        vis_vsg->SetWindowSize(1280, 800);
        vis_vsg->SetWindowPosition(100, 100);
        vis_vsg->SetCameraVertical(camera_vertical);
        vis_vsg->SetCameraAngleDeg(40.0);
        vis_vsg->SetLightIntensity(1.0f);
        vis_vsg->SetLightDirection(-CH_PI_4, CH_PI_4);
        vis_vsg->EnableShadows(enable_shadows);
        vis_vsg->ToggleAbsFrameVisibility();
        vis_vsg->SetAbsFrameScale(2.0);
        vis_vsg->Initialize();

        vis = vis_vsg;
    }
#else
    render = false;
#endif

    // Create output directory
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return false;
        }
        out_dir = out_dir + "/" + model_name;
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return false;
        }
        parser.SetOutputDir(out_dir);
    }

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0;
    int render_frame = 0;
    int output_frame = 0;

    while (true) {
        if (render) {
            if (!vis->Run())
                break;
            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                render_frame++;
            }
        } else {
            std::cout << "\rt = " << time;
            if (time_end > 0 && time >= time_end)
                break;
        }

        if (output) {
            if (time >= output_frame / output_fps) {
                parser.SaveOutput(*sys, output_frame);
                output_frame++;
            }
        }

        sys->DoStepDynamics(time_step);
        if (real_time)
            rt_timer.Spin(time_step);
        time += time_step;
    }

    return true;
}

bool RunVEHICLE(const std::string& yaml_filename, std::string& out_dir, bool disable_output) {
#ifdef CHRONO_VEHICLE
    // Create the YAML parser object
    parsers::ChParserVehicleYAML parser(yaml_filename, true);

    // Create Chrono system and vehicle model
    auto sys = parser.CreateSystem();
    parser.CreateVehicle(*sys);

    // Extract information from parsed YAML files
    auto vehicle = parser.GetVehicle();
    auto vehicle_type = parser.GetVehicleType();
    auto terrain = parser.GetTerrain();

    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool real_time = parser.EnforceRealtime();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();
    bool enable_shadows = parser.EnableShadows();
    bool output = parser.Output() && !disable_output;

    const ChVector3d& chassis_point = parser.GetChassisPoint();
    double chase_distance = parser.GetChaseDistance();
    double chase_height = parser.GetChaseHeight();

    // Create an interactive VSG driver system
    auto driver = chrono_types::make_shared<vehicle::ChInteractiveDriver>(*vehicle);
    driver->SetSteeringDelta(0.02);
    driver->SetThrottleDelta(0.02);
    driver->SetBrakingDelta(0.06);
    driver->Initialize();

    // Create the vehicle run-time visualization interface and the interactive driver
    std::shared_ptr<vehicle::ChVehicleVisualSystem> vis;
    #ifdef CHRONO_VSG
    if (render) {
        std::shared_ptr<vehicle::ChVehicleVisualSystemVSG> vis_vsg;
        if (vehicle_type == parsers::ChParserVehicleYAML::VehicleType::WHEELED)
            vis_vsg = chrono_types::make_shared<vehicle::ChWheeledVehicleVisualSystemVSG>();
        else
            vis_vsg = chrono_types::make_shared<vehicle::ChTrackedVehicleVisualSystemVSG>();
        vis_vsg->SetWindowTitle("Vehicle YAML demo - " + model_name);
        vis_vsg->AttachVehicle(vehicle.get());
        vis_vsg->AttachDriver(driver.get());
        vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
        vis_vsg->SetChaseCamera(chassis_point, chase_distance, chase_height);
        vis_vsg->SetWindowSize(1280, 800);
        vis_vsg->SetWindowPosition(100, 100);
        vis_vsg->EnableSkyBox();
        vis_vsg->SetCameraAngleDeg(40);
        vis_vsg->SetLightIntensity(1.0f);
        vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        vis_vsg->EnableShadows(enable_shadows);
        vis_vsg->Initialize();

        vis = vis_vsg;
    }
    #else
    render = false;
    #endif

    // Create output directory
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return false;
        }
        out_dir = out_dir + "/" + model_name;
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return false;
        }
        parser.SetOutputDir(out_dir);

        double out_step = 1.0 / parser.GetOutputFPS();
        vehicle->SetOutput(parser.GetOutputType(), parser.GetOutputMode(), out_dir, model_name, out_step);
    }

    // Simulation loop
    vehicle->EnableRealtime(real_time);

    double time = 0;
    int render_frame = 0;

    while (true) {
        if (render) {
            if (!vis->Run())
                break;
            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                render_frame++;
            }
        } else {
            std::cout << "\rt = " << time;
            if (time_end > 0 && time >= time_end)
                break;
        }

        // Get driver inputs
        vehicle::DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        driver->Synchronize(time);
        if (terrain) {
            vehicle->Synchronize(time, driver_inputs, *terrain);
            terrain->Synchronize(time);
        } else {
            vehicle->Synchronize(time, driver_inputs);
        }
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(time_step);
        vehicle->Advance(time_step);
        sys->DoStepDynamics(time_step);
        if (terrain)
            terrain->Advance(time_step);
        if (vis)
            vis->Advance(time_step);

        time += time_step;
    }
#endif

    return true;
}

// -----------------------------------------------------------------------------

bool RunFSI(const std::string& yaml_filename, std::string& out_dir, bool disable_output) {
#ifdef CHRONO_FSI
    // Create the FSI YAML parser object
    parsers::ChParserFsiYAML parser(yaml_filename, true);

    // Create the FSI system and the underlying multibody and fluid systems
    parser.CreateFsiSystem();
    auto sysFSI = parser.GetFsiSystem();
    auto sysCFD = parser.GetFluidSystem();
    auto sysMBS = parser.GetMultibodySystem();
    ////auto sysCFD_type = parser.GetFluidSystemType();

    // Extract information from parsed YAML files
    const std::string& model_name = parser.GetName();
    double time_end = parser.GetEndtime();
    double time_step = parser.GetTimestep();
    bool render = parser.Render();
    double render_fps = parser.GetRenderFPS();

    auto& parserMBS = parser.GetMbsParser();
    CameraVerticalDir camera_vertical = parser.GetCameraVerticalDir();
    const ChVector3d& camera_location = parser.GetCameraLocation();
    const ChVector3d& camera_target = parser.GetCameraTarget();
    bool output_MBS = parserMBS.Output() && !disable_output;
    double output_fps_MBS = parser.GetOutputFPS();

    auto& parserCFD = parser.GetCfdParser();
    bool output_CFD = parserCFD.Output() && !disable_output;
    double output_fps_CFD = parser.GetOutputFPS();

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;
    #if defined(CHRONO_VSG)
    if (render) {
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachSystem(sysMBS.get());
        visVSG->SetWindowTitle("YAML FSI model - " + model_name);
        visVSG->AddCamera(camera_location, camera_target);
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
        visVSG->SetCameraVertical(camera_vertical);
        visVSG->SetCameraAngleDeg(40.0);
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(-CH_PI_4, CH_PI_4);
        visVSG->EnableShadows(false);
        visVSG->SetAbsFrameScale(2.0);

        auto plugin = parserCFD.GetVisualizationPlugin();
        if (plugin)
            visVSG->AttachPlugin(plugin);

        visVSG->Initialize();
        vis = visVSG;
    }
    #else
    render = false;
    #endif

    // Create output directory
    if (output_MBS || output_CFD) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return false;
        }
        out_dir = out_dir + "/" + model_name;
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        parser.SetOutputDir(out_dir);
    }

    // Simulation loop
    double time = 0;
    int render_frame = 0;
    int output_frame_MBS = 0;
    int output_frame_CFD = 0;

    while (true) {
        if (render) {
            if (!vis->Run())
                break;
            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();
                render_frame++;
            }
        } else {
            std::cout << "\rt = " << time;
            if (time_end > 0 && time >= time_end)
                break;
        }

        if (output_MBS && time >= output_frame_MBS / output_fps_MBS) {
            parserMBS.SaveOutput(*sysMBS, output_frame_MBS);
            output_frame_MBS++;
        }

        if (output_CFD && time >= output_frame_CFD / output_fps_CFD) {
            parserCFD.SaveOutput(output_frame_CFD);
            output_frame_CFD++;
        }

        sysFSI->DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}