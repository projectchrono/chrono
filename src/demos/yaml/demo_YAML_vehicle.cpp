// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Demo for vehicle simulation specified through YAML files.
// The vehicle is controlled interactively (keyboard).
//
// =============================================================================

#include "chrono_parsers/yaml/ChParserVehicleYAML.h"

#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/driver/ChInteractiveDriver.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Extract filenames from command-line arguments
    std::string sim_yaml_filename = GetChronoDataFile("yaml/vehicle/simulation_vehicle.yaml");
    std::string model_yaml_filename = GetChronoDataFile("yaml/vehicle/polaris.yaml");
    ////std::string model_yaml_filename = GetChronoDataFile("yaml/vehicle/marder.yaml");

    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "m,model_file", "vehicle model specification YAML file", model_yaml_filename);
    cli.AddOption<std::string>("", "s,sim_file", "simulation specification YAML file", sim_yaml_filename);

    if (!cli.Parse(argc, argv, true))
        return 1;

    if (argc == 1) {
        cli.Help();
        std::cout << "Using default YAML model and simulation specification" << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Vehicle model YAML file: " << model_yaml_filename << std::endl;
    std::cout << "Simulation YAML file:    " << sim_yaml_filename << std::endl;

    // Create the YAML parser object
    parsers::ChParserVehicleYAML parser(model_yaml_filename, sim_yaml_filename, true);

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
    bool output = parser.Output();
    double output_fps = parser.GetOutputFPS();

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
    if (render) {
        ChVisualSystem::Type vis_type;

#if defined(CHRONO_VSG)
        vis_type = ChVisualSystem::Type::VSG;
#elif defined(CHRONO_IRRLICHT)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#else
        std::cout << "No Chrono run-time visualization module enabled. Disabling visualization." << std::endl;
        render = false;
#endif

        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                std::shared_ptr<vehicle::ChVehicleVisualSystemIrrlicht> vis_irr;
                if (vehicle_type == parsers::ChParserVehicleYAML::VehicleType::WHEELED)
                    vis_irr = chrono_types::make_shared<vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
                else
                    vis_irr = chrono_types::make_shared<vehicle::ChTrackedVehicleVisualSystemIrrlicht>();
                vis_irr->SetWindowTitle("Vehicle YAML demo - " + model_name);
                vis_irr->SetCameraVertical(CameraVerticalDir::Z);
                vis_irr->SetChaseCamera(chassis_point, chase_distance, chase_height);
                vis_irr->Initialize();
                vis_irr->AddLightDirectional();
                vis_irr->AddSkyBox();
                vis_irr->AddLogo();
                vis_irr->AttachVehicle(vehicle.get());
                vis_irr->AttachDriver(driver.get());

                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
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
#endif
                break;
            }
        }
    }

        // Create output directory
    if (output) {
        std::string out_dir = GetChronoOutputPath() + "YAML_VEHICLE";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        out_dir = out_dir + "/" + model_name;
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        parser.SetOutputDir(out_dir);

        ////vehicle->SetSuspensionOutput(0, true);
        ////vehicle->SetSuspensionOutput(1, true);
        ////vehicle->SetOutput(ChOutput::Type::ASCII, ChOutput::Mode::FRAMES, out_dir, "output", 0.1);
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

    return 0;
}
