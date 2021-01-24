// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// A 3-lane grid of vehicles travels down a flat roadway. Only a block terrain
// is used, not a mesh. Very lightweight demo, useful for scaling analyses.
//
// =============================================================================

#include <chrono>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono::sensor;
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;

// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 3e-3;

// Simulation end time
double end_time = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100 [Hz]

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    // -----------------------
    // Create SynChronoManager
    // -----------------------
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    int node_id = communicator->GetRank();
    int num_nodes = communicator->GetNumRanks();
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Copyright
    LogCopyright(node_id == 0);

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, node_id == 0))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // -------
    // Vehicle
    // -------
    // Grid of vehicles
    int col = node_id % 3;
    int row = node_id / 3;

    // Box dimensions
    double length = 400;
    double width = 25;

    ChVector<double> base = ChVector<>({-length / 2 + 5, -width / 2 + 5, 1.0});
    ChVector<double> offset = ChVector<>({30.0 * row, 5.0 * col, 0});
    ChVector<double> init_loc = base + offset;

    ChQuaternion<> initRot = ChQuaternion<>({1, 0, 0, 0});

    // Create the vehicle, set parameters, and initialize
    WheeledVehicle vehicle(vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json"), contact_method);
    vehicle.Initialize(ChCoordsys<>(base + offset, initRot));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json"));
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json"));
            vehicle.InitializeTire(tire, wheel, tire_vis_type);
        }
    }

    // Add vehicle as an agent and initialize SynChronoManager
    auto agent =
        chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle, synchrono::GetDataFile("vehicle/Sedan.json"));
    syn_manager.AddAgent(agent);
    syn_manager.Initialize(vehicle.GetSystem());

    // -------
    // Terrain
    // -------
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    RigidTerrain terrain(vehicle.GetSystem());
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), length, width);
    terrain.Initialize();

    // Terrain visualization
    // For irrlicht
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    // For sensor
    auto patch_asset = patch->GetGroundBody()->GetAssets()[0];
    if (auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(patch_asset)) {
        auto box_texture = chrono_types::make_shared<ChVisualMaterial>();
        box_texture->SetKdTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
        // FresnelMax and SpecularColor should make it less shiny
        box_texture->SetFresnelMax(0.2f);
        box_texture->SetSpecularColor({0.2f, 0.2f, 0.2f});

        visual_asset->material_list.push_back(box_texture);
    }

    // ---------------------------
    // Controller for the vehicles
    // ---------------------------

    // Drive in a straight line
    std::vector<ChVector<>> curve_pts = {init_loc, init_loc + ChVector<>(1000, 0, 0)};
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);

    ChPathFollowerDriver driver(vehicle, path, "Box path", 10);

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver.GetSteeringController().SetLookAheadDistance(5);

#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleIrrApp> app;
    if (cli.HasValueInVector<int>("irr", node_id)) {
        app = chrono_types::make_shared<ChWheeledVehicleIrrApp>(&vehicle, L"SynChrono Vehicle Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                              130);
        app->SetChaseCamera(trackPoint, 6.0, 0.5);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();
    }
#endif

#ifdef CHRONO_SENSOR
    ChSensorManager sensor_manager(vehicle.GetSystem());
    if (cli.HasValueInVector<int>("sens", node_id)) {
        sensor_manager.scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 5000);
        sensor_manager.scene->AddPointLight({-100, -100, 100}, {1, 1, 1}, 5000);

        auto cam = chrono_types::make_shared<ChCameraSensor>(
            vehicle.GetChassisBody(),                                                      // body camera is attached to
            30.0f,                                                                         // update rate in Hz
            chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(CH_C_PI / 20, {0, 1, 0})),  // offset pose
            1280,                                                                          // image width
            720,                                                                           // image height
            (float)CH_C_PI / 3                                                             // FOV
        );

        if (cli.GetAsType<bool>("sens_vis"))
            cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));

        if (cli.GetAsType<bool>("sens_save")) {
            const std::string path = std::string("SENSOR_OUTPUT/platoon") + std::to_string(node_id) + std::string("/");
            cam->PushFilter(chrono_types::make_shared<ChFilterSave>(path));
        }

        sensor_manager.AddSensor(cam);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    while (true) {
        double time = vehicle.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time         // ran out of time
            || !syn_manager.IsOk())  // SynChronoManager has shutdown
            break;

#ifdef CHRONO_IRRLICHT
        if (app && !app->GetDevice()->run())  //  Irrlicht visualization has stopped
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0 && app) {
            app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app->DrawAll();
            app->EndScene();
        }
#endif

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        syn_manager.Synchronize(time);  // Synchronize between nodes
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Synchronize("", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);

#ifdef CHRONO_IRRLICHT
        if (app)
            app->Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
#endif

        // Increment frame number
        step_number++;
    }

    if (node_id == 0) {
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        SynLog() << "Total Wall Time: " << time_span.count() / 1e6 << "\n";
        SynLog() << "Fraction of real time: " << (time_span.count() / 1e6) / end_time << "\n";
        SynLog() << "Frequency of steps [Hz]: " << step_number / (time_span.count() / 1e6) << "\n";
        SynLog() << "Real time: " << (time_span.count() / 1e6) / end_time << "\n";
    }

    return 0;
}

void LogCopyright(bool show) {
    if (!show)
        return;

    SynLog() << "Copyright (c) 2020 projectchrono.org\n";
    SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "s,step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "e,end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "b,heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");
}
