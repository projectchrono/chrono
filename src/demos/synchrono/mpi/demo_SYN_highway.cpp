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
// Authors: Yan Xiao
// =============================================================================
//
// Demo of several vehicles driving on a highway, vehicles follow paths to stay
// in their lanes and one vehicle changes lanes.
//
// =============================================================================

#include <chrono>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
using namespace chrono::sensor;
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;
using namespace chrono::synchrono;

// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step sizes
double step_size = 3e-3;

// Simulation end time
double end_time = 1000;

// When node_id 0 should change lanes [s]
double lane_change_time = 6;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);
ChCoordsys<> GetVehicleConfig(int node_id,
                              std::string& vehicle_filename,
                              std::string& powertrain_filename,
                              std::string& tire_filename,
                              std::string& zombie_filename,
                              double& cam_distance);

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
    // Get the vehicle JSON filenames and initial locations
    std::string vehicle_filename, powertrain_filename, tire_filename, zombie_filename;
    double cam_distance;
    auto initPos = GetVehicleConfig(node_id,              //
                                    vehicle_filename,     //
                                    powertrain_filename,  //
                                    tire_filename,        //
                                    zombie_filename,      //
                                    cam_distance);        //

    // Create the vehicle, set parameters, and initialize
    WheeledVehicle vehicle(vehicle_filename, contact_method);
    vehicle.Initialize(initPos);
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(powertrain_filename);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(tire_filename);
            vehicle.InitializeTire(tire, wheel, tire_vis_type);
        }
    }

    // Add vehicle as an agent and initialize SynChronoManager
    syn_manager.AddAgent(chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle, zombie_filename));
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
    auto patch =
        terrain.AddPatch(patch_mat, CSYSNORM, synchrono::GetDataFile("meshes/Highway_col.obj"), true, 0.01, false);

    auto vis_mesh =
        ChTriangleMeshConnected::CreateFromWavefrontFile(synchrono::GetDataFile("meshes/Highway_vis.obj"), true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(vis_mesh);
    trimesh_shape->SetMutable(false);
    patch->GetGroundBody()->AddVisualShape(trimesh_shape);
    terrain.Initialize();

    // ----------
    // Controller
    // ----------
    auto loc = vehicle.GetPos();

    // Make node_ids >= 4 start the other direction on the highway, going in a straight line
    auto curve_pts = node_id < 4 ? std::vector<ChVector<>>({loc, loc + ChVector<>(0, 140, 0)})   //
                                 : std::vector<ChVector<>>({loc, loc - ChVector<>(0, 140, 0)});  //
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);

    // Make node_id 2 slower so the passing looks nice, other parameters are normal car-following settings
    double target_speed = node_id == 2 ? 6 : 10;
    double target_following_time = 1.2;
    double target_min_distance = 10;
    double current_distance = 100;
    bool is_path_closed = false;

    std::shared_ptr<ChDriver> driver;
    if (node_id != 0) {
        // These vehicles just follow a single path
        auto acc_driver = chrono_types::make_shared<ChPathFollowerACCDriver>(vehicle, path, "Highway", target_speed,
                                                                             target_following_time, target_min_distance,
                                                                             current_distance, is_path_closed);
        acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        acc_driver->GetSteeringController().SetLookAheadDistance(5);

        driver = acc_driver;
    } else {
        // If we are node_id 0 we know about a second lane's worth of points and will change lanes to it
        std::vector<ChVector<>> curve_pts2 = {ChVector<>({6.4, -70, 0.2}), ChVector<>(6.4, 70, 0.2)};
        auto path2 = chrono_types::make_shared<ChBezierCurve>(curve_pts2);

        std::vector<std::pair<std::shared_ptr<ChBezierCurve>, bool>> path_pairs;
        path_pairs.push_back({path, false});
        path_pairs.push_back({path2, false});

        // Different driver (ChMultiPathFollowerACCDriver) needed in order to change lanes
        auto acc_driver = chrono_types::make_shared<ChMultiPathFollowerACCDriver>(
            vehicle, path_pairs, "Highway", target_speed, target_following_time, target_min_distance, current_distance);

        acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        acc_driver->GetSteeringController().SetLookAheadDistance(5);

        driver = acc_driver;
    }

    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> app;
    if (cli.HasValueInVector<int>("irr", node_id)) {
        app = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        app->SetWindowTitle("SynChrono Vehicle Demo");
        app->SetChaseCamera(trackPoint, 6.0, 0.5);
        app->Initialize();
        app->AddTypicalLights();
        app->AttachVehicle(&vehicle);
    }

#ifdef CHRONO_SENSOR
    const double cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    const double cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    const int cam_res_width = cli.GetAsType<std::vector<int>>("res")[0];
    const int cam_res_height = cli.GetAsType<std::vector<int>>("res")[1];

    const bool use_sensor_vis = cli.HasValueInVector<int>("sens", node_id);

    std::shared_ptr<ChCameraSensor> intersection_camera;
    ChVector<double> camera_loc(cam_x, cam_y, 15);

    ChSensorManager sensor_manager(vehicle.GetSystem());
    if (use_sensor_vis) {
        sensor_manager.scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 6000);
        sensor_manager.scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 6000);

        auto origin = chrono_types::make_shared<ChBody>();
        origin->SetBodyFixed(true);
        vehicle.GetSystem()->AddBody(origin);

        // Rotations to get a nice angle
        ChQuaternion<> rotation = QUNIT;
        ChQuaternion<> qA = Q_from_AngAxis(30 * CH_C_DEG_TO_RAD, VECT_Y);
        ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
        rotation = rotation >> qA >> qB;

        intersection_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
            origin,                                         // body camera is attached to
            30.0f,                                          // update rate in Hz
            chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
            cam_res_width,                                  // image width
            cam_res_height,                                 // image height
            (float)CH_C_PI / 3,                             // FOV
            1,                                              // samples per pixel for antialiasing
            CameraLensModelType::PINHOLE);                                       // camera type

        intersection_camera->SetName("Intersection Cam");
        intersection_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        if (cli.GetAsType<bool>("sens_vis"))
            intersection_camera->PushFilter(
                chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height, "Main Camera"));

        std::string file_path = std::string("SENSOR_OUTPUT/highway") + std::to_string(node_id) + std::string("/");
        if (cli.GetAsType<bool>("sens_save"))
            intersection_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));

        sensor_manager.AddSensor(intersection_camera);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    while (true) {
        double time = vehicle.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time                       // ran out of time
            || !syn_manager.IsOk()                 // SynChronoManager has shutdown
            || (app && !app->GetDevice()->run()))  //  Irrlicht visualization has stopped
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0 && app) {
            app->BeginScene();
            app->Render();
            app->EndScene();
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver->GetInputs();

        // Update modules (process inputs from other modules)
        syn_manager.Synchronize(time);  // Synchronize between nodes
        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        if (app)
            app->Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        if (app)
            app->Advance(step_size);

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
        if (use_sensor_vis) {
            // Move the camera parallel to the vehicle as it goes down the road
            camera_loc += ChVector<>(0, step_size * 7, 0);
            ChQuaternion<> rotation = QUNIT;
            ChQuaternion<> qA = Q_from_AngAxis(30 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
            intersection_camera->SetOffsetPose(chrono::ChFrame<double>(camera_loc, rotation));
        }
#endif  // SENSOR

        // Increment frame number
        step_number++;

        if (node_id == 0 && std::abs(vehicle.GetSystem()->GetChTime() - lane_change_time) < 1e-2)
            std::dynamic_pointer_cast<ChMultiPathFollowerACCDriver>(driver)->changePath(1);
    }
    syn_manager.QuitSimulation();

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

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "20,-85", "X,Y");
}

ChCoordsys<> GetVehicleConfig(int node_id,
                              std::string& vehicle_filename,
                              std::string& powertrain_filename,
                              std::string& tire_filename,
                              std::string& zombie_filename,
                              double& cam_distance) {
    ChVector<> initLoc;
    ChQuaternion<> initRot;
    switch (node_id) {
        case 0:
            vehicle_filename = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
            powertrain_filename = vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
            tire_filename = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
            zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");
            initLoc = ChVector<>(2.8, -70, 0.2);
            initRot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            cam_distance = 6.0;
            break;
        case 1:
            vehicle_filename = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
            powertrain_filename = vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
            tire_filename = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
            zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");
            initLoc = ChVector<>(2.8, -40, 0.2);
            initRot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            cam_distance = 6.0;
            break;
        case 2:
            vehicle_filename = vehicle::GetDataFile("citybus/vehicle/CityBus_Vehicle.json");
            powertrain_filename = vehicle::GetDataFile("citybus/powertrain/CityBus_SimpleMapPowertrain.json");
            tire_filename = vehicle::GetDataFile("citybus/tire/CityBus_TMeasyTire.json");
            zombie_filename = synchrono::GetDataFile("vehicle/CityBus.json");
            initLoc = ChVector<>(6.4, 0, 0.2);
            initRot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            cam_distance = 14.0;
            break;
        default:
            if (node_id % 2 == 0) {
                vehicle_filename = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
                powertrain_filename = vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
                tire_filename = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
                zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");
                initLoc = ChVector<>(-2.8, 70.0 - (node_id - 4.0) * 30, 0.2);
                cam_distance = 6.0;
            } else {
                vehicle_filename = vehicle::GetDataFile("citybus/vehicle/CityBus_Vehicle.json");
                powertrain_filename = vehicle::GetDataFile("citybus/powertrain/CityBus_SimpleMapPowertrain.json");
                tire_filename = vehicle::GetDataFile("citybus/tire/CityBus_TMeasyTire.json");
                zombie_filename = synchrono::GetDataFile("vehicle/CityBus.json");
                initLoc = ChVector<>(-6.4, 70.0 - (node_id - 4.0) * 30, 0.2);
                cam_distance = 14.0;
            }
            initRot = Q_from_AngZ(-90 * CH_C_DEG_TO_RAD);
    }

    return ChCoordsys<>(initLoc, initRot);
}