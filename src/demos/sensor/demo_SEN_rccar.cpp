// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Main driver function for the my_rccar full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/rccar/RCCar.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::rccar;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::PRIMITIVES;

// Collision type for chassis (PRIMITIVES, HULLS, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of tire model (RIGID)
TireModelType tire_model = TireModelType::RIGID;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.2);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "RCCar";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the Sedan vehicle, set parameters, and initialize
    RCCar my_rccar;
    my_rccar.SetContactMethod(contact_method);
    my_rccar.SetChassisCollisionType(chassis_collision_type);
    my_rccar.SetChassisFixed(false);
    my_rccar.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_rccar.SetTireType(tire_model);
    my_rccar.SetTireStepSize(tire_step_size);
    my_rccar.Initialize();

    VisualizationType tire_vis_type =
        (tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;

    my_rccar.SetChassisVisualizationType(chassis_vis_type);
    my_rccar.SetSuspensionVisualizationType(suspension_vis_type);
    my_rccar.SetSteeringVisualizationType(steering_vis_type);
    my_rccar.SetWheelVisualizationType(wheel_vis_type);
    my_rccar.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_rccar.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
                                     "test64", 128, 128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_rccar.GetVehicle(), L"RCCar Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 1.5, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
        driver.SetInputDataFile(driver_file);
        driver.SetInputMode(ChIrrGuiDriver::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_rccar.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << my_rccar.GetVehicle().GetVehicleMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    if (contact_vis) {
        app.SetSymbolscale(1e-4);
        app.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
    }

    // create the sensor manager and a camera
    auto manager = chrono_types::make_shared<ChSensorManager>(my_rccar.GetSystem());

    // set lights
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 500);

    // set environment map
    manager->scene->GetBackground().has_texture = true;
    manager->scene->GetBackground().env_tex = "sensor/textures/cloud_layers_8k.hdr";
    manager->scene->GetBackground().has_changed = true;

    auto cam1 = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
        my_rccar.GetChassisBody(),                                           // body camera is attached to
        30,                                                                  // update rate in Hz
        chrono::ChFrame<double>({-2, 0, 1}, Q_from_AngAxis(.3, {0, 1, 0})),  // offset pose
        1280,                                                                // image width
        720,                                                                 // image height
        CH_C_PI / 3);
    cam1->SetName("Camera Sensor");
    // cam1->SetLag(1 / 30.0);
    // cam1->SetCollectionWindow(0);
    cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));
    cam1->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam1);

    auto cam2 = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
        my_rccar.GetChassisBody(),                                          // body camera is attached to
        30,                                                                 // update rate in Hz
        chrono::ChFrame<double>({0, 0, .2}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose
        640,                                                                // image width
        480,                                                                // image height
        CH_C_PI / 3);
    cam2->SetName("Camera Sensor");
    // cam2->SetLag(1 / 30.0);
    // cam2->SetCollectionWindow(0);
    cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 480));
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam2);

    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        my_rccar.GetChassisBody(),                                          // body to which the IMU is attached
        10,                                                                 // update rate
        chrono::ChFrame<double>({0, 0, .1}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
        2000,                                                               // horizontal samples
        50,                                                                 // vertical samples/channels
        (float)2.0f * CH_C_PI,                                              // horizontal field of view
        (float)CH_C_PI / 6.f, -(float)CH_C_PI / 6.f,                        // vertical field of view
        100.0                                                               // max distance
    );
    lidar->SetName("Lidar Sensor");
    // lidar->SetLag(1 / 10.0);
    // lidar->SetCollectionWindow(0);
    // lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1000,100,"Lidar Data"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1.5, "Lidar Point Cloud"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>("output/ptcloud/sim1/lidar/"));
    manager->AddSensor(lidar);

    double render_time = 0;

    int num_camera_updates = 0;

    std::cout << "setup complete." << '\n';

    auto cam_buffer = cam1->GetMostRecentBuffer<UserRGBA8BufferPtr>();

    while (app.GetDevice()->run()) {
        double chtime = my_rccar.GetSystem()->GetChTime();

        // End simulation
        if (chtime >= t_end)
            break;

        cam_buffer = cam1->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (cam_buffer && cam_buffer->Buffer) {
            int h = cam_buffer->Height;
            int w = cam_buffer->Width;
            int c = 4;
            uint8_t* data = (uint8_t*)(cam_buffer->Buffer.get());
            int min = 1000;
            int zeros = 0;
            for (int i = 0; i < h; i++) {
                for (int j = 0; j < w; j++) {
                    for (int k = 0; k < 1; k++) {
                        if (data[i * w * c + j * c + k] == 0) {
                            zeros++;
                            // min = data[i * w * c + j * c + k];
                        }
                    }
                }
            }
            std::cout << "Min val: " << zeros / 1280.0 << std::endl;

            std::cout << "Readimg camera data...\n";
        }

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_rccar.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << chtime << "\n\n";
            my_rccar.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << chtime << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(chtime);
        terrain.Synchronize(chtime);
        my_rccar.Synchronize(chtime, driver_inputs, terrain);
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_rccar.Advance(step_size);
        app.Advance(step_size);

        std::chrono::high_resolution_clock::time_point r0 = std::chrono::high_resolution_clock::now();
        manager->Update();
        std::chrono::high_resolution_clock::time_point r1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> r_dur = std::chrono::duration_cast<std::chrono::duration<double>>(r1 - r0);
        render_time += r_dur.count();

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}
