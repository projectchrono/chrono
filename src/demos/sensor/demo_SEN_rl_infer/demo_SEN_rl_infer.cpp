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
// Chrono demonstration of an inference driver using a trained neural network
// Uses TensorRT to load a .onnx file as the model
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

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
// #include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShape.h"

#include "InferenceDriver.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Update rate in Hz for each camera
int update_rate = 10;
int rl_update_rate = 50;

// Image width and height for each camera
unsigned int image_width = 1920;
unsigned int image_height = 1080;
unsigned int rl_image_width = 80;
unsigned int rl_image_height = 45;

// Camera's horizontal field of view
float fov = CH_C_PI / 3.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0;

// Exposure (in seconds) of each image
float exposure_time = 0;

// -----------------------------------------------------------------------------
// IMU parameters
// -----------------------------------------------------------------------------
// IMU update rate in Hz
int acc_update_rate = 100;

// IMU lag (in seconds) between sensing and when data becomes accessible
float acc_lag = 0;

// IMU collection time (in seconds) of each sample
float acc_collection_time = 0;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = DEFAULT;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, HULLS, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Type of tire model (RIGID, RIGID_MESH, TMEASY, PACEJKA, FIALA, PAC89, PAC02)
TireModelType tire_model = TireModelType::PAC02;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 250.0;  // size in X direction
double terrainWidth = 15.0;    // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;
bool contact_vis = false;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "hmmwv";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

// Save camera images
bool save = true;

// Render camera images
bool vis = false;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Create the hmmwv vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisCollisionType(chassis_collision_type);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());

    ChContactMaterialData minfo;
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
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV Demo");
    app.AddTypicalLights();
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ----------------------------------------
    // Create boxes HMMWV will attempt to avoid
    // ----------------------------------------
    for (int i = 0; i < 3; i++) {
        auto box = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 10, 1000, true, true);
        box->SetPos({25 + 25 * i, (((float)rand() / (float)RAND_MAX) - .5) * 10, 5.05});
        box->SetBodyFixed(true);

        // Add visual asset to be sensed by camera
        // Will be a solid blue color
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetAmbientColor({0, 0, 0});
        vis_mat->SetDiffuseColor({.2, .2, .9});
        vis_mat->SetSpecularColor({.9, .9, .9});
        box->GetVisualModel()->GetShapes()[0].first->AddMaterial(vis_mat);

        my_hmmwv.GetSystem()->Add(box);
    }

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

    // Initialize output file for driver inputs
    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // Set up vehicle output
    my_hmmwv.GetVehicle().SetChassisOutput(true);
    my_hmmwv.GetVehicle().SetSuspensionOutput(0, true);
    my_hmmwv.GetVehicle().SetSteeringOutput(0, true);
    my_hmmwv.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    my_hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------
    // Create the driver system
    // ------------------------

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_hmmwv.LogHardpointLocations();
    }

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << my_hmmwv.GetVehicle().GetVehicleMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    int render_frame = 0;
    double ch_time = 0;

    if (contact_vis) {
        app.SetSymbolscale(1e-4);
        app.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);
    }

    // create the sensor manager and a camera
    auto manager = chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());

    // set lights
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 1000);
    manager->scene->AddPointLight({-100, -100, 100}, {1, 1, 1}, 1000);

    // --------------------------------------------------------------------------
    // Create a camera to be visualized or saved and add it to the sensor manager
    // --------------------------------------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(.2, {0, 1, 0}));
    auto cam = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
        my_hmmwv.GetChassisBody(),  // body camera is attached to
        update_rate,                // update rate in Hz
        offset_pose,                // offset pose
        image_width,                // image width
        image_height,               // image height
        fov                         // field of view
    );
    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // Visualize the image
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1920 / 2, 1080 / 2, "RL Demo"));

    // Save the third person view at the specified path
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>("SENSOR_OUTPUT/rl/cam/"));

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------------------------------------------------------------
    // Create the camera used for inference and add it to the sensor manager
    // ---------------------------------------------------------------------
    auto rl_offset_pose = chrono::ChFrame<double>({2.0, 0, .875}, Q_from_AngAxis(0, {1, 0, 0}));
    auto rl_cam = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
        my_hmmwv.GetChassisBody(),  // body camera is attached to
        rl_update_rate,             // update rate in Hz
        rl_offset_pose,             // offset pose
        rl_image_width,             // image width
        rl_image_height,            // image height
        fov                         // horizontal field of view of the camera
    );
    rl_cam->SetName("Camera Sensor");
    rl_cam->SetLag(lag);
    rl_cam->SetCollectionWindow(exposure_time);

    // Visualize the image
    if (vis) {
        int f = 4;
        rl_cam->PushFilter(
            chrono_types::make_shared<ChFilterVisualize>(rl_image_width * f, rl_image_height * f, "Inference Camera"));
    }
    // Allows the inference driver access to the RGBA buffer
    rl_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    // add sensor to the manager
    manager->AddSensor(rl_cam);

    // ------------------------------------------------------------------
    // Create the imu used for inference and add it to the sensor manager
    // ------------------------------------------------------------------
    auto acc_noise_none = chrono_types::make_shared<ChNoiseNone>();
    auto acc_offset_pose = chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0}));
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(
        my_hmmwv.GetChassisBody(),  // body to which the acc is attached
        acc_update_rate,            // update rate
        acc_offset_pose,            // offset pose from body
        acc_noise_none);            // acc noise model
    acc->SetName("Accelerometer");
    acc->SetLag(acc_lag);
    acc->SetCollectionWindow(acc_collection_time);

    // Add a filter to access the acc data
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());

    // Add the acc sensor to the sensor manager
    manager->AddSensor(acc);

    // -------------------------------------------------------------
    // Create the buffer pointers to be used in the inference driver
    // -------------------------------------------------------------
    // RGBA input to the model
    UserRGBA8BufferPtr rl_image_data = rl_cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    rl_image_data->Width = rl_image_width;
    rl_image_data->Height = rl_image_height;

    // Any extra data that should be passed to the network
    // In this example: [lateral acceleration in the x direction, speed]
    std::vector<float> extra_data = std::vector<float>(2);

    // ---------------------------
    // Create the inference driver
    // ---------------------------
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = .5;   // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    InferenceDriver driver(my_hmmwv.GetVehicle(),  //
                           GetChronoDataFile("sensor/neural_nets/rlmultisense.onnx"), rl_image_data, &extra_data);
    driver.SetDeltas(step_size / steering_time, step_size / throttle_time, step_size / braking_time);
    driver.Initialize();

    std::cout << "Setup complete." << '\n';
    bool done = false;              // Has the vehicle reached the goal
    double last_update = -1;        // Used for timing
    double control_frequency = 10;  // Frequency of inference in Hz

    // Simulation loop
    while (app.GetDevice()->run() && !done) {
        // Check if vehicle has reached goal or if the sim has failed
        if (my_hmmwv.GetChassisBody()->GetPos().x() > terrainLength / 2.0 ||
            abs(my_hmmwv.GetChassisBody()->GetPos().y()) > terrainWidth / 2.0)
            done = true;

        // Update the image data, if images are ready
        auto temp_image_data = rl_cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (temp_image_data->Buffer)
            rl_image_data->Buffer = temp_image_data->Buffer;

        // Update the extra data, if imu info is available
        auto acc_data = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        if (acc_data->Buffer) {
            extra_data[0] = acc_data->Buffer[0].X;
            extra_data[1] = my_hmmwv.GetChassisBody()->GetRot().RotateBack(my_hmmwv.GetChassisBody()->GetPos_dt()).x();
        }

        // End simulation
        if (ch_time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene();
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << ch_time << "\n\n";
            my_hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }

        // Collect output data from modules (for inter-module communication)
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << ch_time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking
                       << std::endl;
        }

        // Synchronize at the prescribed control frequency
        if (ch_time - last_update >= 1 / control_frequency) {
            driver.Synchronize(ch_time);
            last_update = ch_time;
        }

        // Update modules (process inputs from other modules)
        terrain.Synchronize(ch_time);
        my_hmmwv.Synchronize(ch_time, driver_inputs, terrain);
        app.Synchronize("RL Inference", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Get the current time of the simulation
        ch_time = (float)my_hmmwv.GetSystem()->GetChTime();

        // Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}
