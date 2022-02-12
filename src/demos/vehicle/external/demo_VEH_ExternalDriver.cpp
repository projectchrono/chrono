// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Aaron Young
// =============================================================================
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChSocket.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChExternalDriver.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
// #define USE_IRRLICHT
#endif

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "converters/ChVehicle_DataGeneratorFunctor.h"
#include "converters/ChSystem_DataGeneratorFunctor.h"
#include "converters/ChDriverInputs_DataParserFunctor.h"

#ifdef CHRONO_SENSOR
    #include "chrono_sensor/ChSensorManager.h"
    #include "chrono_sensor/sensors/ChCameraSensor.h"
    #include "chrono_sensor/filters/ChFilterAccess.h"
    #include "chrono_sensor/filters/ChFilterVisualize.h"
    #include "chrono_sensor/filters/ChFilterSave.h"

    #include "converters/ChCameraSensor_DataGeneratorFunctor.h"
#endif
#include "chrono_thirdparty/stb/stb_image_write.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/error/error.h"
#include "chrono_thirdparty/rapidjson/error/en.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using namespace rapidjson;

#ifdef CHRONO_SENSOR
using namespace chrono::sensor;
#endif

// Problem parameters

// Contact method type
ChContactMethod contact_method = ChContactMethod::SMC;

// Type of tire model (RIGID, LUGRE, FIALA, PACEJKA, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Type of powertrain model (SHAFTS or SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::RWD;

// Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
// Note: Compliant steering requires higher PID gains.
SteeringTypeWV steering_type = SteeringTypeWV::PITMAN_ARM;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 300.0;   // size in Y direction

// Point on chassis tracked by the chase camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation step size
double step_size = 2e-3;
double tire_step_size = 1e-3;

// Simulation end time
double t_end = 100;

// Render FPS
double fps = 60;

// Debug logging
bool debug_output = false;
double debug_fps = 10;

// Output directories
const std::string out_dir = GetChronoOutputPath() + "STEERING_CONTROLLER";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = false;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = false;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2022 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ---------
    // CLI setup
    // ---------

    ChCLI cli(argv[0]);

    cli.AddOption<bool>("Demo", "client",
                        "If set, an example external driver will be used. Otherwise, it will be a server.", "false");
    cli.AddOption<int>("Demo", "p,port", "The port to communicate on. Defaults to 50000.", "50000");
    cli.AddOption<std::string>("Demo", "i,address",
                               "(client only) Address to connect to. Defaults to \"127.0.0.1\", i.e. localhost.",
                               "127.0.0.1");

    cli.AddOption<bool>("Sensor", "camera-vis", "Visualize camera sensor image.", "false");
    cli.AddOption<bool>("Sensor", "camera-save", "Save camera sensor image.", "false");

    if (!cli.Parse(argc, argv, true, true))
        return 0;

    const bool USE_CLIENT = cli.GetAsType<bool>("client");
    const int PORT = cli.GetAsType<int>("port");
    const std::string ADDRESS = cli.GetAsType<std::string>("address");
    const bool CAMERA_VIS = cli.GetAsType<bool>("camera-vis");
    const bool CAMERA_SAVE = cli.GetAsType<bool>("camera-save");

    // -----------
    // Client code
    // -----------
    if (USE_CLIENT) {
        utils::ChSocketTCP client(PORT);
        std::string server = ADDRESS;
        client.connectToServer(server, utils::ADDRESS);

        while (true) {
            try {
                // -------
                // Receive
                // -------
                {
                    std::string message;
                    client.receiveMessage(message);
                    message += '\0';

                    // Parse the JSON string
                    Document d;
                    d.Parse(message.c_str());

                    for (auto& m : d.GetArray()) {
                        std::string type = m["type"].GetString();
                        if (type == "ChCameraSensor") {
                            auto data = m["data"].GetObject();

                            if (data.HasMember("image")) {
                                uint64_t total_len =
                                    data["width"].GetUint64() * data["height"].GetUint64() * data["size"].GetUint64();

                                const char* image_ptr = data["image"].GetString();
                                std::vector<uint8_t> image(image_ptr, image_ptr + total_len);

                                static int i = 0;
                                if (!stbi_write_png(
                                        std::string("DEMO_OUTPUT/EXTERNAL/test_" + std::to_string(i++) + ".png")
                                            .c_str(),
                                        data["width"].GetUint64(), data["height"].GetUint64(), data["size"].GetUint64(),
                                        image.data(), data["size"].GetUint64() * data["width"].GetUint64()))
                                    std::cerr << "Failed to write RGBA8 image!\n";
                            }
                        }
                    }
                }

                // ----
                // Send
                // ----
                {
                    std::string message =
                        "[{\"type\":\"ChDriverInputs\",\"data\":{\"throttle\":1.0,\"steering\":0.0,\"braking\":0.0}}]";
                    client.sendMessage(message);
                }

            } catch (utils::ChExceptionSocket& exception) {
                GetLog() << " ERROR with socket system: \n" << exception.what() << "\n";
            }
        }
        return 0;
    }

    // ------------------------------
    // Create the vehicle and terrain
    // ------------------------------

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(contact_method);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetSteeringType(steering_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.8f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
    patch->SetColor(ChColor(1, 1, 1));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    terrain.Initialize();

    // ------------------------
    // Create the driver system
    // ------------------------

    ChExternalDriver driver(my_hmmwv.GetVehicle(), PORT);
    driver.AddDataGenerator(chrono_types::make_shared<ChSystem_DataGeneratorFunctor>(my_hmmwv.GetSystem(), "/clock"));
    driver.AddDataGenerator(
        chrono_types::make_shared<ChVehicle_DataGeneratorFunctor>(my_hmmwv.GetVehicle(), "~/output/ChVehicle"), 100);
    driver.AddDataParser(chrono_types::make_shared<ChDriverInputs_DataParserFunctor>(driver));

    // ---------------------------------------------------------
    // Create the sensor system (if Chrono::Sensor is available)
    // ---------------------------------------------------------

#ifdef CHRONO_SENSOR
    auto manager = chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());
    manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    manager->scene->SetAmbientLight({.1, .1, .1});

    // Set the background to an environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);

    // Create a camera that's placed on the hood
    chrono::ChFrame<double> offset_pose({-8, 0, 3}, Q_from_AngAxis(.2, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(my_hmmwv.GetChassisBody(),  // body camera is attached to
                                                         30,                         // update rate in Hz
                                                         offset_pose,                // offset pose
                                                         280,                        // image width
                                                         120,                        // image height
                                                         CH_C_PI / 4);  // camera's horizontal field of view
    cam->SetName("Camera Sensor");
    if (CAMERA_VIS)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Third Person View"));
    if (CAMERA_SAVE)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "/SENSOR_OUTPUT/cam/"));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    // Add a message generator
    driver.AddDataGenerator(
        chrono_types::make_shared<ChCameraSensor_DataGeneratorFunctor>(cam, "~/output/ChCameraSensor/third_person"),
        30);
#endif

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

#ifdef USE_IRRLICHT
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"External Driver Demo",
                               irr::core::dimension2d<irr::u32>(800, 640));
    app.SetHUDLocation(500, 20);
    app.SetSkyBox();
    app.AddTypicalLogo();
    app.AddTypicalLights(irr::core::vector3df(-150.f, -150.f, 200.f), irr::core::vector3df(-150.f, 150.f, 200.f), 100,
                         100);
    app.AddTypicalLights(irr::core::vector3df(150.f, -150.f, 200.f), irr::core::vector3df(150.0f, 150.f, 200.f), 100,
                         100);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    // Finalize construction of visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();
#endif

    // -----------------
    // Initialize output
    // -----------------

    state_output = state_output || povray_output;

    if (state_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
    }

    utils::CSV_writer csv("\t");
    csv.stream().setf(std::ios::scientific | std::ios::showpos);
    csv.stream().precision(6);

    utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

    utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
    utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

    // ---------------
    // Simulation loop
    // ---------------

    // Driver location in vehicle local frame
    ChVector<> driver_pos = my_hmmwv.GetChassis()->GetLocalDriverCoordsys().pos;

    // Number of simulation steps between miscellaneous events
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);
    double debug_step_size = 1 / debug_fps;
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int sim_frame = 0;
    int render_frame = 0;
    double time = 0;

    ChRealtimeStepTimer realtime_timer;
#ifdef USE_IRRLICHT
    while (app.GetDevice()->run()) {
#else
    while (time < t_end) {
#endif
        // Extract system state
        time = my_hmmwv.GetSystem()->GetChTime();
        ChVector<> acc_CG = my_hmmwv.GetVehicle().GetChassisBody()->GetPos_dtdt();
        ChVector<> acc_driver = my_hmmwv.GetVehicle().GetVehiclePointAcceleration(driver_pos);
        double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x());
        double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y());
        double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x());
        double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y());

        // End simulation
        if (time >= t_end)
            break;

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Output POV-Ray data
        if (sim_frame % render_steps == 0) {
#ifdef USE_IRRLICHT
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(my_hmmwv.GetSystem(), filename);
            }
#endif

            if (state_output) {
                csv << time << driver_inputs.m_steering << driver_inputs.m_throttle << driver_inputs.m_braking;
                csv << my_hmmwv.GetVehicle().GetVehicleSpeed();
                csv << acc_CG.x() << fwd_acc_CG << acc_CG.y() << lat_acc_CG;
                csv << acc_driver.x() << fwd_acc_driver << acc_driver.y() << lat_acc_driver;
                csv << std::endl;
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && sim_frame % debug_steps == 0) {
            GetLog() << "driver acceleration:  " << acc_driver.x() << "  " << acc_driver.y() << "  " << acc_driver.z()
                     << "\n";
            GetLog() << "CG acceleration:      " << acc_CG.x() << "  " << acc_CG.y() << "  " << acc_CG.z() << "\n";
            GetLog() << "\n";
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
#ifdef USE_IRRLICHT
        app.Synchronize("External Driver", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
#ifdef USE_IRRLICHT
        app.Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        // Update the sensor manager
        manager->Update();
#endif

        // Increment simulation frame number
        sim_frame++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    if (state_output)
        csv.write_to_file(out_dir + "/state.out");

    return 0;
}
