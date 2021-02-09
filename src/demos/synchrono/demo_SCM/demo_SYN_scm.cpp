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
// Demo code illustrating synchronization of the SCM semi-empirical model for
// deformable soil
//
// See also in chrono_vehicle:
// - demo_VEH_DeformableSoil
// - demo_VEH_DeformableSoilAndTire
// - demo_VEH_HMMWV_DefSoil
//
// =============================================================================

#include <chrono>

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynSCMTerrainAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
using namespace chrono::sensor;
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Better conserve mass by displacing soil to the sides of a rut
const bool bulldozing = true;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step size
double step_size = 3e-3;

// Simulation end time
double end_time = 1000;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Time interval between two render frames
double render_step_size = 1.0 / 100;  // FPS = 50

// How often SynChrono state messages are interchanged
double heartbeat = 1e-2;  // 100[Hz]

// Forward declares for straight forward helper functions
void LogCopyright(bool show);
void AddCommandLineOptions(ChCLI& cli);
std::string StringFromContactMethod(ChContactMethod contact);
ChContactMethod ContactMethodFromString(std::string str);

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
    contact_method = ContactMethodFromString(cli.GetAsType<std::string>("contact_method"));

    const double size_x = cli.GetAsType<double>("sizeX");
    const double size_y = cli.GetAsType<double>("sizeY");
    const double cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    const double cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    const double dpu = cli.GetAsType<double>("dpu");
    const int cam_res_width = cli.GetAsType<std::vector<int>>("res")[0];
    const int cam_res_height = cli.GetAsType<std::vector<int>>("res")[1];
    const bool use_scm = cli.Matches<std::string>("terrain_type", "SCM");

    // Change SynChronoManager settings
    syn_manager.SetHeartbeat(heartbeat);

    // ----------------------
    // Vehicle Initialization
    // ----------------------
    // Calculate initial position and paths for each vehicle
    // Use up more of the mesh by not placing vehicles in the middle
    ChVector<> offset(-size_x / 2 + 5, 0, 0);

    ChVector<> initLoc;
    ChQuaternion<> initRot;
    std::vector<ChVector<>> curve_pts;
    if (node_id % 2 == 0) {
        // Start even vehicles in a row on the south side, driving north
        initLoc = offset + ChVector<>(0, 2.0 * (node_id + 1), 0.5);
        initRot = Q_from_AngZ(0);
        curve_pts = {initLoc, initLoc + ChVector<>(100, 0, 0)};
    } else {
        // Start odd vehicles staggered going up the west edge, driving east
        initLoc = offset + ChVector<>(2.0 * (node_id - 1), -5.0 - 2.0 * (node_id - 1), 0.5);
        initRot = Q_from_AngZ(CH_C_PI / 2);
        curve_pts = {initLoc, initLoc + ChVector<>(0, 100, 0)};
    }

    // Create the HMMWV
    HMMWV_Full hmmwv;
    hmmwv.SetContactMethod(contact_method);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    hmmwv.SetPowertrainType(PowertrainModelType::SHAFTS);
    hmmwv.SetDriveType(DrivelineType::AWD);
    hmmwv.SetTireType(use_scm ? TireModelType::RIGID : TireModelType::TMEASY);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::MESH);
    hmmwv.SetSteeringVisualizationType(VisualizationType::NONE);
    hmmwv.SetWheelVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Solver settings.
    hmmwv.GetSystem()->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));
    hmmwv.GetSystem()->SetSolverMaxIterations(50);

    // Add vehicle as an agent
    auto vehicle_agent = chrono_types::make_shared<SynWheeledVehicleAgent>(&hmmwv.GetVehicle());
    vehicle_agent->SetZombieVisualizationFiles("hmmwv/hmmwv_chassis.obj", "hmmwv/hmmwv_rim.obj",
                                               "hmmwv/hmmwv_tire_left.obj");
    vehicle_agent->SetNumWheels(4);
    syn_manager.AddAgent(vehicle_agent);

    // ----------------------
    // Terrain specific setup
    // ----------------------
    std::shared_ptr<ChTerrain> terrain;
    if (use_scm) {
        auto scm = chrono_types::make_shared<SCMDeformableTerrain>(hmmwv.GetSystem());

        // Configure the SCM terrain
        if (bulldozing) {
            scm->EnableBulldozing(bulldozing);
            scm->SetBulldozingParameters(
                55,   // angle of friction for erosion of displaced material at the border of the rut
                1,    // displaced material vs downward pressed material.
                5,    // number of erosion refinements per timestep
                10);  // number of concentric vertex selections subject to erosion
        }

        // Only relevant for Irrlicht visualization, gives some nice colors
        scm->SetPlotType(SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);
        scm->GetMesh()->SetWireframe(true);

        // The physics do not change when you add a moving patch, you just make it much easier for the SCM
        // implementation to do its job by restricting where it has to look for contacts
        scm->AddMovingPatch(hmmwv.GetVehicle().GetChassisBody(), ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));

        scm->Initialize(size_x, size_y, 1. / dpu);

        // Create an SCMTerrainAgent and add it to the SynChrono manager
        auto terrain_agent = chrono_types::make_shared<SynSCMTerrainAgent>(scm);
        syn_manager.AddAgent(terrain_agent);

        // Choice of soft parameters is arbitrary
        SCMParameters params;
        params.InitializeParametersAsSoft();
        terrain_agent->SetSoilParametersFromStruct(&params);

        // Add texture for the terrain
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetSpecularColor({.1f, .1f, .1f});
        vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
        scm->GetMesh()->material_list.push_back(vis_mat);

        terrain = scm;
    } else {
        MaterialInfo minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto patch_mat = minfo.CreateMaterial(contact_method);

        auto rigid = chrono_types::make_shared<RigidTerrain>(hmmwv.GetSystem());
        auto patch = rigid->AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), size_x, size_y);
        rigid->Initialize();

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

        terrain = rigid;
    }

    // Create the driver for the vehicle

    // What we defined earlier, a straight line
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "Box path", 10);

    // Reasonable defaults for the underlying PID
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver.GetSteeringController().SetLookAheadDistance(5);

    // Initialzie the SynChrono manager
    syn_manager.Initialize(hmmwv.GetSystem());

    // -------------
    // Visualization
    // -------------
#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleIrrApp> app;
    if (cli.HasValueInVector<int>("irr", node_id)) {
        app = chrono_types::make_shared<ChWheeledVehicleIrrApp>(&hmmwv.GetVehicle(), L"SynChrono SCM Demo");
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
    ChSensorManager sensor_manager(hmmwv.GetSystem());
    if (cli.HasValueInVector<int>("sens", node_id)) {
        // Give the camera a fixed place to live
        auto origin = chrono_types::make_shared<ChBody>();
        origin->SetBodyFixed(true);
        hmmwv.GetSystem()->AddBody(origin);

        // Happens to be a reasonable-looking height
        ChVector<> camera_loc(cam_x, cam_y, 25);

        // Rotations to get a nice angle
        ChQuaternion<> rotation = QUNIT;
        const bool USE_ISO_VIEW = true;
        if (USE_ISO_VIEW) {
            ChQuaternion<> qA = Q_from_AngAxis(35 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        } else {
            // Top down view
            ChQuaternion<> qA = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(180 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        }

        auto overhead_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
            origin,                                         // body camera is attached to
            30.0f,                                          // update rate in Hz
            chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
            cam_res_width,                                  // image width
            cam_res_height,                                 // image height
            (float)CH_C_PI / 3                              // FOV
        );

        overhead_camera->SetName("Overhead Cam");
        overhead_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        // Do we draw a window on the screen?
        if (cli.GetAsType<bool>("sens_vis"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height));

        // Do we save images to disc?
        std::string file_path = std::string("SENSOR_OUTPUT/scm") + std::to_string(node_id) + std::string("/");
        if (cli.GetAsType<bool>("sens_save"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));

        sensor_manager.AddSensor(overhead_camera);
    }
#endif

    // ---------------
    // Simulation loop
    // ---------------
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;

    ChRealtimeStepTimer realtime_timer;
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    while (true) {
        double time = hmmwv.GetSystem()->GetChTime();

        // End simulation
        if (time >= end_time         // ran out of time
            || !syn_manager.IsOk())  // SynChronoManager has shutdown
            break;

#ifdef CHRONO_IRRLICHT
        if (app && !app->GetDevice()->run())  //  Irrlicht visualization has stopped
            break;

        // Render scene
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
        terrain->Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, *terrain);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Synchronize("", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain->Advance(step_size);
        hmmwv.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        if (app)
            app->Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
#endif

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        // realtime_timer.Spin(step_size);

        if ((int)step_number % 100 == 0 && node_id == 1) {
            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            SynLog() << (time_span.count() / 1e3) / time << "\n";
        }
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
    cli.AddOption<std::string>("Simulation", "c,contact_method", "Contact Method",
                               StringFromContactMethod(contact_method), "NSC/SMC");

    // SCM specific options
    cli.AddOption<double>("Demo", "d,dpu", "Divisions per unit", "20");
    cli.AddOption<std::string>("Demo", "t,terrain_type", "Terrain Type", "Rigid", "Rigid,SCM");

    // Visualization is the only reason you should be shy about terrain size. The implementation can easily handle a
    // practically infinite terrain (provided you don't need to visualize it)
    cli.AddOption<double>("Demo", "x,sizeX", "Size in the X", "100");
    cli.AddOption<double>("Demo", "y,sizeY", "Size in the Y", "50");

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "i,irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "-15,-25", "X,Y");
}

ChContactMethod ContactMethodFromString(std::string str) {
    if (str == "SMC")
        return ChContactMethod::SMC;
    if (str == "NSC")
        return ChContactMethod::NSC;
    throw ChException(str + " is not a valid ChContactMethod (SMC or NSC)");
}

std::string StringFromContactMethod(ChContactMethod contact) {
    switch (contact) {
        case ChContactMethod::NSC:
            return "NSC";
        case ChContactMethod::SMC:
            return "SMC";
        default:
            throw ChException("ChContactMethod improperly enumerated in StringFromContactMethod");
    }
}