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
// Authors: 肖言 (Yan Xiao)
// =============================================================================
//
// Demo of several vehicles driving on a highway, vehicles follow paths to stay
// in their lanes and one vehicle changes lanes.
//
// =============================================================================

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/brain/SynACCBrain.h"
#include "chrono_synchrono/brain/driver/SynMultipathDriver.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
using namespace chrono::sensor;
#endif

#include <chrono>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;
using namespace chrono::synchrono;

// =============================================================================
const ChContactMethod contact_method = ChContactMethod::NSC;

// [s]
double end_time = 1000;
double step_size = 3e-3;

// When rank 0 should change lanes [s]
double lane_change_time = 6;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
float heartbeat = 1e-2;  // 100[Hz]

void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

std::shared_ptr<SynWheeledVehicle> InitializeVehicle(int rank);

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);

    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------
    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);

    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");

    const double cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    const double cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    const int cam_res_width = cli.GetAsType<std::vector<int>>("res")[0];
    const int cam_res_height = cli.GetAsType<std::vector<int>>("res")[1];

    const bool use_sensor_vis = cli.HasValueInVector<int>("sens", rank);
    const bool use_irrlicht_vis = !use_sensor_vis && cli.HasValueInVector<int>("irr", rank);

    mpi_manager.SetHeartbeat(heartbeat);
    mpi_manager.SetEndTime(end_time);

    // -------
    // Vehicle
    // -------
    std::shared_ptr<ChDriver> driver;

    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
    agent->SetVehicle(InitializeVehicle(rank));
    mpi_manager.AddAgent(agent, rank);

    // -------
    // Terrain
    // -------
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());
    auto patch =
        terrain->AddPatch(patch_mat, CSYSNORM, synchrono::GetDataFile("meshes/Highway_col.obj"), "", 0.01, false);

    auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    vis_mesh->LoadWavefrontMesh(synchrono::GetDataFile("meshes/Highway_vis.obj"), true, true);
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(vis_mesh);
    trimesh_shape->SetStatic(true);
    patch->GetGroundBody()->AddAsset(trimesh_shape);
    terrain->Initialize();

    agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

    // ----------
    // Controller
    // ----------
    auto loc = agent->GetChVehicle().GetVehiclePos();

    // Make ranks >= 4 start the other direction on the highway, going in a straight line
    auto curve_pts = rank < 4 ? std::vector<ChVector<>>({loc, loc + ChVector<>(0, 140, 0)})   //
                              : std::vector<ChVector<>>({loc, loc - ChVector<>(0, 140, 0)});  //
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);

    // Make rank 2 slower so the passing looks nice, other parameters are normal car-following settings
    double target_speed = rank == 2 ? 6 : 10;
    double target_following_time = 1.2;
    double target_min_distance = 10;
    double current_distance = 100;
    bool is_path_closed = false;

    if (rank != 0) {
        // These vehicles just follow a single path
        auto acc_driver = chrono_types::make_shared<ChPathFollowerACCDriver>(
            agent->GetChVehicle(), path, "Highway", target_speed, target_following_time, target_min_distance,
            current_distance, is_path_closed);
        acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        acc_driver->GetSteeringController().SetLookAheadDistance(5);

        driver = acc_driver;
    } else {
        // If we are rank 0 we know about a second lane's worth of points and will change lanes to it
        std::vector<ChVector<>> curve_pts2 = {ChVector<>({6.4, -70, 0.2}), ChVector<>(6.4, 70, 0.2)};
        auto path2 = chrono_types::make_shared<ChBezierCurve>(curve_pts2);

        std::vector<std::pair<std::shared_ptr<ChBezierCurve>, bool>> path_pairs;
        path_pairs.push_back({path, false});
        path_pairs.push_back({path2, false});

        // Different driver (ChMulPathFollowerACCDriver) needed in order to change lanes
        auto acc_driver = chrono_types::make_shared<ChMulPathFollowerACCDriver>(
            agent->GetChVehicle(), path_pairs, "Highway", target_speed, target_following_time, target_min_distance,
            current_distance);

        acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        acc_driver->GetSteeringController().SetLookAheadDistance(5);

        driver = acc_driver;
    }

    auto brain = chrono_types::make_shared<SynACCBrain>(rank, driver, agent->GetChVehicle());
    if (rank == 0)
        brain->setMultipath(true);
    agent->SetBrain(brain);

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->SetVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
    if (use_irrlicht_vis) {
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver);
        irr_vis->SetRenderStepSize(render_step_size);
        irr_vis->SetStepSize(step_size);
        irr_vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
        vis_manager->AddVisualization(irr_vis);
    }
#endif

#ifdef CHRONO_SENSOR
    std::shared_ptr<ChCameraSensor> intersection_camera;
    ChVector<double> camera_loc(cam_x, cam_y, 15);
    if (use_sensor_vis) {
        auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();

        auto manager = chrono_types::make_shared<ChSensorManager>(agent->GetSystem());
        manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 6000);
        manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 6000);
        sen_vis->SetSensorManager(manager);

        auto origin = chrono_types::make_shared<ChBody>();
        origin->SetBodyFixed(true);
        agent->GetSystem()->AddBody(origin);

        // Rotations to get a nice angle
        ChQuaternion<> rotation = QUNIT;
        ChQuaternion<> qA = Q_from_AngAxis(30 * CH_C_DEG_TO_RAD, VECT_Y);
        ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
        rotation = rotation >> qA >> qB;

        intersection_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
            origin,                                         // body camera is attached to
            30,                                             // update rate in Hz
            chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
            cam_res_width,                                  // image width
            cam_res_height,                                 // image height
            CH_C_PI / 3,                                    // FOV
            1,                                              // samples per pixel for antialiasing
            PINHOLE);                                       // camera type

        intersection_camera->SetName("Intersection Cam");
        intersection_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        if (cli.GetAsType<bool>("sens_vis"))
            intersection_camera->PushFilter(
                chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height, "Main Camera"));

        std::string path = std::string("SENSOR_OUTPUT/highway") + std::to_string(rank) + std::string("/");
        if (cli.GetAsType<bool>("sens_save"))
            intersection_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(path));

        sen_vis->SetSensor(intersection_camera);
        vis_manager->AddVisualization(sen_vis);
    }
#endif

    mpi_manager.Barrier();
    mpi_manager.Initialize();

    int step_number = 0;

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance(heartbeat * step_number++);
        mpi_manager.Synchronize();

#ifdef CHRONO_SENSOR
        if (use_sensor_vis) {
            // Move the camera parallel to the vehicle as it goes down the road
            camera_loc += ChVector<>(0, heartbeat * 7, 0);
            ChQuaternion<> rotation = QUNIT;
            ChQuaternion<> qA = Q_from_AngAxis(30 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
            intersection_camera->SetOffsetPose(chrono::ChFrame<double>(camera_loc, rotation));
        }
#endif  // SENSOR

        if (rank == 0 && std::abs(agent->GetSystem()->GetChTime() - lane_change_time) < 1e-2)
            std::dynamic_pointer_cast<ChMulPathFollowerACCDriver>(driver)->changePath(1);

        mpi_manager.Update();
    }

    return 0;
}

std::shared_ptr<SynWheeledVehicle> InitializeVehicle(int rank) {
    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    std::string filename;
    switch (rank) {
        case 0:
            filename = "vehicle/Sedan.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(2.8, -70, 0.2);
            break;
        case 1:
            filename = "vehicle/Sedan.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(2.8, -40, 0.2);
            break;
        case 2:

            filename = "vehicle/CityBus.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(6.4, 0, 0.2);
            break;
        default:
            if (rank % 2 == 0) {
                filename = "vehicle/Sedan.json";
                init_loc = ChVector<>(-2.8, 70.0 - (rank - 4.0) * 30, 0.2);
            } else {
                filename = "vehicle/CityBus.json";
                init_loc = ChVector<>(-6.4, 70.0 - (rank - 4.0) * 30, 0.2);
            }
            init_rot = Q_from_AngZ(-90 * CH_C_DEG_TO_RAD);
    }
    ChCoordsys<> init_pos(init_loc, init_rot);
    auto vehicle =
        chrono_types::make_shared<SynWheeledVehicle>(init_pos, synchrono::GetDataFile(filename), contact_method);

    return vehicle;
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "20,-85", "X,Y");
}