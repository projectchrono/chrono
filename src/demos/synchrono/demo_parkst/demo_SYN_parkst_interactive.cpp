#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/framework/SynFramework.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/brain/driver/SynInteractiveDriver.h"
#include "chrono_synchrono/brain/SynEnvironmentBrain.h"

#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"

using namespace chrono::sensor;
#endif

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <chrono>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle::sedan;

ChCoordsys<> GetInitialState(int rank) {
    ChVector<> initLoc;
    ChQuaternion<> initRot;
    switch ((rank + 2) % 4) {
        case 0:
            // univ john inner
            // University facing west, just before Park. Second lane from the left.
            initLoc = ChVector<>({65.26 + (rank - 2) / 4 * 9, -2.82, 0.067});
            initRot = ChQuaternion<>({-0.00942852, 0.00497593, -0.00215422, -0.999941});
            break;
        case 1:
            // Grainger loop
            // Park St facing north, just before University. Left turn lane.
            initLoc = ChVector<>({2, -40.0 - (rank - 3) / 4 * 9, 0.5});
            initRot = ChQuaternion<>({-0.7071068, 0, 0, -0.7071068});
            break;
        case 2:
            // Park st. straight
            // Park St facing south, just before University. Right turn lane.
            initLoc = ChVector<>({-2.63, 69.0 + (rank - 4) / 3 * 9, 0.72});
            initRot = ChQuaternion<>({-0.7071068, 0, 0, 0.7071068});
            break;
        case 3:
            // univ john outer
            // University facing west, just before Park. Third lane from the left
            initLoc = ChVector<>({109.56 + (rank - 5) / 4 * 9, 2.41, 0.11});
            initRot = ChQuaternion<>({-0.00942852, 0.00497593, -0.00215422, -0.999941});
            break;
        default:
            std::cerr << "Unexpectedly reached default case statement" << std::endl;
            // University facing west, just before Park. Second lane from the left.
            initLoc = ChVector<>({75.0 - (rank - 1) * 9, -7.5, 0.5});
            initRot = ChQuaternion<>({-0.00942852, 0.00497593, -0.00215422, -0.999941});
            break;
    }

    return ChCoordsys<>(initLoc, initRot);
}

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // CLI tools for default synchrono demos
    SynCLI cli(argv[0]);
    cli.AddDefaultDemoOptions();
    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    int traffic_light_rank = 0;

    // -------------
    // Traffic light
    // -------------
    if (rank == traffic_light_rank) {
        auto agent = chrono_types::make_shared<SynEnvironmentAgent>(traffic_light_rank);
        mpi_manager.AddAgent(agent, traffic_light_rank);
        agent->SetBrain(chrono_types::make_shared<SynEnvironmentBrain>(traffic_light_rank));

        // The schedule will be affected by the initial color. First indice is the color of the initialized color,
        // second one is the next color
        std::vector<double> scheduleGreenStart = {10, 1, 10};
        std::vector<double> scheduleRedStart = {10, 10, 1};
        std::vector<double> scheduleRedStartSlow = {20, 10, 1};

        std::vector<ChVector<>> Lane0Points = {{1.5, -15, 1}, {1.5, -35, 1}};
        ApproachLane Lane0(2.5, Lane0Points);
        agent->AddLane(0, 0, Lane0, LaneColor::GREEN, scheduleGreenStart);

        std::vector<ChVector<>> Lane1Points = {{6.5, -15, 1}, {6.5, -35, 1}};
        ApproachLane Lane1(2.5, Lane1Points);
        agent->AddLane(0, 0, Lane1, LaneColor::GREEN, scheduleGreenStart);

        std::vector<ChVector<>> Lane2Points = {{14, -6, 1}, {34, -6, 1}};
        ApproachLane Lane2(2.5, Lane2Points);
        agent->AddLane(0, 1, Lane2, LaneColor::RED, scheduleRedStartSlow);

        std::vector<ChVector<>> Lane3Points = {{14, -2.75, 1}, {34, -2.75, 1}};
        ApproachLane Lane3(2.5, Lane3Points);
        agent->AddLane(0, 1, Lane3, LaneColor::RED, scheduleRedStartSlow);

        std::vector<ChVector<>> Lane4Points = {{14, 0.5, 1}, {34, 0.5, 1}};
        ApproachLane Lane4(2.5, Lane4Points);
        agent->AddLane(0, 1, Lane4, LaneColor::RED, scheduleRedStartSlow);

        std::vector<ChVector<>> Lane5Points = {{14, 3.75, 1}, {34, 3.75, 1}};
        ApproachLane Lane5(2.5, Lane5Points);
        agent->AddLane(0, 1, Lane5, LaneColor::RED, scheduleRedStartSlow);

        std::vector<ChVector<>> Lane6Points = {{-3.5, 14, 1}, {-3.5, 50, 1}};
        ApproachLane Lane6(2.5, Lane6Points);
        agent->AddLane(0, 2, Lane6, LaneColor::RED, scheduleRedStart);
    } else {
        // Sedan Agent
        auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        mpi_manager.AddAgent(agent, rank);

        // -------
        // Vehicle
        // -------
        auto sedan = chrono_types::make_shared<Sedan>();
        sedan->SetContactMethod(CONTACT_METHOD);
        sedan->SetChassisCollisionType(ChassisCollisionType::NONE);
        sedan->SetChassisFixed(false);
        sedan->SetInitPosition(GetInitialState(rank));
        sedan->SetTireType(TireModelType::TMEASY);
        sedan->SetTireStepSize(STEP_SIZE);
        sedan->Initialize();

        sedan->SetChassisVisualizationType(VisualizationType::MESH);
        sedan->SetSuspensionVisualizationType(VisualizationType::NONE);
        sedan->SetSteeringVisualizationType(VisualizationType::NONE);
        sedan->SetWheelVisualizationType(VisualizationType::MESH);
        sedan->SetTireVisualizationType(VisualizationType::MESH);

        auto vehicle = chrono_types::make_shared<SynCustomWheeledVehicle<Sedan>>(sedan);
        agent->SetVehicle(vehicle);

        // Set the zombie visualization assets of the vehicle
        vehicle->SetZombieVisualizationFiles("sedan/sedan_chassis_vis.obj",  //
                                             "sedan/sedan_rim.obj",          //
                                             "sedan/sedan_tire.obj");        //
        vehicle->SetNumWheels(4);

        // -------
        // Terrain
        // -------
        auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());

        std::string col_mesh_filename = STRINGIFY(PARKST_COL_MESH_PATH);
        auto patch = terrain->AddPatch(DefaultMaterialSurface(), CSYSNORM, col_mesh_filename, "", 0.01, false);

        std::string vis_mesh_filename = STRINGIFY(PARKST_VIS_MESH_PATH);
        auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        vis_mesh->LoadWavefrontMesh(vis_mesh_filename, true, true);

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(vis_mesh);
        trimesh_shape->SetStatic(true);

        patch->GetGroundBody()->AddAsset(trimesh_shape);

        terrain->Initialize();
        agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

        // ----------
        // Controller
        // ----------
        GPScoord origin(43.073268, -89.400636);  // Centered at Park St. and University Ave. in downtown Madison, WI.
        auto framework = chrono_types::make_shared<SynFramework>(origin, agent->GetTerrain());

        std::shared_ptr<ChBezierCurve> path;
        int lane;
        switch ((rank + 2) % 4) {
            case 0:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_john_inner.txt"));
                lane = 3;
                break;
            case 1:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_grainger_loop.txt"));
                lane = 0;
                break;
            case 2:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_park_straight.txt"));
                lane = 6;
                break;
            case 3:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_john_outer.txt"));
                lane = 4;
                break;
            default:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_john_inner.txt"));
                lane = 3;
                break;
        }

        double target_speed = 7.5;
        double target_following_time = 1.2;
        double target_min_distance = 10;
        double current_distance = 100;
        bool isPathClosed = false;

        if (rank == 4) {
            target_min_distance = 13;
        }

        auto driver = chrono_types::make_shared<SynInteractiveDriver>(vehicle->GetVehicle());

        // Set the time response for steering and throttle keyboard inputs.
        double steering_time = 1.0;          // time to go from 0 to +1 (or from 0 to -1)
        double throttle_time = 1.0;          // time to go from 0 to +1
        double braking_time = 0.3;           // time to go from 0 to +1
        double render_step_size = 1.0 / 50;  // FPS = 50
        driver->SetSteeringDelta(render_step_size / steering_time);
        driver->SetThrottleDelta(render_step_size / throttle_time);
        driver->SetBrakingDelta(render_step_size / braking_time);

        auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, agent->GetChVehicle());
        agent->SetBrain(brain);

        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
        agent->AttachVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
        if (cli.HasValueInVector<int>("irr", rank)) {
            auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver, render_step_size);
            irr_vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
            vis_manager->AddVisualization(irr_vis);
        }
#endif

#ifdef CHRONO_SENSOR
        if (cli.HasValueInVector<int>("sens", rank)) {
            auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();

            auto manager = chrono_types::make_shared<ChSensorManager>(agent->GetSystem());
            manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 2000);
            manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 2000);
            sen_vis->SetSensorManager(manager);

            auto cam = chrono_types::make_shared<ChCameraSensor>(
                vehicle->GetVehicle().GetChassisBody(),                                  // body camera is attached to
                30,                                                                      // update rate in Hz
                chrono::ChFrame<double>({-.25, 0, 1.25}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
                3840,                                                                    // image width
                720,                                                                     // image height
                CH_C_PI / 3);

            if (cli.GetAsType<bool>("sens_vis"))
                cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(3840, 720));

            if (cli.GetAsType<bool>("sens_save")) {
                std::string path = std::string("SENSOR_OUTPUT/ParkSt") + std::to_string(rank) + std::string("/");
                cam->PushFilter(chrono_types::make_shared<ChFilterSave>(path));
            }

            sen_vis->SetSensor(cam);
            vis_manager->AddVisualization(sen_vis);
        }
#endif  // SENSOR
    }

    mpi_manager.Initialize();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance();
        mpi_manager.Synchronize();
        mpi_manager.Update();
    }

    return 0;
}
