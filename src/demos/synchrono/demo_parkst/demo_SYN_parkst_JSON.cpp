#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/framework/SynFramework.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/brain/SynACCBrain.h"
#include "chrono_synchrono/brain/SynEnvironmentBrain.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

#include <chrono>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;
using namespace chrono::synchrono;

// ------------------------------------------------------------------------------------

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

// ------------------------------------------------------------------------------------

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

    // -------------
    // Traffic light
    // -------------
    int traffic_light_rank = 0;
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
        std::string vehicle_filename = GetSynDataFile("vehicle/Sedan.json");
        auto vehicle = chrono_types::make_shared<SynWheeledVehicle>(vehicle_filename, CONTACT_METHOD);
        vehicle->Initialize(GetInitialState(rank));
        agent->SetVehicle(vehicle);

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

        auto driver = chrono_types::make_shared<ChPathFollowerACCDriver>(
            vehicle->GetVehicle(), path, "Park St", target_speed, target_following_time, target_min_distance,
            current_distance, isPathClosed);

        driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        driver->GetSteeringController().SetLookAheadDistance(5);

        auto brain = chrono_types::make_shared<SynACCBrain>(rank, driver, vehicle->GetVehicle());
        agent->SetBrain(brain);

        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
        agent->AttachVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
        if (cli.HasValueInVector<int>("irr", rank)) {
            auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver);
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

            auto origin = chrono_types::make_shared<ChBody>();
            origin->SetBodyFixed(true);
            agent->GetSystem()->AddBody(origin);

            ChVector<> camera_loc(0, 0, 45);
            // double theta = acos((camera_loc ^ chrono::VECT_X) / (camera_loc.Length() * 1));
            // ChVector<> rot_axis = chrono::VECT_X % (-camera_loc);
            // ChQuaternion<> rotation = Q_from_AngAxis(theta, rot_axis);

            // Rotations to get a nice angle
            ChQuaternion<> rotation = QUNIT;
            // ChQuaternion<> qA = Q_from_AngAxis(45 * CH_C_DEG_TO_RAD, VECT_Y);
            // ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            ChQuaternion<> qA = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(180 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;

            double cam_res_width = 1280;
            double cam_res_height = 720;

            auto intersection_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
                origin,                                         // body camera is attached to
                30,                                             // update rate in Hz
                chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
                cam_res_width,                                  // image width
                cam_res_height,                                 // image height
                CH_C_PI / 3);

            intersection_camera->SetName("Intersection Cam");
            intersection_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
            if (cli.GetAsType<bool>("sens_vis"))
                intersection_camera->PushFilter(
                    chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height));

            std::string path = std::string("SENSOR_OUTPUT/Sedan") + std::to_string(rank) + std::string("/");
            if (cli.GetAsType<bool>("sens_save")) {
                intersection_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(path));
            }

            sen_vis->SetSensor(intersection_camera);
            vis_manager->AddVisualization(sen_vis);
        }
#endif  // SENSOR
    }

    mpi_manager.Barrier();
    mpi_manager.Initialize();
    mpi_manager.Barrier();

    double step = 0;
    double wall_time = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();

        // Advance
        mpi_manager.Advance();
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
        auto inc = t.count() / 1e6;
        wall_time += inc;
        if (VERBOSE && inc > 1e-4) {
            std::cout << step << "." << rank << " adv: " << t.count() / 1e6 << std::endl;
        }

        // Synchronize
        mpi_manager.Synchronize();
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        t = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        inc = t.count() / 1e6;
        wall_time += inc;
        if (VERBOSE && inc > 1e-4) {
            std::cout << step << "." << rank << " syn: " << inc << std::endl;
        }

        // Broadcast
        mpi_manager.Broadcast();
        std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
        t = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
        inc = t.count() / 1e6;
        wall_time += inc;
        if (VERBOSE && inc > 1e-4) {
            std::cout << step << "." << rank << " bro: " << inc << std::endl;
        }

        // Update
        mpi_manager.Update();
        std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
        t = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
        inc = t.count() / 1e6;
        wall_time += inc;
        if (VERBOSE && inc > 1e-4) {
            std::cout << step << "." << rank << " upd: " << inc << std::endl;
        }

        // increment the step
        step++;

        std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
        t = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t0);
        wall_time += t.count() / 1e6;

        if (fmod(step * HEARTBEAT, 1.0) <= 1e-5) {
            // std::cout << "Wall Time (" << rank << "):: " << wall_time << std::endl;
            wall_time = 0;
        }
    }

    if (rank == 0) {
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        // std::cout << "Total Wall Time: " << time_span.count() / 1e6 << "." << std::endl;

        std::cout << "Fraction of real time: " << (time_span.count() / 1e6) / END_TIME << std::endl;
    }

    return 0;
}
