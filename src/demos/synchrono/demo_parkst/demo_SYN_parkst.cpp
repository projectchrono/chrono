#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/framework/SynFramework.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

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

#include "chrono_models/vehicle/sedan/Sedan.h"

#include <chrono>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle::sedan;

// =============================================================================

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;

// Simulation end time
double end_time = 1000;

// Simulation step sizes
double step_size = 3e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Render rank
int render_rank = 0;

// SynChrono synchronization heartbeat
float heartbeat = 1e-2;  // 100[Hz]

// Sensor saving and visualizing
bool sens_save = false;
bool sens_vis = true;

// =============================================================================

ChCoordsys<> GetInitialState(int rank) {
    ChVector<> initLoc;
    ChQuaternion<> initRot;
    switch ((rank + 3) % 4) {
        case 0:
            // univ john inner
            // University facing west, just before Park. Second lane from the left.
            initLoc = ChVector<>({65.26 + (rank - 1) / 4 * 9, -2.82, 0.067});
            initRot = ChQuaternion<>({-0.00942852, 0.00497593, -0.00215422, -0.999941});
            break;
        case 1:
            // Grainger loop
            // Park St facing north, just before University. Left turn lane.
            initLoc = ChVector<>({2, -40.0 - (rank - 2) / 4 * 9, 0.5});
            initRot = ChQuaternion<>({-0.7071068, 0, 0, -0.7071068});
            break;
        case 2:
            // Park st. straight
            // Park St facing south, just before University. Right turn lane.
            initLoc = ChVector<>({-2.63, 69.0 + (rank - 3) / 3 * 9, 0.72});
            initRot = ChQuaternion<>({-0.7071068, 0, 0, 0.7071068});
            break;
        case 3:
            // univ john outer
            // University facing west, just before Park. Third lane from the left
            initLoc = ChVector<>({109.56 + (rank - 4) / 4 * 9, 2.41, 0.11});
            initRot = ChQuaternion<>({-0.00942852, 0.00497593, -0.00215422, -0.999941});
            break;
        default:
            std::cerr << "Unexpectedly reached default case statement" << std::endl;
            // University facing west, just before Park. Second lane from the left.
            initLoc = ChVector<>({75.0 - rank * 9, -7.5, 0.5});
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
        std::vector<double> schedule_green_start = {10, 1, 10};
        std::vector<double> schedule_red_start = {10, 10, 1};
        std::vector<double> schedule_red_start_slow = {20, 10, 1};

        std::vector<ChVector<>> Lane0Points = {{1.5, -15, 1}, {1.5, -35, 1}};
        ApproachLane Lane0(2.5, Lane0Points);
        agent->AddLane(0, 0, Lane0, LaneColor::GREEN, schedule_green_start);

        std::vector<ChVector<>> Lane1Points = {{6.5, -15, 1}, {6.5, -35, 1}};
        ApproachLane Lane1(2.5, Lane1Points);
        agent->AddLane(0, 0, Lane1, LaneColor::GREEN, schedule_green_start);

        std::vector<ChVector<>> Lane2Points = {{14, -6, 1}, {34, -6, 1}};
        ApproachLane Lane2(2.5, Lane2Points);
        agent->AddLane(0, 1, Lane2, LaneColor::RED, schedule_red_start_slow);

        std::vector<ChVector<>> Lane3Points = {{14, -2.75, 1}, {34, -2.75, 1}};
        ApproachLane Lane3(2.5, Lane3Points);
        agent->AddLane(0, 1, Lane3, LaneColor::RED, schedule_red_start_slow);

        std::vector<ChVector<>> Lane4Points = {{14, 0.5, 1}, {34, 0.5, 1}};
        ApproachLane Lane4(2.5, Lane4Points);
        agent->AddLane(0, 1, Lane4, LaneColor::RED, schedule_red_start_slow);

        std::vector<ChVector<>> Lane5Points = {{14, 3.75, 1}, {34, 3.75, 1}};
        ApproachLane Lane5(2.5, Lane5Points);
        agent->AddLane(0, 1, Lane5, LaneColor::RED, schedule_red_start_slow);

        std::vector<ChVector<>> Lane6Points = {{-3.5, 14, 1}, {-3.5, 50, 1}};
        ApproachLane Lane6(2.5, Lane6Points);
        agent->AddLane(0, 2, Lane6, LaneColor::RED, schedule_red_start);
    } else {
        // Sedan Agent
        auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        mpi_manager.AddAgent(agent, rank);

        // -------
        // Vehicle
        // -------
        auto sedan = chrono_types::make_shared<Sedan>();
        sedan->SetContactMethod(contact_method);
        sedan->SetChassisCollisionType(ChassisCollisionType::NONE);
        sedan->SetChassisFixed(false);
        sedan->SetInitPosition(GetInitialState(rank));
        sedan->SetTireType(TireModelType::TMEASY);
        sedan->SetTireStepSize(step_size);
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
        switch ((rank + 3) % 4) {
            case 0:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_john_inner.txt"));
                break;
            case 1:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_grainger_loop.txt"));
                break;
            case 2:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_park_straight.txt"));
                break;
            case 3:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_john_outer.txt"));
                break;
            default:
                path = framework->CurveFromGPS(GetSynDataFile("path/parkst_john_inner.txt"));
                break;
        }

        double target_speed = 7.5;
        double target_following_time = 1.2;
        double target_min_distance = 10;
        double current_distance = 100;
        bool isPathClosed = false;

        auto driver = chrono_types::make_shared<ChPathFollowerACCDriver>(
            vehicle->GetVehicle(), path, "Park St", target_speed, target_following_time, target_min_distance,
            current_distance, isPathClosed);

        driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        driver->GetSteeringController().SetLookAheadDistance(5);

        auto brain = chrono_types::make_shared<SynACCBrain>(rank, driver, vehicle->GetVehicle());
        agent->SetBrain(brain);

        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
        agent->SetVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
        if (rank == render_rank) {
            auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver);
            irr_vis->SetRenderStepSize(render_step_size);
            irr_vis->SetStepSize(step_size);
            irr_vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
            vis_manager->AddVisualization(irr_vis);
        }
#endif

#ifdef CHRONO_SENSOR
        if (rank == render_rank) {
            auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();

            auto manager = chrono_types::make_shared<ChSensorManager>(agent->GetSystem());
            manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 2000);
            manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 2000);
            sen_vis->SetSensorManager(manager);

            auto origin = chrono_types::make_shared<ChBody>();
            origin->SetBodyFixed(true);
            agent->GetSystem()->AddBody(origin);

            // ISO Angle
            ChVector<> camera_loc(40, -40, 55);

            ChQuaternion<> rotation = QUNIT;
            ChQuaternion<> qA = Q_from_AngAxis(45 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;

            // Overhead Angle
            // ChVector<> camera_loc(0, 0, 45);

            // ChQuaternion<> rotation = QUNIT;
            // ChQuaternion<> qA = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_Y);
            // ChQuaternion<> qB = Q_from_AngAxis(180 * CH_C_DEG_TO_RAD, VECT_Z);
            // rotation = rotation >> qA >> qB;

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
            if (sens_vis)
                intersection_camera->PushFilter(
                    chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height));

            std::string path = std::string("SENSOR_OUTPUT/Sedan") + std::to_string(rank) + std::string("/");
            if (sens_save)
                intersection_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(path));

            sen_vis->SetSensor(intersection_camera);
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
