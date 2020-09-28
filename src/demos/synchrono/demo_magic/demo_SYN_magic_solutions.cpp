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
// Demo code of vehicles on a highway, used for hands-on exercises at MaGIC 2020
// These are the solutions and contain the completed tutorial
//
// =============================================================================

#include <chrono>

#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/framework/SynFramework.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/brain/SynACCBrain.h"
#include "chrono_synchrono/brain/SynEnvironmentBrain.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;

std::shared_ptr<SynWheeledVehicle> InitializeVehicle(int rank) {
    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    std::string filename;

    double init_z = 0.5;
    switch (rank) {
        case 0:
            filename = "vehicle/Sedan.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(2.8, -70, init_z);
            break;
        case 1:
            filename = "vehicle/CityBus.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(5.6, -70, init_z);
            break;
        default:
            std::cout << "No initial location specificied for this rank. Extra case needed?" << std::endl;
    }
    auto vehicle = chrono_types::make_shared<SynWheeledVehicle>(GetSynDataFile(filename), CONTACT_METHOD);
    vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));

    return vehicle;
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

    std::shared_ptr<SynWheeledVehicleAgent> agent;
    std::shared_ptr<ChMulPathFollowerACCDriver> multi_driver;

    int traffic_light_rank = 1;
    if (rank == traffic_light_rank) {
        // -------------
        // Traffic light
        // -------------

        auto agent = chrono_types::make_shared<SynEnvironmentAgent>(traffic_light_rank);
        mpi_manager.AddAgent(agent, traffic_light_rank);
        agent->SetBrain(chrono_types::make_shared<SynEnvironmentBrain>(traffic_light_rank));

        std::vector<double> schedule1 = {10, 1, 5};
        std::vector<ChVector<>> lane1_points = {{6.4, -50, 0.2}, {6.4, -15, 0.2}};
        ApproachLane lane_1(2.5, lane1_points);

        agent->AddLane(0, 0, lane_1, LaneColor::RED, schedule1);
    } else {
        // -------
        // Vehicle
        // -------
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        agent->SetVehicle(InitializeVehicle(rank));
        mpi_manager.AddAgent(agent, rank);

        // -------
        // Terrain
        // -------
        auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());

        auto patch = terrain->AddPatch(DefaultMaterialSurface(), CSYSNORM,
                                       GetSynDataFile("meshes/Highway_intersection.obj"), "", 0.01, false);

        auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        vis_mesh->LoadWavefrontMesh(GetSynDataFile("meshes/Highway_intersection.obj"), true, true);

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(vis_mesh);
        trimesh_shape->SetStatic(true);

        patch->GetGroundBody()->AddAsset(trimesh_shape);

        terrain->Initialize();
        agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

        // -------------------
        // Vehicle Controllers
        // -------------------

        auto loc = agent->GetChVehicle().GetVehiclePos();
        auto curve_pts = std::vector<ChVector<>>({loc, loc + ChVector<>(0, 140, 0)});
        auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);

        double target_speed = (rank == 0) ? 10 : 5;
        double target_following_time = 1.2;
        double target_min_distance = 10;
        double current_distance = 100;
        bool is_path_closed = false;

        std::shared_ptr<ChDriver> driver;

        if (rank != 0) {
            auto acc_driver = chrono_types::make_shared<ChPathFollowerACCDriver>(
                agent->GetChVehicle(), path, "Highway", target_speed, target_following_time, target_min_distance,
                current_distance, is_path_closed);
            acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
            acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
            acc_driver->GetSteeringController().SetLookAheadDistance(5);

            auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, acc_driver, agent->GetChVehicle());
            agent->SetBrain(brain);

            driver = acc_driver;
        } else {
            auto curve_pts2 = std::vector<ChVector<>>({ChVector<>({6.4, -70, 0.2}), ChVector<>(6.4, 70, 0.2)});
            auto path2 = chrono_types::make_shared<ChBezierCurve>(curve_pts2);

            std::vector<std::pair<std::shared_ptr<ChBezierCurve>, bool>> path_pairs;
            path_pairs.push_back({path, false});
            path_pairs.push_back({path2, false});

            multi_driver = chrono_types::make_shared<ChMulPathFollowerACCDriver>(
                agent->GetChVehicle(), path_pairs, "Highway", target_speed, target_following_time, target_min_distance,
                current_distance);

            multi_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
            multi_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
            multi_driver->GetSteeringController().SetLookAheadDistance(5);

            auto brain = chrono_types::make_shared<SynACCBrain>(rank, multi_driver, agent->GetChVehicle());
            agent->SetBrain(brain);

            driver = multi_driver;
        }

        // -------------
        // Visualization
        // -------------
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
            std::string path = std::string("SENSOR_OUTPUT/magic") + std::to_string(rank) + std::string("/");

            std::shared_ptr<SynSensorVisualization> sen_vis = chrono_types::make_shared<SynSensorVisualization>();
            sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
            sen_vis->InitializeAsDefaultChaseCamera(agent->GetChVehicle().GetChassisBody());

            if (cli.GetAsType<bool>("sens_save"))
                sen_vis->AddFilterSave(path);

            if (cli.GetAsType<bool>("sens_vis"))
                sen_vis->AddFilterVisualize();

            vis_manager->AddVisualization(sen_vis);
        }
#endif
    }

    mpi_manager.Initialize();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance();
        mpi_manager.Synchronize();
        mpi_manager.Update();

        if (rank == 0 && std::abs(agent->GetSystem()->GetChTime() - 2) < 1e-2)
            multi_driver->changePath(1);

        if (rank != traffic_light_rank) {
            auto pos = agent->GetChVehicle().GetVehicleCOMPos();
            // std::cout << "Position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
        }
    }

    return 0;
}