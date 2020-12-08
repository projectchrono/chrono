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
// Authors: Asher Elmquist, Aaron Young
// =============================================================================
//
// Basic demonstration of multiple tracked vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/vehicle/SynTrackedVehicle.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"

using namespace chrono::sensor;
#endif

#include "chrono_models/vehicle/m113/M113.h"

using namespace chrono;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================
const ChContactMethod contact_method = ChContactMethod::NSC;

// [s]
double end_time = 1000;
double step_size = 3e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
float heartbeat = 1e-2;  // 100[Hz]

void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

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

    const bool use_sensor_vis = cli.HasValueInVector<int>("sens", rank);
    const bool use_irrlicht_vis = !use_sensor_vis && cli.HasValueInVector<int>("irr", rank);

    mpi_manager.SetHeartbeat(heartbeat);
    mpi_manager.SetEndTime(end_time);

    // --------------------
    // Agent Initialization
    // --------------------

    // All ranks will be a tracked vehicle agent
    auto agent = chrono_types::make_shared<SynTrackedVehicleAgent>(rank);
    agent->SetStepSize(step_size);
    mpi_manager.AddAgent(agent, rank);

    // -------
    // Vehicle
    // -------

    // Load in a M113 vehicle and it's subsequent SynChrono relavent information from a JSON specification file
    std::shared_ptr<SynTrackedVehicle> vehicle;
    ChCoordsys<> init_pos({0, 3.0 * rank, 0.75}, {1, 0, 0, 0});
    if (rank % 2 == 0) {
        // Even numbered ranks will be a custom vehicle that is not specified by a JSON file
        auto m113 = chrono_types::make_shared<M113>();
        m113->SetContactMethod(contact_method);
        m113->SetChassisCollisionType(CollisionType::NONE);
        m113->SetChassisFixed(false);
        m113->SetInitPosition(init_pos);
        m113->SetTrackShoeType(TrackShoeType::SINGLE_PIN);
        m113->SetBrakeType(BrakeType::SIMPLE);
        m113->Initialize();

        m113->SetChassisVisualizationType(VisualizationType::MESH);
        m113->SetSprocketVisualizationType(VisualizationType::MESH);
        m113->SetIdlerVisualizationType(VisualizationType::MESH);
        m113->SetRoadWheelAssemblyVisualizationType(VisualizationType::NONE);
        m113->SetRoadWheelVisualizationType(VisualizationType::MESH);
        m113->SetTrackShoeVisualizationType(VisualizationType::MESH);

        vehicle = chrono_types::make_shared<SynCustomTrackedVehicle<M113>>(m113);

        // Set the zombie visualization assets of the vehicle
        // This is done only with custom vehicles because this information is required in the JSON format
        vehicle->SetZombieVisualizationFiles("M113/Chassis.obj",     //
                                             "M113/TrackShoe.obj",   //
                                             "M113/Sprocket_L.obj",  //
                                             "M113/Sprocket_R.obj",  //
                                             "M113/Idler_L.obj",     //
                                             "M113/Idler_R.obj",     //
                                             "M113/Roller_L.obj",    //
                                             "M113/Roller_R.obj");   //

        vehicle->SetNumAssemblyComponents(127, 2, 2, 10);
    } else {
        // Odd ranks will be a M113 vehicle specified through a JSON file
        auto vehicle_filename = synchrono::GetDataFile("vehicle/M113.json");
        vehicle = chrono_types::make_shared<SynTrackedVehicle>(init_pos, vehicle_filename, contact_method);
    }
    agent->SetVehicle(vehicle);

    // Specify the driver and brain of the vehicle
    auto driver = chrono_types::make_shared<ChDriver>(vehicle->GetVehicle());
    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, vehicle->GetVehicle());
    agent->SetBrain(brain);

    // -------
    // Terrain
    // -------

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());
    auto patch = terrain->AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 100, 100);
    terrain->Initialize();

    // Terrain visualization
    // For irrlicht
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    // For sensor
    auto patch_asset = patch->GetGroundBody()->GetAssets()[0];
    if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(patch_asset)) {
        std::shared_ptr<ChVisualMaterial> box_texture = chrono_types::make_shared<ChVisualMaterial>();
        box_texture->SetKdTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
        // FresnelMax and SpecularColor should make it less shiny
        box_texture->SetFresnelMax(0.2);
        box_texture->SetSpecularColor({0.2, 0.2, 0.2});

        visual_asset->material_list.push_back(box_texture);
    }

    // Set the agents terrain
    agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->SetVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (use_irrlicht_vis) {
        // Set driver as ChIrrGuiDriver
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>();
        irr_vis->SetRenderStepSize(render_step_size);
        irr_vis->SetStepSize(step_size);
        irr_vis->InitializeAsDefaultTrackedChaseCamera(vehicle, 10);

        // Set the driver in the vehicle brain and the irrlicht visualizer
        auto driver = chrono_types::make_shared<ChIrrGuiDriver>(*irr_vis->GetIrrApp());
        brain->SetDriver(driver);
        irr_vis->SetDriver(driver);

        // Add the visualization
        vis_manager->AddVisualization(irr_vis);
    }
#endif

#ifdef CHRONO_SENSOR
    if (use_sensor_vis) {
        auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();
        sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
        sen_vis->InitializeAsDefaultChaseCamera(agent->GetChVehicle().GetChassisBody());

        if (cli.GetAsType<bool>("sens_save")) {
            std::string path = std::string("SENSOR_OUTPUT/tracked") + std::to_string(rank) + std::string("/");
            sen_vis->AddFilterSave(path);
        }

        if (cli.GetAsType<bool>("sens_vis"))
            sen_vis->AddFilterVisualize();

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
        mpi_manager.Update();
    }

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
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
}
