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
// =============================================================================
//
// Basic demonstration of multiple wheeled vehicles in a single simulation using
// the SynChrono wrapper
//
// =============================================================================

#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
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

#include "chrono_models/vehicle/gator/Gator.h"

using namespace chrono;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;

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

// =============================================================================

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    mpi_manager.SetHeartbeat(heartbeat);
    mpi_manager.SetEndTime(end_time);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // --------------------
    // Agent Initialization
    // --------------------

    // All ranks will be wheeled vehicle agents
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
    agent->SetStepSize(step_size);
    mpi_manager.AddAgent(agent, rank);

    // -------
    // Vehicle
    // -------

    std::shared_ptr<SynWheeledVehicle> vehicle;
    ChCoordsys<> init_pos({0, 3.0 * rank, 0.5}, {1, 0, 0, 0});
    if (rank % 2 == 0) {
        // Even numbered ranks will be a custom vehicle that is not specified by a JSON file
        auto gator = chrono_types::make_shared<Gator>();
        gator->SetContactMethod(contact_method);
        gator->SetChassisCollisionType(ChassisCollisionType::NONE);
        gator->SetChassisFixed(false);
        gator->SetInitPosition(init_pos);
        gator->SetTireType(TireModelType::TMEASY);
        gator->SetTireStepSize(step_size);
        gator->Initialize();

        gator->SetChassisVisualizationType(VisualizationType::MESH);
        gator->SetSuspensionVisualizationType(VisualizationType::NONE);
        gator->SetSteeringVisualizationType(VisualizationType::NONE);
        gator->SetWheelVisualizationType(VisualizationType::MESH);
        gator->SetTireVisualizationType(VisualizationType::MESH);

        vehicle = chrono_types::make_shared<SynCustomWheeledVehicle<Gator>>(gator);

        // Set the zombie visualization assets of the vehicle
        // This is done only with custom vehicles because this information is required in the JSON format
        vehicle->SetZombieVisualizationFiles("gator/gator_chassis.obj",      //
                                             "gator/gator_wheel_FL.obj",     //
                                             "gator/gator_tireF_fine.obj");  //
        vehicle->SetNumWheels(4);
    } else {
        // Odd ranks will be a Sedan vehicle specified through a JSON file
        auto vehicle_filename = GetSynDataFile("vehicle/Sedan.json");
        vehicle = chrono_types::make_shared<SynWheeledVehicle>(init_pos, vehicle_filename, contact_method);
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
    if (auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(patch_asset)) {
        auto box_texture = chrono_types::make_shared<ChVisualMaterial>();
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
    if (rank == render_rank) {
        // Set driver as ChIrrGuiDriver
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>();
        irr_vis->SetRenderStepSize(render_step_size);
        irr_vis->SetStepSize(step_size);
        irr_vis->InitializeAsDefaultChaseCamera(vehicle);

        // Set the driver in the vehicle brain and the irrlicht visualizer
        auto driver = chrono_types::make_shared<ChIrrGuiDriver>(*irr_vis->GetIrrApp());
        brain->SetDriver(driver);
        irr_vis->SetDriver(driver);

        // Add the visualization
        vis_manager->AddVisualization(irr_vis);
    }
#endif

#ifdef CHRONO_SENSOR
    if (rank == render_rank) {
        std::string path = std::string("SENSOR_OUTPUT/wheeled") + std::to_string(rank) + std::string("/");

        auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();
        sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
        sen_vis->InitializeAsDefaultChaseCamera(agent->GetChVehicle().GetChassisBody());
        sen_vis->AddFilterSave(path);
        sen_vis->AddFilterVisualize();
        vis_manager->AddVisualization(sen_vis);
    }
#endif

    mpi_manager.Barrier();
    mpi_manager.Initialize();
    mpi_manager.Barrier();

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
