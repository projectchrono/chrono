#include "chrono_synchrono/visualization/SynVisualizationManager.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

#include "chrono_models/vehicle/gator/Gator.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>

using namespace chrono;
using namespace chrono::synchrono;
using namespace chrono::vehicle::gator;

// =============================================================================

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // CLI tools for default synchrono demos
    SynCLI cli("demo_SYN_vehicles");
    cli.AddDefaultDemoOptions();
    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    // --------------------
    // Agent Initialization
    // --------------------

    // All ranks will be vehicle agents with an attached Gator vehicle
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
    mpi_manager.AddAgent(agent, rank);

    // -------
    // Vehicle
    // -------
    auto gator = chrono_types::make_shared<Gator>();
    gator->SetContactMethod(CONTACT_METHOD);
    gator->SetChassisCollisionType(ChassisCollisionType::NONE);
    gator->SetChassisFixed(false);
    gator->SetInitPosition(ChCoordsys<>({0, 3.0 * rank, 0.5}, {1, 0, 0, 0}));
    gator->SetTireType(TireModelType::TMEASY);
    gator->SetTireStepSize(STEP_SIZE);
    gator->Initialize();

    gator->SetChassisVisualizationType(VisualizationType::MESH);
    gator->SetSuspensionVisualizationType(VisualizationType::NONE);
    gator->SetSteeringVisualizationType(VisualizationType::NONE);
    gator->SetWheelVisualizationType(VisualizationType::MESH);
    gator->SetTireVisualizationType(VisualizationType::MESH);

    auto vehicle = chrono_types::make_shared<SynCustomWheeledVehicle<Gator>>(gator);
    agent->SetVehicle(vehicle);

    // Set the zombie visualization assets of the vehicle
    vehicle->SetZombieVisualizationFiles("gator/gator_chassis.obj",      //
                                         "gator/gator_wheel_FL.obj",     //
                                         "gator/gator_tireF_fine.obj");  //
    vehicle->SetNumWheels(4);

    // -------
    // Terrain
    // -------
    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());
    auto patch = terrain->AddPatch(DefaultMaterialSurface(), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 100, 100);
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

    auto driver = chrono_types::make_shared<ChDriver>(vehicle->GetVehicle());
    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, vehicle->GetVehicle());
    agent->SetBrain(brain);

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->AttachVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (cli.HasValueInVector<int>("irr", rank)) {
        // Set driver as ChIrrGuiDriver
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>();
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
    if (cli.HasValueInVector<int>("sens", rank)) {
        std::string path = std::string("SENSOR_OUTPUT/gator") + std::to_string(rank) + std::string("/");

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

    mpi_manager.Barrier();
    mpi_manager.Initialize();
    mpi_manager.Barrier();

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance();
        mpi_manager.Synchronize();
        mpi_manager.Update();
    }

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
