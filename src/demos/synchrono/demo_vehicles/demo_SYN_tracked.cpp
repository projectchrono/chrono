#include "chrono_synchrono/visualization/SynVisualizationManager.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/vehicle/SynTrackedVehicle.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono/solver/ChSolverBB.h"

#include <chrono>

using namespace chrono;
using namespace chrono::synchrono;
using namespace chrono::vehicle::m113;

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
    auto agent = chrono_types::make_shared<SynTrackedVehicleAgent>(rank);
    mpi_manager.AddAgent(agent, rank);

    // -------
    // Vehicle
    // -------
    auto m113 = chrono_types::make_shared<M113>();
    m113->SetContactMethod(CONTACT_METHOD);
    m113->SetChassisCollisionType(ChassisCollisionType::NONE);
    m113->SetChassisFixed(false);
    m113->SetInitPosition(ChCoordsys<>({0, 3.0 * (rank + 1), 0.75}, {1, 0, 0, 0}));
    m113->SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113->SetBrakeType(BrakeType::SIMPLE);
    m113->Initialize();

    m113->SetChassisVisualizationType(VisualizationType::MESH);
    m113->SetSprocketVisualizationType(VisualizationType::MESH);
    m113->SetIdlerVisualizationType(VisualizationType::MESH);
    m113->SetRoadWheelAssemblyVisualizationType(VisualizationType::NONE);
    m113->SetRoadWheelVisualizationType(VisualizationType::MESH);
    m113->SetTrackShoeVisualizationType(VisualizationType::MESH);

    auto vehicle = chrono_types::make_shared<SynCustomTrackedVehicle<M113>>(m113);
    agent->SetVehicle(vehicle);

    // Set the zombie visualization assets of the vehicle
    vehicle->SetZombieVisualizationFiles("M113/Chassis.obj",     //
                                         "M113/TrackShoe.obj",   //
                                         "M113/Sprocket_L.obj",  //
                                         "M113/Sprocket_R.obj",  //
                                         "M113/Idler_L.obj",     //
                                         "M113/Idler_R.obj",     //
                                         "M113/Roller_L.obj",    //
                                         "M113/Roller_R.obj");   //

    vehicle->SetNumAssemblyComponents(127, 2, 2, 10);

    // -------
    // Terrain
    // -------
    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());

    auto patch = terrain->AddPatch(DefaultMaterialSurface(), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 100, 100);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
#ifdef CHRONO_SENSOR
    // For OptiX
    // Note: might not always be the 0th texture, may need to loop until you find one that
    //          casts correctly
    auto patch_asset = patch->GetGroundBody()->GetAssets()[0];
    if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(patch_asset)) {
        std::shared_ptr<ChVisualMaterial> box_texture = chrono_types::make_shared<ChVisualMaterial>();
        box_texture->SetKdTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
        // FresnelMax and SpecularColor should make it less shiny
        box_texture->SetFresnelMax(0.2);
        box_texture->SetSpecularColor({0.2, 0.2, 0.2});

        visual_asset->material_list.push_back(box_texture);
    }
#endif
    terrain->Initialize();
    agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

    auto driver = chrono_types::make_shared<ChDriver>(vehicle->GetVehicle());
    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, vehicle->GetVehicle());
    agent->SetBrain(brain);

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->AttachVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (cli.HasValueInVector<int>("irr", rank)) {
        // Set driver as ChIrrGuiDriver
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(nullptr, STEP_SIZE);
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
        mpi_manager.Broadcast();
        mpi_manager.Update();
    }

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
