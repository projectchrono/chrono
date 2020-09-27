#include "chrono_synchrono/visualization/SynVisualizationManager.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/vehicle/SynTrackedVehicle.h"
#include "chrono_synchrono/terrain/SynTerrainFactory.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

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
    auto vehicle_filename = GetSynDataFile("vehicle/M113.json");
    auto vehicle = chrono_types::make_shared<SynTrackedVehicle>(vehicle_filename, CONTACT_METHOD);
    vehicle->Initialize(ChCoordsys<>({0, 3.0 * (rank + 1), 0.75}, {1, 0, 0, 0}));
    agent->SetVehicle(vehicle);

    auto driver = chrono_types::make_shared<ChDriver>(vehicle->GetVehicle());
    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, vehicle->GetVehicle());
    agent->SetBrain(brain);

    // -------
    // Terrain
    // -------
    auto terrain_filename = GetSynDataFile("terrain/RigidPlane.json");
    auto terrain = SynTerrainFactory::CreateTerrain(agent->GetSystem(), terrain_filename);
    agent->SetTerrain(terrain);

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->AttachVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (cli.HasValueInVector<int>("irr", rank)) {
        // Set driver as ChIrrGuiDriver
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>();
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
        mpi_manager.Update();
    }

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
