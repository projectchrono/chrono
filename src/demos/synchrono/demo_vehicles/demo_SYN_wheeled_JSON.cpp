#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono_synchrono/scenario/SynScenarioManager.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/terrain/SynTerrainFactory.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

#include "chrono_models/vehicle/gator/Gator.h"

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
    cli.AddOption<std::string>("Demo", "type", "JSON Loader Type", "Vehicle", "Vehicle/Agent/Scenario");
    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    std::string type = cli.GetAsType<std::string>("type");

    // --------------------
    // Agent Initialization
    // --------------------

    std::shared_ptr<SynWheeledVehicleAgent> agent;

    if (type.compare("Scenario") == 0) {
        SynScenarioManager scenario_manager(mpi_manager);
        scenario_manager.LoadScenario(GetSynDataFile("scenario/VehicleSimple.json"));

        agent = std::static_pointer_cast<SynWheeledVehicleAgent>(mpi_manager.GetAgent(rank));
    } else if (type.compare("Agent") == 0) {
        auto filename = GetSynDataFile("agent/SedanAgent.json");
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, filename, CONTACT_METHOD);
        agent->Initialize(ChCoordsys<>({0, 3.0 * (rank - 1), 0.5}, {1, 0, 0, 0}));

        mpi_manager.AddAgent(agent, rank);
    } else if (type.compare("Vehicle") == 0) {
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        mpi_manager.AddAgent(agent, rank);

        auto vehicle_filename = GetSynDataFile("vehicle/Sedan.json");
        auto vehicle = chrono_types::make_shared<SynWheeledVehicle>(vehicle_filename, CONTACT_METHOD);
        vehicle->Initialize(ChCoordsys<>({0, 3.0 * (rank - 1), 0.5}, {1, 0, 0, 0}));
        agent->SetVehicle(vehicle);

        auto driver = chrono_types::make_shared<ChDriver>(vehicle->GetVehicle());
        auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, vehicle->GetVehicle());
        agent->SetBrain(brain);

        auto terrain_filename = GetSynDataFile("terrain/RigidPlane.json");
        auto terrain = SynTerrainFactory::CreateTerrain(agent->GetSystem(), terrain_filename);
        agent->SetTerrain(terrain);
    } else {
        std::string out = "\"" + type + "\" is not a viable type. Must be \"Scenario\", \"Agent\" or \"Vehicle\".";
        std::cout << out << std::endl;
        mpi_manager.Exit();
    }

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->AttachVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (cli.HasValueInVector<int>("irr", rank)) {
        // Set driver as ChIrrGuiDriver
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>();
        irr_vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());

        // Set the driver in the vehicle brain and the irrlicht visualizer
        auto driver = chrono_types::make_shared<ChIrrGuiDriver>(*irr_vis->GetIrrApp());
        agent->GetBrain()->SetDriver(driver);
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
