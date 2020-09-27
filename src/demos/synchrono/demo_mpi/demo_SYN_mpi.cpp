#include <chrono>

#include "chrono_synchrono/simulation/SynSimulationConfig.h"

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;

int main(int argc, char** argv) {
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    SynCLI cli(argv[0]);
    cli.AddDefaultDemoOptions();
    cli.Parse(argc, argv);

    std::string filename = GetSynDataFile("agent/SedanAgent.json");
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, filename, CONTACT_METHOD);
    agent->Initialize(ChCoordsys<>({0, 3.0 * rank, 0.5}, {1, 0, 0, 0}));

    if (rank == 1) {
        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
        auto vis = chrono_types::make_shared<SynIrrVehicleVisualization>(agent->GetDriver());
        vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
        vis_manager->AddVisualization(vis);

        auto driver = chrono_types::make_shared<ChIrrGuiDriver>(*vis->GetIrrApp());
        auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, agent->GetChVehicle());
        agent->SetBrain(brain);
        vis->SetDriver(driver);

        agent->AttachVisualizationManager(vis_manager);
    }

    mpi_manager.AddAgent(agent);

    mpi_manager.Initialize();

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance();
        mpi_manager.Synchronize();
        mpi_manager.Update();
    }

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Rank " << rank << ": " << (time_span.count() / 1e3) / END_TIME << std::endl;

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
