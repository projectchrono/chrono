#include "chrono_synchrono/simulation/SynSimulationConfig.h"

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynDDSAgent.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/cli/SynCLI.h"

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;

int main(int argc, char** argv) {
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    SynCLI cli(argv[0]);
    cli.AddDefaultDemoOptions();
    cli.AddOption<bool>("Demo", "z,zombie", "Zombie", "false");
    cli.Parse(argc, argv);

    // Chrono system
    ChSystem* sys = (CONTACT_METHOD == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                             : static_cast<ChSystem*>(new ChSystemSMC);
    sys->Set_G_acc(ChVector<>(0, 0, -9.81));
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys->SetSolverMaxIterations(150);
    sys->SetMaxPenetrationRecoverySpeed(4.0);

    std::string filename = GetSynDataFile("agent/SedanAgent.json");

    std::shared_ptr<SynWheeledVehicleAgent> agent;
    if (cli.GetAsType<bool>("zombie") && rank == 0) {
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, filename);
        agent->SetSystem(sys);
    } else {
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, filename, sys);
        agent->Initialize(ChCoordsys<>({0, 3.0 * rank, 0.5}, {1, 0, 0, 0}));
    }

    if (!cli.GetAsType<bool>("zombie")) {
        // if (false) {
        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
        auto vis = chrono_types::make_shared<SynIrrVehicleVisualization>(agent->GetDriver());
        vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
        vis_manager->AddVisualization(vis);

        auto driver = chrono_types::make_shared<ChIrrGuiDriver>(*vis->GetIrrApp());
        auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver);
        agent->SetBrain(brain);
        vis->SetDriver(driver);

        agent->AttachVisualizationManager(vis_manager);
    }

    if (rank == 0) {
        auto dds_agent = chrono_types::make_shared<SynDDSAgent>(agent, cli.GetAsType<bool>("zombie"));
        dds_agent->Initialize();

        mpi_manager.AddAgent(dds_agent);
    } else {
        mpi_manager.AddAgent(agent);
    }

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

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Rank " << rank << ": " << (time_span.count() / 1e3) / END_TIME << std::endl;

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
