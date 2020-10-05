#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/communication/dds/SynDDSManager.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono/core/ChRealtimeStep.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;

int main(int argc, char** argv) {
    SynCLI cli(argv[0]);
    cli.AddDefaultDemoOptions();
    cli.AddOption<int>("Demo", "r,rank", "Rank", "1");
    cli.AddOption<int>("Demo", "n,num_ranks", "Num Ranks", "2");
    cli.AddOption<std::string>("Demo", "i,ip", "IP address to look at in addition to localhost", "127.0.0.1");
    cli.AddOption<int>("Demo", "p,port", "Port", "0");
    cli.Parse(argc, argv);

    const int rank = cli.GetAsType<int>("rank");
    const int num_ranks = cli.GetAsType<int>("num_ranks");

    SynDDSManager dds_manager(rank, num_ranks);

    std::string filename = GetSynDataFile("agent/SedanAgent.json");

    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank, filename, CONTACT_METHOD);
    agent->Initialize(ChCoordsys<>({(rank - 1) * -10., 3.0 * rank, 0.5}, {1, 0, 0, 0}));

    // if (false) {
    if (rank == 1) {
        // {
        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
        auto vis = chrono_types::make_shared<SynIrrVehicleVisualization>(agent->GetDriver(), 1. / 50);
        vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
        // if (rank == 1)
        vis->SetSave(true);
        vis_manager->AddVisualization(vis);

        auto driver = chrono_types::make_shared<ChIrrGuiDriver>(*vis->GetIrrApp());
        auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver);
        agent->SetBrain(brain);
        vis->SetDriver(driver);

        agent->SetVisualizationManager(vis_manager);
    }

    dds_manager.AddAgent(agent, rank);

    // if (!dds_manager.Initialize(cli.GetAsType<std::string>("ip"), cli.GetAsType<int>("port"))) {
    if (!dds_manager.Initialize()) {
        std::cout << "DDSManager failed to initialize." << std::endl;
        return -1;
    }

    double step = 0;

    std::cout << "Entering simulation loop." << std::endl;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    ChRealtimeStepTimer realtime_timer;
    while (dds_manager.IsOk()) {
        dds_manager.Advance();
        dds_manager.Synchronize();

        if (rank != 1) {
            agent->GetDriver()->SetThrottle(0.4);
            if (agent->GetSystem()->GetChTime() > 5)
                agent->GetDriver()->SetSteering(-0.4);
        }

        // increment the step
        step++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(STEP_SIZE);
    }

    std::cout << END_TIME << std::endl;

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Rank " << rank << ": " << (time_span.count() / 1e3) / END_TIME << std::endl;

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
