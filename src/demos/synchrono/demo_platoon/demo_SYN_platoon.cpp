#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include <chrono>

using namespace chrono;
using namespace chrono::synchrono;

// =============================================================================

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

    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
    mpi_manager.AddAgent(agent, rank);
    // -------
    // Vehicle
    // -------
    // Grid of vehicles
    int col = (rank - 1) % 3;
    int row = rank / 3;

    // Box dimensions
    double length = 400;
    double width = 25;

    ChVector<double> base = ChVector<>({-length / 2 + 5, -width / 2 + 5, 1.0});
    ChVector<double> offset = ChVector<>({30.0 * row, 5.0 * col, 0});
    ChVector<double> init_loc = base + offset;

    ChQuaternion<> initRot = ChQuaternion<>({1, 0, 0, 0});

    std::string vehicle_filename = GetSynDataFile("vehicle/Sedan.json");
    auto vehicle = chrono_types::make_shared<SynWheeledVehicle>(vehicle_filename, CONTACT_METHOD);
    vehicle->Initialize(ChCoordsys<>(base + offset, initRot));
    agent->SetVehicle(vehicle);

    // -------
    // Terrain
    // -------
    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());

    auto patch = terrain->AddPatch(DefaultMaterialSurface(), ChVector<>(), ChVector<>(0, 0, 1), length, width);
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

    std::vector<ChVector<double>> path_points = {init_loc + ChVector<double>(0, 0, 0),
                                                 init_loc + ChVector<double>(length, 0, 0)};
    std::shared_ptr<ChBezierCurve> path = chrono_types::make_shared<ChBezierCurve>(path_points);

    auto driver = chrono_types::make_shared<ChPathFollowerDriver>(vehicle->GetVehicle(), path, "Box path", 10);

    driver->GetSpeedController().SetGains(0.4, 0, 0);
    driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver->GetSteeringController().SetLookAheadDistance(5);

    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, agent->GetChVehicle());
    agent->SetBrain(brain);

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
        auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();
        sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
        sen_vis->InitializeAsDefaultChaseCamera(agent->GetChVehicle().GetChassisBody());
        if (cli.GetAsType<bool>("sens_save"))
            sen_vis->AddFilterSave();
        if (cli.GetAsType<bool>("sens_vis"))
            sen_vis->AddFilterVisualize();
        vis_manager->AddVisualization(sen_vis);
    }
#endif

    mpi_manager.Barrier();
    mpi_manager.Initialize();
    mpi_manager.Barrier();

    double step = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance();
        mpi_manager.Synchronize();
        mpi_manager.Update();

        // increment the step
        step++;
    }

    if (VERBOSE && rank == 0) {
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Total Wall Time: " << time_span.count() / 1e6 << "." << std::endl;
        std::cout << "Fraction of real time: " << (time_span.count() / 1e6) / END_TIME << std::endl;
        std::cout << "Frequency of steps [Hz]: " << step / (time_span.count() / 1e6) << std::endl;
        std::cout << "Real time: " << (time_span.count() / 1e6) / END_TIME << std::endl;
    }

    return 0;
}
