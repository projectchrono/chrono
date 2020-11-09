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

    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
    agent->SetStepSize(step_size);
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

    std::string vehicle_filename = synchrono::GetDataFile("vehicle/Sedan.json");
    ChCoordsys<> init_pos(base + offset, initRot);
    auto vehicle = chrono_types::make_shared<SynWheeledVehicle>(init_pos, vehicle_filename, contact_method);
    agent->SetVehicle(vehicle);

    // -------
    // Terrain
    // -------
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());
    auto patch = terrain->AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), length, width);
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
        // Add an irrlicht visualization
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>();
        irr_vis->SetRenderStepSize(render_step_size);
        irr_vis->SetStepSize(step_size);
        irr_vis->InitializeAsDefaultChaseCamera(vehicle);
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

    double step = 0;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance(heartbeat * step);
        mpi_manager.Synchronize();
        mpi_manager.Update();

        // increment the step
        step++;
    }

    if (rank == 0) {
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Total Wall Time: " << time_span.count() / 1e6 << "." << std::endl;
        std::cout << "Fraction of real time: " << (time_span.count() / 1e6) / end_time << std::endl;
        std::cout << "Frequency of steps [Hz]: " << step / (time_span.count() / 1e6) << std::endl;
        std::cout << "Real time: " << (time_span.count() / 1e6) / end_time << std::endl;
    }

    return 0;
}
