// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// A 3-lane grid of vehicles travels down a flat roadway. Only a block terrain
// is used, not a mesh. Very lightweight demo, useful for scaling analyses.
//
// =============================================================================

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
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
using namespace chrono::vehicle;
using namespace chrono::synchrono;

// =============================================================================

const ChContactMethod contact_method = ChContactMethod::NSC;

double end_time = 1000;   // [s]
double step_size = 3e-3;  // [s]

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
float heartbeat = 1e-2;  // 100 [Hz]

void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);

    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------
    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);

    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");

    const bool use_sensor_vis = cli.HasValueInVector<int>("sens", rank);
    const bool use_irrlicht_vis = !use_sensor_vis && cli.HasValueInVector<int>("irr", rank);

    mpi_manager.SetHeartbeat(heartbeat);
    mpi_manager.SetEndTime(end_time);

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
    int col = rank % 3;
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

    // ---------------------------
    // Controller for the vehicles
    // ---------------------------

    // Drive in a straight line
    std::vector<ChVector<>> curve_pts = {init_loc, init_loc + ChVector<>(1000, 0, 0)};
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);
    auto driver = chrono_types::make_shared<ChPathFollowerDriver>(vehicle->GetVehicle(), path, "Box path", 10);

    // Reasonable defaults for the underlying PID
    driver->GetSpeedController().SetGains(0.4, 0, 0);
    driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver->GetSteeringController().SetLookAheadDistance(5);

    // Wrap the ChDriver in a SynVehicleBrain and add it to our agent
    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, agent->GetChVehicle());
    agent->SetBrain(brain);

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->SetVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (use_irrlicht_vis) {
        // Add an irrlicht visualization
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver, step_size, render_step_size);
        irr_vis->SetRenderStepSize(render_step_size);
        irr_vis->SetStepSize(step_size);
        irr_vis->InitializeAsDefaultChaseCamera(vehicle);
        vis_manager->AddVisualization(irr_vis);
    }
#endif

#ifdef CHRONO_SENSOR
    if (use_sensor_vis) {
        auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();
        sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
        sen_vis->InitializeAsDefaultChaseCamera(agent->GetChVehicle().GetChassisBody());

        if (cli.GetAsType<bool>("sens_vis"))
            sen_vis->AddFilterVisualize();

        if (cli.GetAsType<bool>("sens_save")) {
            std::string path = std::string("SENSOR_OUTPUT/platoon") + std::to_string(rank) + std::string("/");
            sen_vis->AddFilterSave(path);
        }

        vis_manager->AddVisualization(sen_vis);
    }
#endif

    mpi_manager.Barrier();
    mpi_manager.Initialize();

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

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");
}
