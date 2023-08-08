// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo for tracked vehicle cosimulation on SCM terrain.
// The vehicle (specified through a pair of JSON files, one for the vehicle
// itself, the other for the powertrain) is co-simulated with an SCM terrain.
//
// Global reference frame: Z up, X towards the front, and Y pointing to the left
//
// =============================================================================

#include <iostream>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_Vehicle.h"
#include "chrono_models/vehicle/m113/powertrain/M113_EngineShafts.h"
#include "chrono_models/vehicle/m113/powertrain/M113_AutomaticTransmissionShafts.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimTrackedVehicleNode.h"
    #include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeRigid.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.h"
#ifdef CHRONO_MULTICORE
    #include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularOMP.h"
#endif
#ifdef CHRONO_FSI
    #include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.h"
#endif
#ifdef CHRONO_GPU
    #include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularGPU.h"
#endif

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Forward declarations
bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     std::string& terrain_specfile,
                     double& length,
                     double& width,
                     bool& verbose);

// =============================================================================

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        double Ts = 4;
        double Te = 8;

        if (eff_time > 0.2)
            m_throttle = 0.8;
        else
            m_throttle = 4 * eff_time;

        if (eff_time < Ts)
            m_steering = 0;
        else if (eff_time > Te)
            m_steering = 1;
        else
            m_steering = 0.5 + 0.5 * std::sin(CH_C_PI * (eff_time - Ts) / (Te - Ts) - CH_C_PI_2);
    }

  private:
    double m_delay;
};

// =============================================================================

int main(int argc, char** argv) {
    // Initialize MPI.
    int num_procs;
    int rank;
    int name_len;
    char procname[MPI_MAX_PROCESSOR_NAME];

    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Get_processor_name(procname, &name_len);

#ifdef _DEBUG
    if (rank == 0) {
        int foo;
        cout << "Enter something to continue..." << endl;
        cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);
#endif

    if (num_procs != 2) {
        if (rank == 0)
            std::cout << "\n\nTracked vehicle cosimulation code must be run on exactly 2 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Simulation parameters
    std::string terrain_specfile;
    double step_size = 5e-4;
    double sim_time = 20.0;
    double output_fps = 100;
    double render_fps = 100;
    bool renderRT = true;
    bool writeRT = true;
    std::string suffix = "";
    bool verbose = true;

    // If use_JSON_spec=true, use an M113 model specified through JSON files.
    // If use_JSON_spec=false, use an M113 model from the Chrono::Vehicle model library
    bool use_JSON_spec = true;

    // If use_DBP_rig=true, attach a drawbar pull rig to the vehicle
    bool use_DBP_rig = false;

    double terrain_length = 40;
    double terrain_width = 20;

    if (!GetProblemSpecs(argc, argv, rank, terrain_specfile, terrain_length, terrain_width, verbose)) {
        MPI_Finalize();
        return 1;
    }

    ChVector<> init_loc(-terrain_length / 2 + 5, -terrain_width / 2 + 2, 0.9);

    // Overwrite terrain patch size if using a DBP rig
    if (use_DBP_rig) {
        terrain_length = 20;
        terrain_width = 5;
        init_loc = ChVector<>(-5, 0, 0.9);
    }

    // Peek in spec file and extract terrain type
    auto terrain_type = ChVehicleCosimTerrainNodeChrono::GetTypeFromSpecfile(terrain_specfile);
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::UNKNOWN) {
        MPI_Finalize();
        return 1;
    }

    // Prepare output directory.
    std::string out_dir = GetChronoOutputPath() + "TRACKED_VEHICLE_COSIM";
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    MPI_Barrier(MPI_COMM_WORLD);

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));

    // Initialize co-simulation framework.
    cosim::InitializeFramework(0);

    // Create the node (vehicle or terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Vehicle node] rank = " << rank << " running on: " << procname << endl;

        ChVehicleCosimTrackedVehicleNode* vehicle;
        if (use_JSON_spec) {
            vehicle = new ChVehicleCosimTrackedVehicleNode(
                vehicle::GetDataFile("M113/vehicle/M113_Vehicle_SinglePin.json"),
                vehicle::GetDataFile("M113/powertrain/M113_EngineShafts.json"),
                vehicle::GetDataFile("M113/powertrain/M113_AutomaticTransmissionShafts.json"));
        } else {
            auto m113_vehicle = chrono_types::make_shared<m113::M113_Vehicle_SinglePin>(
                false, DrivelineTypeTV::BDS, BrakeType::SIMPLE, false, false, false, nullptr);
            auto m113_engine = chrono_types::make_shared<m113::M113_EngineShafts>("Engine");
            auto m113_transmission =
                chrono_types::make_shared<m113::M113_AutomaticTransmissionShafts>("Transmission");
            auto m113_powertrain = chrono_types::make_shared<ChPowertrainAssembly>(m113_engine, m113_transmission);
            vehicle = new ChVehicleCosimTrackedVehicleNode(m113_vehicle, m113_powertrain);
        }

        if (use_DBP_rig) {
            auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
            double base_vel = 1;
            double slip = 0;
            auto dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
            vehicle->AttachDrawbarPullRig(dbp_rig);
        }

        auto driver = chrono_types::make_shared<MyDriver>(*vehicle->GetVehicle(), 0.5);
        ////vehicle->SetChassisFixed(true);
        vehicle->SetDriver(driver);
        vehicle->SetVerbose(verbose);
        vehicle->SetInitialLocation(init_loc);
        vehicle->SetInitialYaw(0);
        vehicle->SetStepSize(step_size);
        vehicle->SetNumThreads(1);
        vehicle->SetOutDir(out_dir, suffix);
        if (renderRT)
            vehicle->EnableRuntimeVisualization(render_fps, writeRT);
        vehicle->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
        if (verbose)
            cout << "[Vehicle node] output directory: " << vehicle->GetOutDirName() << endl;

        node = vehicle;

    } else if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        switch (terrain_type) {
            case ChVehicleCosimTerrainNodeChrono::Type::RIGID: {
                auto method = ChContactMethod::SMC;
                auto terrain = new ChVehicleCosimTerrainNodeRigid(method, terrain_specfile);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                node = terrain;
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::SCM: {
                auto terrain = new ChVehicleCosimTerrainNodeSCM(vehicle::GetDataFile("cosim/terrain/scm_hard.json"));
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetNumThreads(2);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                node = terrain;
                break;
            }
        }
    }

    // Initialize systems
    // (perform initial inter-node data exchange)
    node->Initialize();

    // Perform co-simulation
    // (perform synchronization inter-node data exchange)
    int output_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        if (verbose && rank == 0)
            cout << is << "  " << time << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

        node->Synchronize(is, time);
        node->Advance(step_size);
        if (verbose)
            cout << "Node" << rank << " sim time = " << node->GetStepExecutionTime() << "  ["
                 << node->GetTotalExecutionTime() << "]" << endl;

        if (is % output_steps == 0) {
            node->OutputData(output_frame);
            node->OutputVisualizationData(output_frame);
            output_frame++;
        }
    }

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     std::string& terrain_specfile,
                     double& length,
                     double& width,
                     bool& verbose) {
    ChCLI cli(argv[0], "Tracked vehicle co-simulation (run on 2 MPI ranks)");

    cli.AddOption<std::string>("", "terrain_specfile", "Terrain specification file [JSON format]");
    cli.AddOption<double>("", "terrain_length", "Terrain length [m]", std::to_string(length));
    cli.AddOption<double>("", "terrain_width", "Terrain width [m]", std::to_string(width));

    cli.AddOption<bool>("", "quiet", "Disable verbose messages");

    if (!cli.Parse(argc, argv)) {
        if (rank == 0)
            cli.Help();
        return false;
    }

    try {
        terrain_specfile = cli.Get("terrain_specfile").as<std::string>();
    } catch (std::domain_error&) {
        if (rank == 0) {
            cout << "\nERROR: Missing terrain specification file!\n\n" << endl;
            cli.Help();
        }
        return false;
    }

    length = cli.GetAsType<double>("terrain_length");
    width = cli.GetAsType<double>("terrain_width");

    verbose = !cli.GetAsType<bool>("quiet");

    return true;
}
