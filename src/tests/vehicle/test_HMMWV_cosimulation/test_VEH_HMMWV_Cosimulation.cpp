// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// MAIN DRIVER
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <iostream>
#include <string>
#include <algorithm>
#include <omp.h>
#include "mpi.h"

#include "chrono/core/ChFileutils.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "VehicleNode.h"
#include "TerrainNode.h"
#include "TireNode.h"

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Cosimulation step size
double step_size = 4e-5;

// Output frequency (frames per second)
double output_fps = 200;

// Checkpointing frequency (frames per second)
double checkpoint_fps = 100;

// Output directory
std::string out_dir = "../HMMWV_COSIM";

// =============================================================================

// ID values to identify command line arguments
enum {
    OPT_HELP,
    OPT_THREADS_TIRE,
    OPT_THREADS_TERRAIN,
    OPT_USE_CHECKPOINT,
    OPT_SIM_TIME,
    OPT_NO_OUTPUT,
    OPT_NO_RENDERING,
    OPT_COHESION,
    OPT_SUFFIX
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_THREADS_TIRE, "--num-threads-tire", SO_REQ_CMB},
                                    {OPT_THREADS_TERRAIN, "--num-threads-terrain", SO_REQ_CMB},
                                    {OPT_USE_CHECKPOINT, "-c", SO_NONE},
                                    {OPT_USE_CHECKPOINT, "--use-checkpoint", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "-t", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "--simulation-time", SO_REQ_CMB},
                                    {OPT_NO_OUTPUT, "--no-output", SO_NONE},
                                    {OPT_NO_RENDERING, "--no-rendering", SO_NONE},
                                    {OPT_COHESION, "-ch", SO_REQ_CMB},
                                    {OPT_COHESION, "--cohesion-terrain", SO_REQ_CMB},
                                    {OPT_SUFFIX, "--suffix", SO_REQ_CMB},
                                    {OPT_HELP, "-?", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_HELP, "--help", SO_NONE},
                                    SO_END_OF_OPTIONS};

// Forward declarations
void ShowUsage();
bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& nthreads_tire,
                     int& nthreads_terrain,
                     double& sim_time,
                     double& cohesion,
                     bool& use_checkpoint,
                     bool& output,
                     bool& render,
                     std::string& suffix);

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

    // Ensure we are running exactly 6 nodes
    if (num_procs != 6) {
        if (rank == 0)
            cout << "Must use exactly 6 nodes" << endl;
        MPI_Finalize();
        return 1;
    }

    // Parse command line arguments
    int nthreads_tire = 2;
    int nthreads_terrain = 2;
    double sim_time = 10;
    double coh_pressure = 8e4;
    bool use_checkpoint = false;
    bool output = true;
    bool render = true;
    std::string suffix = "";
    if (!GetProblemSpecs(argc, argv, rank, nthreads_tire, nthreads_terrain, sim_time, coh_pressure,
                         use_checkpoint, output, render, suffix)) {
        MPI_Finalize();
        return 1;
    }

    // Prepare output directory.
    if (rank == 0) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }
    }

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));
    int checkpoint_steps = (int)std::ceil(1 / (checkpoint_fps * step_size));

    // Create the systems and run the settling phase for terrain.
    VehicleNode* my_vehicle = NULL;
    TerrainNode* my_terrain = NULL;
    TireNode* my_tire = NULL;

    switch (rank) {
        case VEHICLE_NODE_RANK: {
            my_vehicle = new VehicleNode();
            my_vehicle->SetStepSize(step_size);
            my_vehicle->SetOutDir(out_dir, suffix);
            my_vehicle->SetChassisFixed(false);
            cout << my_vehicle->GetPrefix() << " rank = " << rank << " running on: " << procname << endl;
            cout << my_vehicle->GetPrefix() << " output directory: " << my_vehicle->GetOutDirName() << endl;

            break;
        }
        case TERRAIN_NODE_RANK: {
            auto type = TerrainNode::RIGID;
            auto method = ChMaterialSurfaceBase::DEM;

            my_terrain = new TerrainNode(type, method, 4, use_checkpoint, render, nthreads_terrain);
            my_terrain->SetStepSize(step_size);
            my_terrain->SetOutDir(out_dir, suffix);
            cout << my_terrain->GetPrefix() << " rank = " << rank << " running on: " << procname << endl;
            cout << my_terrain->GetPrefix() << " output directory: " << my_terrain->GetOutDirName() << endl;

            my_terrain->SetContainerDimensions(10, 3, 1, 0.2);

            double radius = 0.006;
            double coh_force = CH_C_PI * radius * radius * coh_pressure;

            switch (method) {
                case ChMaterialSurfaceBase::DEM: {
                    auto material = std::make_shared<ChMaterialSurfaceDEM>();
                    material->SetFriction(0.9f);
                    material->SetRestitution(0.0f);
                    material->SetYoungModulus(8e5f);
                    material->SetPoissonRatio(0.3f);
                    material->SetAdhesion(coh_force);
                    material->SetKn(1.0e6f);
                    material->SetGn(6.0e1f);
                    material->SetKt(4.0e5f);
                    material->SetGt(4.0e1f);
                    my_terrain->SetMaterialSurface(material);
                    my_terrain->UseMaterialProperties(false);
                    my_terrain->SetContactForceModel(ChSystemDEM::PlainCoulomb);
                    break;
                }
                case ChMaterialSurfaceBase::DVI: {
                    auto material = std::make_shared<ChMaterialSurface>();
                    material->SetFriction(0.9f);
                    material->SetRestitution(0.0f);
                    material->SetCohesion(coh_force);
                    my_terrain->SetMaterialSurface(material);
                    break;
                }
            }

            switch (type) {
                case TerrainNode::RIGID:
                    my_terrain->SetProxyProperties(1, 0.01, false);
                    break;
                case TerrainNode::GRANULAR:
                    my_terrain->SetProxyProperties(1, false);
                    my_terrain->SetGranularMaterial(radius, 2500, 6);
                    my_terrain->SetSettlingTime(0.2);
                    ////my_terrain->EnableSettlingOutput(true);
                    my_terrain->Settle();
                    break;
            }

            break;
        }
        case TIRE_NODE_RANK(0):
        case TIRE_NODE_RANK(1):
        case TIRE_NODE_RANK(2):
        case TIRE_NODE_RANK(3): {
            my_tire = new TireNode(WheelID(rank - 2), nthreads_tire);
            my_tire->SetStepSize(step_size);
            my_tire->SetOutDir(out_dir, suffix);
            cout << my_tire->GetPrefix() << " rank = " << rank << " running on: " << procname << endl;
            cout << my_tire->GetPrefix() << " output directory: " << my_tire->GetOutDirName() << endl;

            my_tire->SetTireJSONFile(vehicle::GetDataFile("hmmwv/tire/HMMWV_ANCFTire.json"));
            my_tire->SetProxyProperties(45, ChVector<>(0.113, 0.113, 0.113), false);
            my_tire->EnableTirePressure(true);

            break;
        }
    }

    // Initialize systems.
    // Data exchange:
    //   terrain => vehicle (initial terrain height)
    //   vehicle => tire (initial wheel position)
    //   tire => terrain (tire mesh topology information)
    //   tire => terrain (tire contact material properties)
    switch (rank) {
        case VEHICLE_NODE_RANK:
            my_vehicle->Initialize();
            break;
        case TERRAIN_NODE_RANK:
            my_terrain->Initialize();
            break;
        case TIRE_NODE_RANK(0):
        case TIRE_NODE_RANK(1):
        case TIRE_NODE_RANK(2):
        case TIRE_NODE_RANK(3):
            my_tire->Initialize();
            break;
    }

    // Perform co-simulation.
    // At synchronization, there is bi-directional data exchange:
    //     tire => terrain (vertex state information)
    //     terrain => tire (vertex force information)
    //     tire => vehicle (wheel force)
    //     vehicle => tire (wheel state)
    int output_frame = 0;
    int checkpoint_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        MPI_Barrier(MPI_COMM_WORLD);

        switch (rank) {
            case VEHICLE_NODE_RANK: {
                cout << is << " ---------------------------- " << endl;
                my_vehicle->Synchronize(is, time);
                cout << " --- " << endl;

                my_vehicle->Advance(step_size);
                cout << my_vehicle->GetPrefix() << " sim time =    " << my_vehicle->GetSimTime() << "  ["
                     << my_vehicle->GetTotalSimTime() << "]" << endl;

                if (output && is % output_steps == 0) {
                    my_vehicle->OutputData(output_frame);
                    output_frame++;
                }

                break;
            }
            case TERRAIN_NODE_RANK: {
                my_terrain->Synchronize(is, time);
                my_terrain->Advance(step_size);
                cout << my_terrain->GetPrefix() << " sim time = " << my_terrain->GetSimTime() << "  ["
                     << my_terrain->GetTotalSimTime() << "]" << endl;

                if (output && is % output_steps == 0) {
                    my_terrain->OutputData(output_frame);
                    output_frame++;
                }

                if (is % checkpoint_steps == 0) {
                    my_terrain->WriteCheckpoint();
                    checkpoint_frame++;
                }

                break;
            }
            case TIRE_NODE_RANK(0):
            case TIRE_NODE_RANK(1):
            case TIRE_NODE_RANK(2):
            case TIRE_NODE_RANK(3): {
                my_tire->Synchronize(is, time);
                my_tire->Advance(step_size);
                cout << my_tire->GetPrefix() << " sim time =    " << my_tire->GetSimTime() << "  ["
                     << my_tire->GetTotalSimTime() << "]" << endl;

                if (output && is % output_steps == 0) {
                    my_tire->OutputData(output_frame);
                    output_frame++;
                }
                break;
            }
        }
    }

    // Cleanup.
    delete my_vehicle;
    delete my_terrain;
    delete my_tire;

    MPI_Finalize();

    return 0;
}

// =============================================================================

void ShowUsage() {
    cout << "Usage:  mpiexec -np 2 test_VEH_tireRig_Cosimulation [OPTIONS]" << endl;
    cout << endl;
    cout << " --num-threads-rig=NUM_THREADS_RIG" << endl;
    cout << "        Specify number of OpenMP threads for the rig node [default: 2]" << endl;
    cout << " --num-threads-terrain=NUM_THREADS_TERRAIN" << endl;
    cout << "        Specify number of OpenMP threads for the terrain node [default: 2]" << endl;
    cout << " -c" << endl;
    cout << " --use-checkpoint" << endl;
    cout << "        Initialize granular terrain from checkppoint file" << endl;
    cout << "        If not specified, the granular material is settled through simulation" << endl;
    cout << " -t=SIM_TIME" << endl;
    cout << " --simulation-time=SIM_TIME" << endl;
    cout << "        Specify simulation length in seconds [default: 10]" << endl;
    cout << " -ch=COHESION" << endl;
    cout << " --cohesion-terrain=COHESION" << endl;
    cout << "        Specify the value of the terrain cohesion in Pa [default: 80e3]" << endl;
    cout << " --no-output" << endl;
    cout << "        Disable generation of output files" << endl;
    cout << " --no-rendering" << endl;
    cout << "        Disable OpenGL rendering" << endl;
    cout << " --suffix=SUFFIX" << endl;
    cout << "        Specify suffix for output directory names [default: \"\"]" << endl;
    cout << " -? -h --help" << endl;
    cout << "        Print this message and exit." << endl;
    cout << endl;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& nthreads_tire,
                     int& nthreads_terrain,
                     double& sim_time,
                     double& cohesion,
                     bool& use_checkpoint,
                     bool& output,
                     bool& render,
                     std::string& suffix) {
    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            if (rank == 0) {
                cout << "Invalid argument: " << args.OptionText() << endl;
                ShowUsage();
            }
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                if (rank == 0) {
                    ShowUsage();
                }
                return false;
            case OPT_THREADS_TIRE:
                nthreads_tire = std::stoi(args.OptionArg());
                break;
            case OPT_THREADS_TERRAIN:
                nthreads_terrain = std::stoi(args.OptionArg());
                break;
            case OPT_SIM_TIME:
                sim_time = std::stod(args.OptionArg());
                break;
            case OPT_COHESION:
                cohesion = std::stod(args.OptionArg());
                break;
            case OPT_NO_OUTPUT:
                output = false;
                break;
            case OPT_NO_RENDERING:
                render = false;
                break;
            case OPT_USE_CHECKPOINT:
                use_checkpoint = true;
                break;
            case OPT_SUFFIX:
                suffix = args.OptionArg();
                break;
        }
    }

    return true;
}
