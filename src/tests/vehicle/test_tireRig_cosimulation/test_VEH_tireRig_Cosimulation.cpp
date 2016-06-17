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
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a Chrono::Parallel system for the granular terrain.
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
#include "thirdparty/SimpleOpt/SimpleOpt.h"

#include "Settings.h"
#include "RigNode.h"
#include "TerrainNode.h"

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// ID values to identify command line arguments
enum {
    OPT_HELP,
    OPT_THREADS_RIG,
    OPT_THREADS_TERRAIN,
    OPT_USE_CHECKPOINT,
    OPT_SIM_TIME,
    OPT_NO_OUTPUT,
    OPT_NO_RENDERING,
    OPT_INIT_VEL,
    OPT_LONG_SLIP,
    OPT_SUFFIX
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_THREADS_RIG, "--num-threads-rig", SO_REQ_CMB},
                                    {OPT_THREADS_TERRAIN, "--num-threads-terrain", SO_REQ_CMB},
                                    {OPT_USE_CHECKPOINT, "-c", SO_NONE},
                                    {OPT_USE_CHECKPOINT, "--use-checkpoint", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "-t", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "--simulation-time", SO_REQ_CMB},
                                    {OPT_NO_OUTPUT, "--no-output", SO_NONE},
                                    {OPT_NO_RENDERING, "--no-rendering", SO_NONE},
                                    {OPT_INIT_VEL, "-v", SO_REQ_CMB},
                                    {OPT_INIT_VEL, "--initial-velocity", SO_REQ_CMB},
                                    {OPT_LONG_SLIP, "-s", SO_REQ_CMB},
                                    {OPT_LONG_SLIP, "--longitudinal-slip", SO_REQ_CMB},
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
                     int& nthreads_rig,
                     int& nthreads_terrain,
                     double& sim_time,
                     double& init_vel,
                     double& slip,
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

    // Parse command line arguments
    int nthreads_rig = 2;
    int nthreads_terrain = 2;
    double sim_time = 10;
    double init_vel = 0;
    double slip = 0;
    bool use_checkpoint = false;
    bool output = true;
    bool render = true;
    std::string suffix = "";
    if (!GetProblemSpecs(argc, argv, rank, nthreads_rig, nthreads_terrain, sim_time, init_vel, slip, use_checkpoint,
                         output, render, suffix)) {
        MPI_Finalize();
        return 1;
    }

    // Append suffix to output directories
    rig_dir = rig_dir + suffix;
    terrain_dir = terrain_dir + suffix;

    // Prepare output directories.
    if (rank == 0) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }
        if (ChFileutils::MakeDirectory(rig_dir.c_str()) < 0) {
            cout << "Error creating directory " << rig_dir << endl;
            return 1;
        }
        if (ChFileutils::MakeDirectory(terrain_dir.c_str()) < 0) {
            cout << "Error creating directory " << terrain_dir << endl;
            return 1;
        }
    }

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));
    int checkpoint_steps = (int)std::ceil(1 / (checkpoint_fps * step_size));

    // Create the two systems and run settling phase for terrain.
    // Data exchange:
    //   rig => terrain (tire contact material properties)
    RigNode* my_rig = NULL;
    TerrainNode* my_terrain = NULL;

    switch (rank) {
        case RIG_NODE_RANK:
            cout << "[Rig node    ] rank = " << rank << " running on: " << procname << endl;
            my_rig = new RigNode(init_vel, slip, nthreads_rig);
            if (output) {
                my_rig->SetOutputFile(rig_dir + "/rig_results.txt");
            }
            break;
        case TERRAIN_NODE_RANK:
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;
            my_terrain = new TerrainNode(TerrainNode::GRANULAR, ChMaterialSurfaceBase::DEM, use_checkpoint, render, nthreads_terrain);
            if (output) {
                ////my_terrain->SetOutputFile(terrain_dir + "/terrain_results.txt");
            }
            my_terrain->Settle();
            break;
    }

    // Initialize systems.
    // Data exchange:
    //   terrain => rig (terrain height)
    //   rig => terrain (tire mesh topology information)
    switch (rank) {
        case RIG_NODE_RANK:
            my_rig->Initialize();
            break;
        case TERRAIN_NODE_RANK:
            my_terrain->Initialize();
            break;
    }

    // Perform co-simulation.
    // At synchronization, there is bi-directional data exchange:
    //     rig => terrain (position information)
    //     terrain => rig (force information)
    int output_frame = 0;
    int checkpoint_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        MPI_Barrier(MPI_COMM_WORLD);

        switch (rank) {
            case RIG_NODE_RANK: {
                cout << is << " ---------------------------- " << endl;
                my_rig->Synchronize(is, time);
                cout << " --- " << endl;

                my_rig->Advance(step_size);
                cout << "Tire sim time =    " << my_rig->GetSimTime() << "  [" << my_rig->GetTotalSimTime() << "]"
                     << endl;

                if (output && is % output_steps == 0) {
                    my_rig->OutputData(output_frame);
                    output_frame++;
                }

                break;
            }
            case TERRAIN_NODE_RANK: {
                my_terrain->Synchronize(is, time);
                my_terrain->Advance(step_size);
                cout << "Terrain sim time = " << my_terrain->GetSimTime() << "  [" << my_terrain->GetTotalSimTime()
                     << "]" << endl;

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
        }
    }

    // Cleanup.
    delete my_rig;
    delete my_terrain;

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
    cout << " -v=INIT_VEL" << endl;
    cout << " --initial-velocity=INIT_VEL" << endl;
    cout << "        Specify the initial tire linear velocity [default: 0]" << endl;
    cout << " -s=LONG_SLIP" << endl;
    cout << " --longitudinal-slip=LONG_SLIP" << endl;
    cout << "        Specify the value of the longitudinal slip [default: 0]" << endl;
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
                     int& nthreads_rig,
                     int& nthreads_terrain,
                     double& sim_time,
                     double& init_vel,
                     double& slip,
                     bool& use_checkpoint,
                     bool& output,
                     bool& render,
                     std::string& suffix
                     ) {
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
            case OPT_THREADS_RIG:
                nthreads_rig = std::stoi(args.OptionArg());
                break;
            case OPT_THREADS_TERRAIN:
                nthreads_terrain = std::stoi(args.OptionArg());
                break;
            case OPT_SIM_TIME:
                sim_time = std::stod(args.OptionArg());
                break;
            case OPT_INIT_VEL:
                init_vel = std::stod(args.OptionArg());
                break;
            case OPT_LONG_SLIP:
                slip = std::stod(args.OptionArg());
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
