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

#include <omp.h>
#include "mpi.h"

#include "chrono/core/ChFileutils.h"

#include "Settings.h"
#include "RigNode.h"
#include "TerrainNode.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

int main() {
    // Initialize MPI.
    int num_procs;
    int rank;
    MPI_Init(NULL, NULL);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

#ifdef _DEBUG
    if (rank == 0) {
        int foo;
        std::cout << "Enter something to continue..." << std::endl;
        std::cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);
#endif

    // Prepare output directories.
    if (rank == 0) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (ChFileutils::MakeDirectory(rig_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << rig_dir << std::endl;
            return 1;
        }
        if (ChFileutils::MakeDirectory(terrain_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << terrain_dir << std::endl;
            return 1;
        }
    }

    // Number of simulation steps between miscellaneous events.
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));
    int checkpoint_steps = (int)std::ceil(1 / (checkpoint_fps * step_size));

    // Create the two systems and run settling phase for terrain.
    // Data exchange:
    //   rig => terrain (tire contact material properties)
    RigNode* my_rig = NULL;
    TerrainNode* my_terrain = NULL;

    switch (rank) {
        case RIG_NODE_RANK:
            my_rig = new RigNode(nthreads_rignode);
            my_rig->SetOutputFile(rig_dir + "/rig_results.txt");
            break;
        case TERRAIN_NODE_RANK:
            my_terrain = new TerrainNode(TerrainNode::GRANULAR, ChMaterialSurfaceBase::DEM, nthreads_terrainnode);
            ////my_terrain->SetOutputFile(terrain_dir + "/terrain_results.txt");
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

    for (int is = 0; is < num_steps; is++) {
        double time = is * step_size;

        MPI_Barrier(MPI_COMM_WORLD);

        switch (rank) {
            case RIG_NODE_RANK: {
                std::cout << is << " ---------------------------- " << std::endl;
                my_rig->Synchronize(is, time);
                std::cout << " --- " << std::endl;

                my_rig->Advance(step_size);
                std::cout << "Tire sim time =    " << my_rig->GetSimTime() << "  [" << my_rig->GetTotalSimTime() << "]"
                          << std::endl;

                if (is % output_steps == 0) {
                    my_rig->OutputData(output_frame);
                    output_frame++;
                }

                if (phase == SETTLING && is % checkpoint_steps == 0) {
                    my_rig->WriteCheckpoint();
                    checkpoint_frame++;
                }

                break;
            }
            case TERRAIN_NODE_RANK: {
                my_terrain->Synchronize(is, time);
                my_terrain->Advance(step_size);
                std::cout << "Terrain sim time = " << my_terrain->GetSimTime() << "  [" << my_terrain->GetTotalSimTime()
                          << "]" << std::endl;

                if (is % output_steps == 0) {
                    my_terrain->OutputData(output_frame);
                    output_frame++;
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