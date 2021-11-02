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
// Demo for single-wheel rig cosimulation framework using a custom terrain node.
// Illustrates use of a terrain simulation potentially done oustside Chrono.
//
// Global reference frame: Z up, X front, and Y left.
//
// =============================================================================

#include <iostream>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularMPI.h"

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

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

    if (num_procs < 3) {
        if (rank == 0)
            std::cout << "\n\nSingle wheel cosimulation code must be run on at least 3 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Simulation parameters
    double step_size = 1e-4;
    double sim_time = 10;
    double output_fps = 100;
    double render_fps = 100;
    bool render = true;
    std::string suffix = "";
    bool verbose = true;

    // Prepare output directory.
    std::string out_dir = GetChronoOutputPath() + "RIG_COSIM_DISTRIBUTED";
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

    // Initialize co-simulation framework (specify 1 tire node).
    cosim::InitializeFramework(1);

    // Create the node (a rig, tire, or terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        auto rig_type = ChVehicleCosimDBPRig::Type::IMPOSED_SLIP;
        std::shared_ptr<ChVehicleCosimDBPRig> dbp_rig;
        switch (rig_type) {
            case ChVehicleCosimDBPRig::Type::IMPOSED_SLIP: {
                auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
                double base_vel = 1.0;
                double slip = 0;
                dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
                break;
            }
            case ChVehicleCosimDBPRig::Type::IMPOSED_ANG_VEL: {
                double ang_speed = 1;
                double force_rate = 40;
                dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedAngVel>(ang_speed, force_rate);
                break;
            }
        }
        dbp_rig->SetDBPFilterWindow(0.1);

        auto mbs = new ChVehicleCosimRigNode();
        mbs->SetVerbose(verbose);
        mbs->SetStepSize(step_size);
        mbs->SetNumThreads(1);
        mbs->SetTotalMass(100);
        mbs->SetOutDir(out_dir, suffix);
        mbs->AttachDrawbarPullRig(dbp_rig);

        node = mbs;
    } else if (rank == TIRE_NODE_RANK(0)) {
        std::string tire_specfile("../data/vehicle/hmmwv/tire/HMMWV_RigidMeshTire_CoarseClosed.json");
        auto tire = new ChVehicleCosimTireNodeRigid(0);
        tire->SetTireFromSpecfile(tire_specfile);
        tire->SetVerbose(verbose);
        tire->SetStepSize(step_size);
        tire->SetNumThreads(1);
        tire->SetOutDir(out_dir, suffix);

        node = tire;
    } else {
        std::string terrain_specfile("../data/vehicle/cosim/terrain/granular_mpi.json");
        int nthreads_terrain = 2;
        auto terrain = new ChVehicleCosimTerrainNodeGranularMPI(terrain_specfile);
        terrain->SetVerbose(verbose);
        terrain->SetStepSize(step_size);
        terrain->SetNumThreads(nthreads_terrain);
        terrain->SetOutDir(out_dir, suffix);
        terrain->EnableRuntimeVisualization(render, render_fps);

        node = terrain;
    }

    // Initialize systems.
    node->Initialize();

    if (verbose) {
        if (rank == 0)
            cout << "---------------------------- " << endl;
        for (int i = 0; i < num_procs; i++) {
            if (rank == i) {
                cout << "rank: " << rank << " running on: " << procname << endl;
                cout << "   node type:    " << node->GetNodeTypeString() << endl;
                cout << "   cosim node:   " << (node->IsCosimNode() ? "yes" : "no") << endl;
                cout << "   output dir:   " << node->GetOutDirName() << endl;
            }
            MPI_Barrier(MPI_COMM_WORLD);
        }
    }

    // Perform co-simulation.
    int output_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        if (verbose && rank == 0)
            cout << is << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

        node->Synchronize(is, time);
        node->Advance(step_size);

        if (node->IsCosimNode()) {
            if (verbose)
                cout << "Node" << rank << " sim time = " << node->GetStepExecutionTime() << "  ["
                     << node->GetTotalExecutionTime() << "]" << endl;

            if (is % output_steps == 0) {
                node->OutputData(output_frame);
                output_frame++;
            }
        }
    }

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}
