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
// Demo for Viper rover cosimulation on SCM terrain. 
// Therobot is co-simulated with an SCM terrain node and a number of tire nodes
// equal to the number of wheels.
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

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimViperNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChIrrApp.h"
#endif

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::viper;

// =============================================================================

class MyDriver : public ViperDriver {
  public:
    MyDriver(double delay) : ViperDriver(), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Update(double time) override {
        // Do not generate any driver inputs for a duration equal to m_delay.
        double eff_time = time - m_delay;
        if (eff_time < 0)
            return;

        double driving = 0;
        double steering = 0;
        if (eff_time > 0.2)
            driving = 0.7;
        else
            driving = 3.5 * eff_time;

        if (eff_time < 2)
            steering = 0;
        else
            steering = 0.6 * std::sin(CH_C_2PI * (eff_time - 2) / 6);
    
        for (int i = 0; i < 4; i++) {
            drive_speeds[i] = driving;
            steer_speeds[i] = steering;
        }
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

    if (num_procs != 6) {
        if (rank == 0)
            std::cout << "\n\nViper cosimulation code must be run on exactly 6 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Simulation parameters
    double step_size = 1e-3;
    double sim_time = 20;
    double output_fps = 100;
    double render_fps = 100;
    bool render = true;
    std::string suffix = "";
    bool verbose = true;

    // Prepare output directory.
    std::string out_dir = GetChronoOutputPath() + "VIPER_COSIM";
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

    // Create the node (a rig node or a terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Viper node  ] rank = " << rank << " running on: " << procname << endl;

        auto driver = chrono_types::make_shared<MyDriver>(0.2);
        auto viper = new ChVehicleCosimViperNode();
        viper->SetDriver(driver);

        viper->SetIntegratorType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED, ChSolver::Type::BARZILAIBORWEIN);
        viper->SetVerbose(verbose);
        viper->SetInitialLocation(ChVector<>(-15, -8, 0.5));
        viper->SetInitialYaw(0);
        viper->SetStepSize(step_size);
        viper->SetNumThreads(1);
        viper->SetOutDir(out_dir, suffix);
        if (verbose)
            cout << "[Viper node  ] output directory: " << viper->GetOutDirName() << endl;

        node = viper;

    } else if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        auto terrain = new ChVehicleCosimTerrainNodeSCM(vehicle::GetDataFile("cosim/scm.json"));
        terrain->SetDimensions(40, 20);
        terrain->SetVerbose(verbose);
        terrain->SetStepSize(step_size);
        terrain->SetNumThreads(2);
        terrain->SetOutDir(out_dir, suffix);
        terrain->EnableRuntimeVisualization(render, render_fps);
        if (verbose)
            cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

        node = terrain;

    } else {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;

        auto tire = new ChVehicleCosimTireNodeRigid(rank - 2);
        ////tire->SetTireFromSpecfile(vehicle::GetDataFile("viper/Viper_RigidTire_real.json"));
        tire->SetTireFromSpecfile(vehicle::GetDataFile("viper/Viper_RigidTire_cyl.json"));
        tire->SetVerbose(verbose);
        tire->SetStepSize(step_size);
        tire->SetNumThreads(1);
        tire->SetOutDir(out_dir, suffix);

        node = tire;
    }

    // Initialize systems.
    node->Initialize();

    // Perform co-simulation.
    int output_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        if (verbose && rank == 0)
            cout << is << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

        node->Synchronize(is, time);
        node->Advance(step_size);
        if (verbose)
            cout << "Node" << rank << " sim time = " << node->GetStepExecutionTime() << "  ["
                 << node->GetTotalExecutionTime() << "]" << endl;

        if (is % output_steps == 0) {
            node->OutputData(output_frame);
            output_frame++;
        }
    }

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}
