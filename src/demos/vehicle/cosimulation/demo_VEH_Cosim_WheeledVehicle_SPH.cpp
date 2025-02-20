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
// Demo for Polaris wheeled vehicle cosimulation on CRM terrain.
// The vehicle (specified through JSON files, for the vehicle, engine, and
// transmission) is co-simulated with a terrain node and a number of tire
// nodes equal to the number of wheels.
//
// Global reference frame: Z up, X towards the front, and Y pointing to the left
//
// =============================================================================

#include <iostream>
#include <iomanip>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimWheeledVehicleNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.h"

#undef CHRONO_MUMPS
#include "demos/SetChronoSolver.h"

using std::cout;
using std::cin;
using std::endl;
using std::flush;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

int main(int argc, char** argv) {
    // Initialize MPI
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
            std::cout << "\n\n4-wheel vehicle cosimulation code must be run on exactly 6 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Simulation parameters
    // - sim_time:        total simulation time
    // - step_cosim:      co-simulation meta-step (controls frequency of data exchange)
    // - step_mbs:        step size for vehicle dynamics
    // - step_terrain:    step size for FSI terrain simulation
    // - step_rigid_tire: step size for rigid tire dynamics
    // - step_fea_tire:   step size for flexible tire dynamics
    double sim_time = 20.0;

    double step_cosim = 1e-3;
    double step_mbs = 1e-4;
    double step_terrain = 1e-4;
    double step_rigid_tire = 1e-4;
    double step_fea_tire = 1e-4;

    bool fix_chassis = false;

    // Output and rendering frequency (in FPS)
    double output_fps = 100;
    double render_fps = 100;

    // Visualization flags
    // - verbose:      enable verbose terminal output
    // - output:       generate output files
    // - renderRT:     enable run-time visualization
    // - writeRT:      save snapshots from run-time visualization
    // - writePP:      save data files for Blender post-processing
    // - render_tires: enable run-time and post-processing for individual tires
    bool verbose = false;
    bool output = false;
    bool renderRT = true;
    bool writeRT = false;
    bool writePP = false;
    bool render_tire[4] = {true, false, true, false};

    std::string terrain_specfile = "cosim/terrain/granular_sph.json";

    std::string vehicle_specfile = "Polaris/Polaris.json";
    std::string engine_specfile = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_specfile = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
    std::string tire_specfile = "Polaris/Polaris_RigidMeshTire.json";
    ////std::string tire_specfile = "Polaris/Polaris_ANCF4Tire_Lumped.json";
    ////std::string tire_specfile = "Polaris/Polaris_ANCF8Tire_Lumped.json";

    ChVector3d init_loc(3.5, 0, 0.20);
    double target_speed = 4.0;

    // Prepare output directory.
    std::string out_dir = GetChronoOutputPath() + "WHEELED_VEHICLE_SPH_COSIM";
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    MPI_Barrier(MPI_COMM_WORLD);

    // Initialize co-simulation framework (specify 4 tire nodes).
    cosim::InitializeFramework(4);

    // Peek in spec file and check terrain type
    auto terrain_type = ChVehicleCosimTerrainNodeChrono::GetTypeFromSpecfile(vehicle::GetDataFile(terrain_specfile));
    if (terrain_type != ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_SPH) {
        if (rank == 0)
            std::cout << "Incorrect terrain type" << std::endl;
        MPI_Finalize();
        return 1;
    }

    // Peek in spec file and extract tire type
    auto tire_type = ChVehicleCosimTireNode::GetTireTypeFromSpecfile(vehicle::GetDataFile(tire_specfile));
    if (tire_type == ChVehicleCosimTireNode::TireType::UNKNOWN) {
        if (rank == 0)
            std::cout << "Unsupported tire type" << std::endl;
        MPI_Finalize();
        return 1;
    }

    // Create the node (vehicle, terrain, or tire node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    // VEHICLE node
    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Vehicle node] rank = " << rank << " running on: " << procname << endl;

        auto vehicle = new ChVehicleCosimWheeledVehicleNode(vehicle::GetDataFile(vehicle_specfile),
                                                            vehicle::GetDataFile(engine_specfile),
                                                            vehicle::GetDataFile(transmission_specfile));
        vehicle->SetVerbose(verbose);
        vehicle->SetInitialLocation(init_loc);
        vehicle->SetInitialYaw(0);
        vehicle->SetStepSize(step_mbs);
        vehicle->SetOutDir(out_dir);
        if (verbose)
            cout << "[Vehicle node] output directory: " << vehicle->GetOutDirName() << endl;

        if (renderRT)
            vehicle->EnableRuntimeVisualization(render_fps, writeRT);
        if (writePP)
            vehicle->EnablePostprocessVisualization(render_fps);
        vehicle->SetCameraPosition(ChVector3d(20, 6, 2));

        vehicle->SetChassisFixed(fix_chassis);

        node = vehicle;
    }

    // TERRAIN node
    if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        auto terrain = new ChVehicleCosimTerrainNodeGranularSPH(vehicle::GetDataFile(terrain_specfile));
        terrain->SetVerbose(verbose);
        terrain->SetStepSize(step_terrain);
        terrain->SetOutDir(out_dir);
        if (verbose)
            cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

        terrain->SetSolidVisualization(true, false);

        if (renderRT)
            terrain->EnableRuntimeVisualization(render_fps, writeRT);
        if (writePP)
            terrain->EnablePostprocessVisualization(render_fps);
        terrain->SetCameraPosition(ChVector3d(4, 6, 2.5));

        node = terrain;
    }

    // TIRE nodes
    if (rank > TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;

        switch (tire_type) {
            case ChVehicleCosimTireNode::TireType::RIGID: {
                auto tire = new ChVehicleCosimTireNodeRigid(rank - 2, vehicle::GetDataFile(tire_specfile));
                tire->SetVerbose(verbose);
                tire->SetStepSize(step_rigid_tire);
                tire->SetOutDir(out_dir);

                tire->GetSystem().SetNumThreads(1);

                node = tire;
                break;
            }
            case ChVehicleCosimTireNode::TireType::FLEXIBLE: {
                auto tire = new ChVehicleCosimTireNodeFlexible(rank - 2, vehicle::GetDataFile(tire_specfile));
                tire->EnableTirePressure(true);
                tire->SetVerbose(verbose);
                tire->SetStepSize(step_fea_tire);
                tire->SetOutDir(out_dir);

                int tire_index = rank - TERRAIN_NODE_RANK - 1;
                if (render_tire[tire_index]) {
                    auto visFEA = chrono_types::make_shared<ChVisualShapeFEA>();
                    visFEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
                    visFEA->SetShellResolution(3);
                    visFEA->SetWireframe(false);
                    visFEA->SetColorscaleMinMax(0.0, 12.0);
                    visFEA->SetSmoothFaces(true);
                    tire->AddVisualShapeFEA(visFEA);
                    if (renderRT)
                        tire->EnableRuntimeVisualization(render_fps, writeRT);
                    if (writePP)
                        tire->EnablePostprocessVisualization(render_fps);
                }

                auto& sys = tire->GetSystem();
                auto solver_type = ChSolver::Type::PARDISO_MKL;
                auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
                int num_threads_chrono = std::min(8, ChOMP::GetNumProcs());
                int num_threads_collision = 1;
                int num_threads_eigen = 1;
                int num_threads_pardiso = std::min(8, ChOMP::GetNumProcs());
                SetChronoSolver(sys, solver_type, integrator_type, num_threads_pardiso);
                sys.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);
                if (auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
                    hht->SetAlpha(-0.2);
                    hht->SetMaxIters(5);
                    hht->SetAbsTolerances(1e-2);
                    hht->SetStepControl(false);
                    hht->SetMinStepSize(1e-4);
                    ////hht->SetVerbose(true);
                }

                node = tire;
                break;
            }
            default:
                break;
        }
    }

    // Initialize systems
    // (perform initial inter-node data exchange)
    node->Initialize();

    // Defer creation and initialization of the driver system until after the vehicle is initialized on the MBS node.
    if (rank == MBS_NODE_RANK) {
        auto vehicle = static_cast<ChVehicleCosimWheeledVehicleNode*>(node);
        auto path = vehicle->GetPath();
        if (!path) {
            cout << "Error: no vehicle path provided." << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
        ////double x_max = path->GetPoint(path->GetNumPoints() - 2).x() - 3.0;
        auto driver = chrono_types::make_shared<ChPathFollowerDriver>(*vehicle->GetVehicle(), path, "path",
                                                                      target_speed, 0.5, 0.5);
        driver->GetSteeringController().SetLookAheadDistance(2.0);
        driver->GetSteeringController().SetGains(1.0, 0, 0);
        driver->GetSpeedController().SetGains(0.6, 0.05, 0);
        driver->Initialize();

        vehicle->SetDriver(driver);
    }

    // Perform co-simulation
    // (perform synchronization inter-node data exchange)
    int cosim_frame = 0;
    int output_frame = 0;
    double time = 0.0;

    double t_start = MPI_Wtime();
    double t_total;
    while (time < sim_time) {
        if (verbose && rank == 0)
            cout << cosim_frame << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

        node->Synchronize(cosim_frame, time);
        node->Advance(step_cosim);
        if (verbose)
            cout << "Node" << rank << " sim time = " << node->GetStepExecutionTime() << "  ["
                 << node->GetTotalExecutionTime() << "]" << endl;

        if (output && time > output_frame / output_fps) {
            node->OutputData(output_frame);
            node->OutputVisualizationData(output_frame);
            output_frame++;
        }

        cosim_frame++;
        time += step_cosim;

        if (!verbose && rank == 0) {
            t_total = MPI_Wtime() - t_start;
            cout << "\rRTF: " << t_total / time << flush;
        }
    }
    t_total = MPI_Wtime() - t_start;

    cout << "Node" << rank << " sim time: " << node->GetTotalExecutionTime() << " total time: " << t_total << endl;

    // Cleanup
    delete node;
    MPI_Finalize();
    return 0;
}
