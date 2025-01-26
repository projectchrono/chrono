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
// Demo for wheeled vehicle cosimulation on SCM or rigid terrain.
// The vehicle (specified through JSON files, for the vehicle, engine, and
// transmission) is co-simulated with a terrain node and a number of tire nodes
// equal to the number of wheels.
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

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimWheeledVehicleNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeRigid.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeSCM.h"

#include "demos/SetChronoSolver.h"

using std::cout;
using std::cin;
using std::endl;
using std::flush;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Specification of a vehicle model from JSON files

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double InitHeight() const = 0;
};

class HMMWV_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual double InitHeight() const override { return 0.75; }
    virtual std::string VehicleJSON() const override { return "hmmwv/vehicle/HMMWV_Vehicle.json"; }
    virtual std::string EngineJSON() const override { return "hmmwv/powertrain/HMMWV_EngineShafts.json"; }
    virtual std::string TransmissionJSON() const override {
        return "hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json";
    }
    virtual std::string TireJSON() const override { return "hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json"; }
    ////virtual std::string TireJSON() const override { return "hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json"; }
    ////virtual std::string TireJSON() const override { return "hmmwv/tire/HMMWV_ANCF8Tire_Lumped.json"; }
};

class Polaris_Model : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Polaris"; }
    virtual double InitHeight() const override { return 0.2; }
    virtual std::string VehicleJSON() const override { return "Polaris/Polaris.json"; }
    virtual std::string EngineJSON() const override { return "Polaris/Polaris_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
    }
    virtual std::string TireJSON() const override { return "Polaris/Polaris_RigidMeshTire.json"; }
    ////virtual std::string TireJSON() const override { return "Polaris/Polaris_ANCF4Tire_Lumped.json"; }
    ////virtual std::string TireJSON() const override { return "Polaris/Polaris_ANCF8Tire_Lumped.json"; }
};

auto vehicle_model = Polaris_Model();

// =============================================================================
// Specification of a terrain model from JSON file

////std::string terrain_specfile = "cosim/terrain/rigid.json";
std::string terrain_specfile = "cosim/terrain/scm_soft.json";

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

        if (eff_time > 0.2)
            m_throttle = 0.7;
        else
            m_throttle = 3.5 * eff_time;

        if (eff_time < 2)
            m_steering = 0;
        else
            m_steering = 0.6 * std::sin(CH_2PI * (eff_time - 2) / 6);
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
    double sim_time = 8.0;

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

    // If use_DBP_rig=true, attach a drawbar pull rig to the vehicle
    bool use_DBP_rig = false;

    double terrain_length = 40;
    double terrain_width = 20;
    if (use_DBP_rig) {
        terrain_length = 20;
        terrain_width = 5;
    }

    ChVector3d init_loc(3.5, 0, vehicle_model.InitHeight());

    // Initialize co-simulation framework (specify 4 tire nodes).
    cosim::InitializeFramework(4);

    // Peek in spec file and extract terrain type
    auto terrain_type = ChVehicleCosimTerrainNodeChrono::GetTypeFromSpecfile(vehicle::GetDataFile(terrain_specfile));
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::UNKNOWN) {
        MPI_Finalize();
        return 1;
    }

    // Peek in spec file and extract tire type
    auto tire_type = ChVehicleCosimTireNode::GetTireTypeFromSpecfile(vehicle::GetDataFile(vehicle_model.TireJSON()));
    if (tire_type == ChVehicleCosimTireNode::TireType::UNKNOWN) {
        if (rank == 0)
            std::cout << "Unsupported tire type" << std::endl;
        MPI_Finalize();
        return 1;
    }

    // Prepare output directory.
    std::string out_dir = GetChronoOutputPath() + "WHEELED_VEHICLE_COSIM";
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    out_dir = out_dir + "/" + vehicle_model.ModelName() + "_"                                    //
              + (tire_type == ChVehicleCosimTireNode::TireType::RIGID ? "Rigid_" : "Flexible_")  //
              + (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::RIGID ? "Rigid" : "SCM");
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    MPI_Barrier(MPI_COMM_WORLD);

    // Create the node (vehicle, terrain, or tire node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    // VEHICLE node
    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Vehicle node] rank = " << rank << " running on: " << procname << endl;

        ChVehicleCosimWheeledVehicleNode* vehicle;
        vehicle = new ChVehicleCosimWheeledVehicleNode(vehicle::GetDataFile(vehicle_model.VehicleJSON()),
                                                       vehicle::GetDataFile(vehicle_model.EngineJSON()),
                                                       vehicle::GetDataFile(vehicle_model.TransmissionJSON()));

        if (use_DBP_rig) {
            auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
            double base_vel = 1;
            double slip = 0;
            auto dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
            vehicle->AttachDrawbarPullRig(dbp_rig);
        }

        auto driver = chrono_types::make_shared<MyDriver>(*vehicle->GetVehicle(), 0.5);
        vehicle->SetDriver(driver);
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
        vehicle->SetCameraPosition(ChVector3d(terrain_length / 2, 0, 2));

        vehicle->SetChassisFixed(fix_chassis);

        node = vehicle;
    }

    // TERRAIN node
    if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        switch (terrain_type) {
            default:
                cout << "TERRAIN TYPE NOT SUPPORTED!\n" << endl;
                break;

            case ChVehicleCosimTerrainNodeChrono::Type::RIGID: {
                auto method = ChContactMethod::SMC;
                auto terrain = new ChVehicleCosimTerrainNodeRigid(vehicle::GetDataFile(terrain_specfile), method);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_terrain);
                terrain->SetOutDir(out_dir);
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps);
                if (writePP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector3d(terrain_length / 2, 0, 2));

                node = terrain;
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::SCM: {
                auto terrain = new ChVehicleCosimTerrainNodeSCM(vehicle::GetDataFile(terrain_specfile));
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_terrain);
                terrain->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));
                terrain->SetOutDir(out_dir);
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                if (writePP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector3d(terrain_length / 2, 0, 2));

                node = terrain;
                break;
            }
        }
    }

    // TIRE nodes
    if (rank > TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;

        switch (tire_type) {
            case ChVehicleCosimTireNode::TireType::RIGID: {
                auto tire = new ChVehicleCosimTireNodeRigid(rank - 2, vehicle::GetDataFile(vehicle_model.TireJSON()));
                tire->SetVerbose(verbose);
                tire->SetStepSize(step_rigid_tire);
                tire->SetOutDir(out_dir);

                tire->GetSystem().SetNumThreads(1);

                node = tire;
                break;
            }
            case ChVehicleCosimTireNode::TireType::FLEXIBLE: {
                auto tire =
                    new ChVehicleCosimTireNodeFlexible(rank - 2, vehicle::GetDataFile(vehicle_model.TireJSON()));
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

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}