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
#include <mpi.h>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/cosim/ChVehicleCosimRigNode.h"
#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeRigid.h"
#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNodeGranularOMP.h"

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Cosimulation step size
double step_size = 1e-3;  // 4e-5;

// Output frequency (frames per second)
double output_fps = 200;

// Checkpointing frequency (frames per second)
double checkpoint_fps = 100;

// Output directory
std::string out_dir = "../TIRE_RIG_COSIM";

// Tire type
ChVehicleCosimRigNode::Type tire_type = ChVehicleCosimRigNode::Type::RIGID;

// Terrain type
ChVehicleCosimTerrainNode::Type terrain_type = ChVehicleCosimTerrainNode::Type::RIGID;

// =============================================================================

// Forward declarations
bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& nthreads_rig,
                     int& nthreads_terrain,
                     double& sim_time,
                     double& init_vel,
                     double& slip,
                     double& coh_pressure,
                     double& sys_mass,
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

    auto err = MPI_Init(&argc, &argv);
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
    double coh_pressure = 8e4;
    bool use_checkpoint = false;
    bool output = true;
    bool render = true;
    double sys_mass = 450;
    std::string suffix = "";
    if (!GetProblemSpecs(argc, argv, rank, nthreads_rig, nthreads_terrain, sim_time, init_vel, slip, coh_pressure,
                         sys_mass, use_checkpoint, output, render, suffix)) {
        MPI_Finalize();
        return 1;
    }

    // Prepare output directory.
    if (rank == 0 && !filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        MPI_Finalize();
        return 1;
    }

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));
    int checkpoint_steps = (int)std::ceil(1 / (checkpoint_fps * step_size));

    // Create the two systems and run the settling phase for terrain.
    ChVehicleCosimRigNode* my_rig = nullptr;
    ChVehicleCosimTerrainNode* my_terrain = nullptr;

    switch (rank) {
        case RIG_NODE_RANK: {
            cout << "[Rig node    ] rank = " << rank << " running on: " << procname << endl;

            switch (tire_type) {
                case ChVehicleCosimRigNode::Type::RIGID: {
                    auto rig = new ChVehicleCosimRigNodeRigidTire(init_vel, slip, nthreads_rig);
                    rig->SetTireJSONFile(vehicle::GetDataFile("hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json"));
                    my_rig = rig;
                    break;
                }
                case ChVehicleCosimRigNode::Type::FLEXIBLE: {
                    auto rig = new ChVehicleCosimRigNodeFlexibleTire(init_vel, slip, nthreads_rig);
                    rig->SetTireJSONFile(vehicle::GetDataFile("hmmwv/tire/HMMWV_ANCFTire.json"));
                    my_rig = rig;
                    break;
                }
            }

            my_rig->SetStepSize(step_size);
            my_rig->SetOutDir(out_dir, suffix);
            my_rig->SetBodyMasses(1, 1, sys_mass, 15);
            my_rig->EnableTirePressure(true);

            cout << "[Rig node    ] output directory: " << my_rig->GetOutDirName() << endl;

            break;
        }
        case TERRAIN_NODE_RANK: {
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

            switch (terrain_type) {
                case ChVehicleCosimTerrainNode::Type::RIGID: {
                    auto method = ChContactMethod::SMC;
                    auto terrain = new ChVehicleCosimTerrainNodeRigid(method, render);
                    terrain->SetStepSize(step_size);
                    terrain->SetOutDir(out_dir, suffix);
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                    terrain->SetPatchDimensions(4, 2);
                    terrain->SetProxyProperties(0.002, true);

                    switch (method) {
                        case ChContactMethod::SMC: {
                            auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
                            material->SetFriction(0.9f);
                            material->SetRestitution(0.0f);
                            material->SetYoungModulus(8e5f);
                            material->SetPoissonRatio(0.3f);
                            material->SetKn(1.0e6f);
                            material->SetGn(6.0e1f);
                            material->SetKt(4.0e5f);
                            material->SetGt(4.0e1f);
                            terrain->SetMaterialSurface(material);
                            terrain->UseMaterialProperties(true);
                            terrain->SetContactForceModel(ChSystemSMC::Hertz);
                            break;
                        }
                        case ChContactMethod::NSC: {
                            auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
                            material->SetFriction(0.9f);
                            material->SetRestitution(0.0f);
                            terrain->SetMaterialSurface(material);
                            break;
                        }
                    }

                    my_terrain = terrain;
                    break;
                }
                case ChVehicleCosimTerrainNode::Type::GRANULAR_OMP: {
                    auto method = ChContactMethod::NSC;
                    auto terrain =
                        new ChVehicleCosimTerrainNodeGranularOMP(method, use_checkpoint, render, nthreads_terrain);
                    terrain->SetStepSize(step_size);
                    terrain->SetOutDir(out_dir, suffix);
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                    ////my_terrain->SetPatchDimensions(10, 0.6);
                    terrain->SetPatchDimensions(2, 0.6);
                    terrain->SetContainerDimensions(1, 0.2);

                    terrain->SetProxyProperties(0.002, true);

                    ////double radius = 0.006;
                    double radius = 0.02;

                    double coh_force = CH_C_PI * radius * radius * coh_pressure;

                    terrain->SetGranularMaterial(radius, 2500, 8);
                    terrain->SetSettlingTime(0.2);
                    ////terrain->EnableSettlingOutput(true);

                    switch (method) {
                        case ChContactMethod::SMC: {
                            auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
                            material->SetFriction(0.9f);
                            material->SetRestitution(0.0f);
                            material->SetYoungModulus(8e5f);
                            material->SetPoissonRatio(0.3f);
                            material->SetAdhesion(static_cast<float>(coh_force));
                            material->SetKn(1.0e6f);
                            material->SetGn(6.0e1f);
                            material->SetKt(4.0e5f);
                            material->SetGt(4.0e1f);
                            terrain->SetMaterialSurface(material);
                            terrain->UseMaterialProperties(false);
                            terrain->SetContactForceModel(ChSystemSMC::PlainCoulomb);
                            break;
                        }
                        case ChContactMethod::NSC: {
                            auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
                            material->SetFriction(0.9f);
                            material->SetRestitution(0.0f);
                            material->SetCohesion(static_cast<float>(coh_force));
                            terrain->SetMaterialSurface(material);
                            break;
                        }
                    }

                    terrain->Settle();

                    my_terrain = terrain;
                    break;
                }
            }

            break;
        }
    }

    // Initialize systems.
    // Data exchange:
    //   terrain => rig (terrain height)
    //   rig => terrain (tire mesh topology and local vertex information)
    //   rig => terrain (tire contact material properties)
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
    //     rig => terrain (state information)
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

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& nthreads_rig,
                     int& nthreads_terrain,
                     double& sim_time,
                     double& init_vel,
                     double& slip,
                     double& coh_pressure,
                     double& sys_mass,
                     bool& use_checkpoint,
                     bool& output,
                     bool& render,
                     std::string& suffix) {
    // Default values
    nthreads_rig = 2;
    nthreads_terrain = 2;
    sim_time = 10;
    init_vel = 0;
    slip = 0;
    coh_pressure = 8e4;
    use_checkpoint = false;
    output = true;
    render = true;
    sys_mass = 450;
    suffix = "";

    ChCLI cli(argv[0]);

    cli.AddOption<double>("Demo", "sim_time", "Simulation length after robot release [s]", "10.0");

    cli.AddOption<double>("Demo", "init_vel", "Initial tire linear velocity [m/s]", "0.0");
    cli.AddOption<double>("Demo", "slip", "Longitudinal slip", "0.0");
    cli.AddOption<double>("Demo", "coh_pressure", "Terrain cohesion [Pa]", "8e4");
    cli.AddOption<double>("Demo", "sys_mass", "Mass of wheel carrier [kg]", "450.0");

    cli.AddOption<bool>("Demo", "no_render", "Disable OpenGL rendering");
    cli.AddOption<bool>("Demo", "no_output", "Disable generation of result output files");
    cli.AddOption<bool>("Demo", "use_checkpoint", "Initialize granular terrain from checkppoint file");

    cli.AddOption<int>("Demo", "threads_rig", "Number of OpenMP threads for the rig node", "1");
    cli.AddOption<int>("Demo", "threads_terrain", "Number of OpenMP threads for the terrain node", "2");

    cli.AddOption<std::string>("Demo", "suffix", "Suffix for output directory names", "");

    if (!cli.Parse(argc, argv)) {
        if (rank == 0)
            cli.Help();
        return false;
    }

    sim_time = cli.GetAsType<double>("sim_time");

    init_vel = cli.GetAsType<double>("init_vel");
    slip = cli.GetAsType<double>("slip");
    coh_pressure = cli.GetAsType<double>("coh_pressure");
    sys_mass = cli.GetAsType<double>("sys_mass");

    render = !cli.GetAsType<bool>("no_render");
    output = !cli.GetAsType<bool>("no_output");
    use_checkpoint = cli.GetAsType<bool>("use_checkpoint");

    nthreads_rig = cli.GetAsType<int>("threads_rig");
    nthreads_terrain = cli.GetAsType<int>("threads_terrain");

    suffix = cli.GetAsType<std::string>("suffix");

    return true;
}
