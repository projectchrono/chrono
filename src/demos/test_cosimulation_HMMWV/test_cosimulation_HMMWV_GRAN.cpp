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
// Authors: Nic Olsen, Radu Serban
// =============================================================================
//
// MAIN DRIVER
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <omp.h>
#include <algorithm>
#include <iostream>
#include <string>

#include "chrono/core/ChFileutils.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "TerrainNodeGran.h"
#include "TireNode.h"
#include "VehicleNode.h"

using std::cin;
using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// Maximum "radius" of a proxy triangle
double triangle_radius = 0.026;

// Enable detailed console print
bool verbose = false;

// Output initial body information
bool initial_output = false;

// Output during settling phase
bool settling_output = true;

// Output frequency (frames per second)
double output_fps = 60;

// Checkpointing frequency (frames per second)
double checkpoint_fps = 100;

// Console reporting frequency (frames per second)
double report_fps = 1000;

// Output directory
std::string out_dir = "../HMMWV_COSIM_GRAN";

// =============================================================================

// ID values to identify command line arguments
enum {
    OPT_HELP,
    OPT_THREADS_TIRE,
    OPT_TIRE_MODEL,
    OPT_MANEUVER,
    OPT_SETTLING_TIME,
    OPT_SIM_TIME,
    OPT_STEP_SIZE,
    OPT_NO_OUTPUT,
    OPT_NO_RENDERING,
    OPT_NUM_LAYERS,
    OPT_PART_RADIUS,
    OPT_COHESION,
    OPT_INIT_VEL,
    OPT_INIT_OMEGA,
    OPT_SUFFIX
};

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_THREADS_TIRE, "--num-threads-tire", SO_REQ_CMB},
                                    {OPT_TIRE_MODEL, "--tire-model", SO_REQ_CMB},
                                    {OPT_MANEUVER, "--maneuver", SO_REQ_CMB},
                                    {OPT_SETTLING_TIME, "--settling-time", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "-t", SO_REQ_CMB},
                                    {OPT_SIM_TIME, "--simulation-time", SO_REQ_CMB},
                                    {OPT_STEP_SIZE, "-s", SO_REQ_CMB},
                                    {OPT_STEP_SIZE, "--step-size", SO_REQ_CMB},
                                    {OPT_NO_OUTPUT, "--no-output", SO_NONE},
                                    {OPT_NO_RENDERING, "--no-rendering", SO_NONE},
                                    {OPT_NUM_LAYERS, "-n", SO_REQ_CMB},
                                    {OPT_NUM_LAYERS, "--num-layers", SO_REQ_CMB},
                                    {OPT_PART_RADIUS, "-r", SO_REQ_CMB},
                                    {OPT_PART_RADIUS, "--particle-radius", SO_REQ_CMB},
                                    {OPT_COHESION, "--cohesion-terrain", SO_REQ_CMB},
                                    {OPT_INIT_VEL, "--initial-fwd-velocity", SO_REQ_CMB},
                                    {OPT_INIT_OMEGA, "--initial-wheel-omega", SO_REQ_CMB},
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
                     double& settling_time,
                     double& sim_time,
                     double& step_size,
                     int& maneuver,
                     int& tire,
                     int& num_layers,
                     double& particle_radius,
                     double& cohesion,
                     double& init_fwd_vel,
                     double& init_wheel_omega,
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
    int nthreads_tire = 2;
    double settling_time = 1;
    double sim_time = 5;
    double step_size = 4e-5;
    int imaneuver = 1;
    int itire = 0;
    int num_layers = 15;
    double particle_radius = 0.012;
    double coh_pressure = 100e3;
    double init_fwd_vel = 0;
    double init_wheel_omega = 0;
    bool use_checkpoint = false;
    bool output = true;
    bool render = true;
    std::string suffix = "";
    if (!GetProblemSpecs(argc, argv, rank, nthreads_tire, settling_time, sim_time, step_size, imaneuver, itire,
                         num_layers, particle_radius, coh_pressure, init_fwd_vel, init_wheel_omega, output, render,
                         suffix)) {
        MPI_Finalize();
        return 1;
    }

    // Set tire specification file (relative to the Chrono::Vehicle data directory)
    std::string tire_filename;
    switch (itire) {
        case 0:
            tire_filename = "hmmwv/tire/HMMWV_ANCFTire.json";
            break;
        case 1:
            tire_filename = "hmmwv/tire/HMMWV_ANCFTire_Lumped.json";
            break;
        case 2:
            tire_filename = "hmmwv/tire/HMMWV_RigidMeshTire.json";
            break;
        case 3:
            tire_filename = "hmmwv/tire/HMMWV_RigidMeshTire_Coarse.json";
            break;
    }

    // Based on specified maneuver, set driver type and terrain dimensions.
    VehicleNode::DriverType driver_type;
    double container_length;
    double container_width;
    double container_height;
    std::shared_ptr<ChBezierCurve> path;

    switch (imaneuver) {
        case 0:
            driver_type = VehicleNode::DEFAULT_DRIVER;
            container_length = 5.5;
            container_width = 3;
            container_height = 1;
            break;
        case 1:
            driver_type = VehicleNode::DATA_DRIVER;
            container_length = 10;
            container_width = 3;
            container_height = 1;
            break;
        case 2:
            driver_type = VehicleNode::PATH_DRIVER;
            container_length = 110;
            container_width = 6;
            container_height = 1;
            path = DoubleLaneChangePath(ChVector<>(-container_length / 2, -1.5, 0), 20, 3, 20, 40, true);
            break;
    }

    // Prepare output directory.
    if (rank == 0) {
        if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
            cout << "Error creating directory " << out_dir << endl;
            return 1;
        }
    }

    // Create the systems and run the settling phase for terrain.
    VehicleNode* my_vehicle = nullptr;
    TerrainNodeGran* my_terrain = nullptr;
    TireNode* my_tire = nullptr;

    switch (rank) {
        case VEHICLE_NODE_RANK: {
            my_vehicle = new VehicleNode();
            my_vehicle->SetVerbose(verbose);
            my_vehicle->SetStepSize(step_size);
            my_vehicle->SetOutDir(out_dir, suffix);
            my_vehicle->SetChassisFixed(false);
            my_vehicle->SetInitFwdVel(init_fwd_vel);
            my_vehicle->SetInitWheelAngVel(init_wheel_omega);

            cout << my_vehicle->GetPrefix() << " rank = " << rank << " running on: " << procname << endl;
            cout << my_vehicle->GetPrefix() << " output directory: " << my_vehicle->GetOutDirName() << endl;

            switch (driver_type) {
                case VehicleNode::DEFAULT_DRIVER: {
                    cout << my_vehicle->GetPrefix() << " Drop test." << endl;
                    break;
                }
                case VehicleNode::DATA_DRIVER: {
                    std::vector<ChDataDriver::Entry> data;
                    data.push_back({0.0, 0, 0.0, 0});
                    data.push_back({0.5, 0, 0.0, 0});
                    data.push_back({0.7, 0, 0.8, 0});
                    data.push_back({1.0, 0, 0.8, 0});
                    my_vehicle->SetDataDriver(data);
                    cout << my_vehicle->GetPrefix() << " Acceleration test." << endl;
                    break;
                }
                case VehicleNode::PATH_DRIVER: {
                    double target_speed = 10.0;
                    my_vehicle->SetPathDriver(path, target_speed);
                    cout << my_vehicle->GetPrefix() << " Path following.  V = " << target_speed << endl;
                    break;
                }
            }

            break;
        }
        case TIRE_NODE_RANK(0):
        case TIRE_NODE_RANK(1):
        case TIRE_NODE_RANK(2):
        case TIRE_NODE_RANK(3): {
            int wheel_id = rank - 2;
            my_tire = new TireNode(vehicle::GetDataFile(tire_filename), WheelID(wheel_id), nthreads_tire);
            my_tire->SetVerbose(verbose);
            my_tire->SetStepSize(step_size);
            my_tire->SetOutDir(out_dir, suffix);
            cout << my_tire->GetPrefix() << " rank = " << rank << " running on: " << procname << endl;
            cout << my_tire->GetPrefix() << " output directory: " << my_tire->GetOutDirName() << endl;

            my_tire->SetProxyProperties(45, ChVector<>(0.113, 0.113, 0.113), false);
            my_tire->EnableTirePressure(true);

            my_tire->SetVerboseSolver(false);
            my_tire->SetVerboseForces(false);
            my_tire->SetVerboseStates(false);

            break;
        }
        default: {
            my_terrain = new TerrainNodeGran(4);
            my_terrain->SetVerbose(verbose);
            my_terrain->SetStepSize(step_size);
            my_terrain->SetOutDir(out_dir, suffix);
            if (rank == TERRAIN_NODE_RANK) {
                cout << my_terrain->GetPrefix() << " rank = " << rank << " running on: " << procname << endl;
                cout << my_terrain->GetPrefix() << " output directory: " << my_terrain->GetOutDirName() << endl;
            }

            my_terrain->SetContainerDimensions(container_length, container_width, container_height);

            float coh_force = static_cast<float>(CH_C_PI * particle_radius * particle_radius * coh_pressure);

            auto material = std::make_shared<ChMaterialSurfaceSMC>();
            material->SetFriction(0.9f);
            material->SetRestitution(0.0f);
            material->SetYoungModulus(8e5f);
            material->SetPoissonRatio(0.3f);
            material->SetAdhesion(static_cast<float>(coh_force));
            material->SetKn(1.0e6f);
            material->SetGn(6.0e1f);
            material->SetKt(4.0e5f);
            material->SetGt(4.0e1f);
            my_terrain->SetMaterialSurface(material);
            my_terrain->EnableInitialOutput(initial_output);
            my_terrain->EnableSettlingOutput(settling_output, output_fps);

            my_terrain->SetProxyProperties(1, false);
            my_terrain->SetGranularMaterial(particle_radius, 2500, num_layers);
            my_terrain->SetSettlingTime(settling_time);
            my_terrain->Settle();
            my_terrain->SetPath(path);

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
        case TIRE_NODE_RANK(0):
        case TIRE_NODE_RANK(1):
        case TIRE_NODE_RANK(2):
        case TIRE_NODE_RANK(3):
            my_tire->Initialize();
            break;
        default:
            my_terrain->Initialize();
            break;
    }

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));
    int checkpoint_steps = (int)std::ceil(1 / (checkpoint_fps * step_size));
    int report_steps = (int)std::ceil(1 / (report_fps * step_size));

    // Perform co-simulation.
    // At synchronization, there is bi-directional data exchange:
    //     tire => terrain (vertex state information)
    //     terrain => tire (vertex force information)
    //     tire => vehicle (wheel force)
    //     vehicle => tire (wheel state)
    MPI_Barrier(MPI_COMM_WORLD);

    ChTimer<double> timer;
    double cum_time = 0;

    int output_frame = 0;
    int checkpoint_frame = 0;

    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;
        timer.reset();
        timer.start();

        switch (rank) {
            case VEHICLE_NODE_RANK: {
                my_vehicle->Synchronize(is, time);
                my_vehicle->Advance(step_size);

                if (is % report_steps == 0) {
                    cout << is << " ---------------------------- " << endl;
                    cout << my_vehicle->GetPrefix() << " sim time = " << my_vehicle->GetSimTime() << "  ["
                         << my_vehicle->GetTotalSimTime() << "]" << endl;
                }

                if (output && is % output_steps == 0) {
                    my_vehicle->OutputData(output_frame);
                    output_frame++;
                }

                break;
            }
            case TIRE_NODE_RANK(0):
            case TIRE_NODE_RANK(1):
            case TIRE_NODE_RANK(2):
            case TIRE_NODE_RANK(3): {
                my_tire->Synchronize(is, time);
                my_tire->Advance(step_size);

                if (is % report_steps == 0) {
                    cout << my_tire->GetPrefix() << " sim time = " << my_tire->GetSimTime() << "  ["
                         << my_tire->GetTotalSimTime() << "]" << endl;
                }

                if (output && is % output_steps == 0) {
                    my_tire->OutputData(output_frame);
                    output_frame++;
                }
                break;
            }
            default: {
                my_terrain->Synchronize(is, time);

                my_terrain->Advance(step_size);

                if (rank == TERRAIN_NODE_RANK && is % report_steps == 0) {
                    cout << my_terrain->GetPrefix() << " sim time = " << my_terrain->GetSimTime() << "  ["
                         << my_terrain->GetTotalSimTime() << "]" << endl;
                }

                if (output && is % output_steps == 0) {
                    my_terrain->OutputData(output_frame);
                    output_frame++;
                }

                break;
            }
        }

        timer.stop();
        cum_time += timer();
        double max_cum_time;
        MPI_Reduce(&cum_time, &max_cum_time, 1, MPI_DOUBLE, MPI_MAX, VEHICLE_NODE_RANK, MPI_COMM_WORLD);
        if (rank == VEHICLE_NODE_RANK && is % report_steps == 0) {
            cout << "sim time = " << max_cum_time << endl;
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
    cout << "Usage:  mpiexec -np N test_VEH_HMMWV_Cosimulation [OPTIONS]" << endl;
    cout << "    (Note that N must be at least 6)" << endl;
    cout << endl;
    cout << " --num-threads-tire=NUM_THREADS_TIRE" << endl;
    cout << "        Specify number of OpenMP threads for the rig node [default: 2]" << endl;
    cout << " --settling-time=STL_TIME" << endl;
    cout << "        Specify duration for granular terrain settling [default: 1]" << endl;
    cout << " -t=SIM_TIME" << endl;
    cout << " --simulation-time=SIM_TIME" << endl;
    cout << "        Specify simulation length in seconds [default: 5]" << endl;
    cout << " -s=STEP_SIZE" << endl;
    cout << " --step-size=STEP_SIZE" << endl;
    cout << "        Specify integration step size in seconds [default: 4e-5]" << endl;
    cout << " --tire-model=TYPE" << endl;
    cout << "        Specify tire type [default: ANCF]" << endl;
    cout << "             0: ANCF" << endl;
    cout << "             1: ANCF_lumped" << endl;
    cout << "             2: RIGID (fine mesh)" << endl;
    cout << "             3: RIGID (Coarse mesh)" << endl;
    cout << " --maneuver=TEST" << endl;
    cout << "        Specify the particular test to be executed [default: acceleration test]" << endl;
    cout << "             0: drop test" << endl;
    cout << "             1: acceleration test" << endl;
    cout << "             2: double-lane change test" << endl;
    cout << " -n=NUM_LAYERS" << endl;
    cout << " --num-layers=NUM_LAYERS" << endl;
    cout << "        Specify the initial number of particle layers [default: 15]" << endl;
    cout << " -r=RADIUS" << endl;
    cout << " --particle-radius=RADIUS" << endl;
    cout << "        Specify particle radius for granular terrain in m [default: 0.006]" << endl;
    cout << " --cohesion-terrain=COHESION" << endl;
    cout << "        Specify the value of the terrain cohesion in Pa [default: 80e3]" << endl;
    cout << " --initial-fwd-velocity=VELOCITY" << endl;
    cout << "        Specify initial chassis forward velocity in m/s [default: 0]" << endl;
    cout << " --initial-wheel-omega" << endl;
    cout << "        Specify initial wheel angular velocities in rad/s [default: 0]" << endl;
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
                     double& settling_time,
                     double& sim_time,
                     double& step_size,
                     int& maneuver,
                     int& tire,
                     int& num_layers,
                     double& particle_radius,
                     double& cohesion,
                     double& init_fwd_vel,
                     double& init_wheel_omega,
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
            case OPT_SETTLING_TIME:
                settling_time = std::stod(args.OptionArg());
                break;
            case OPT_SIM_TIME:
                sim_time = std::stod(args.OptionArg());
                break;
            case OPT_STEP_SIZE:
                step_size = std::stod(args.OptionArg());
                break;
            case OPT_TIRE_MODEL:
                tire = std::stoi(args.OptionArg());
                break;
            case OPT_MANEUVER:
                maneuver = std::stoi(args.OptionArg());
                break;
            case OPT_NUM_LAYERS:
                num_layers = std::stoi(args.OptionArg());
                break;
            case OPT_PART_RADIUS:
                particle_radius = std::stod(args.OptionArg());
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
            case OPT_INIT_VEL:
                init_fwd_vel = std::stod(args.OptionArg());
                break;
            case OPT_INIT_OMEGA:
                init_wheel_omega = std::stod(args.OptionArg());
                break;
            case OPT_SUFFIX:
                suffix = args.OptionArg();
                break;
        }
    }

    return true;
}
