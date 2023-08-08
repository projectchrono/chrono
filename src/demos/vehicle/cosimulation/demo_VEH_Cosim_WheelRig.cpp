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
// Demo for single-wheel rig cosimulation framework
//
// Global reference frame: Z up, X towards the front, and Y pointing to the left
//
// =============================================================================

#include <iostream>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeFlexible.h"
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
                     std::string& tire_specfile,
                     int& nthreads_tire,
                     int& nthreads_terrain,
                     double& step_size,
                     bool& fixed_settling_time,
                     double& KE_threshold,
                     double& settling_time,
                     double& sim_time,
                     ChVehicleCosimDBPRigImposedSlip::ActuationType& act_type,
                     double& base_vel,
                     double& slip,
                     double& total_mass,
                     double& toe_angle,
                     double& dbp_filter_window,
                     bool& use_checkpoint,
                     double& output_fps,
                     double& vis_output_fps,
                     double& render_fps,
                     bool& sim_output,
                     bool& settling_output,
                     bool& vis_output,
                     bool& render,
                     bool& verbose,
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

    if (num_procs != 3) {
        if (rank == 0)
            std::cout << "\n\nSingle wheel cosimulation code must be run on exactly 3 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Parse command line arguments
    std::string terrain_specfile;
    std::string tire_specfile;
    auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
    int nthreads_tire = 1;
    int nthreads_terrain = 1;
    double step_size = 1e-4;
    bool fixed_settling_time = true;
    double KE_threshold = std::numeric_limits<double>::infinity();
    double settling_time = 0.4;
    double sim_time = 10;
    double base_vel = 1.0;
    double slip = 0;
    bool use_checkpoint = false;
    double output_fps = 100;
    double vis_output_fps = 100;
    double render_fps = 0;
    bool sim_output = true;
    bool settling_output = true;
    bool vis_output = true;
    bool renderRT = true;
    bool renderPP = false;
    bool writeRT = true;
    double total_mass = 500;
    double toe_angle = 0;
    double dbp_filter_window = 0.1;
    std::string suffix = "";
    bool verbose = true;
    if (!GetProblemSpecs(argc, argv, rank, terrain_specfile, tire_specfile, nthreads_tire, nthreads_terrain, step_size,
                         fixed_settling_time, KE_threshold, settling_time, sim_time, act_type, base_vel, slip,
                         total_mass, toe_angle, dbp_filter_window, use_checkpoint, output_fps, vis_output_fps,
                         render_fps, sim_output, settling_output, vis_output, renderRT, verbose, suffix)) {
        MPI_Finalize();
        return 1;
    }

    // Peek in spec file and extract tire type
    auto tire_type = ChVehicleCosimTireNode::GetTireTypeFromSpecfile(tire_specfile);
    if (tire_type == ChVehicleCosimTireNode::TireType::UNKNOWN) {
        if (rank == 0)
            std::cout << "Unsupported tire type" << std::endl;
        MPI_Finalize();
        return 1;
    }

    // Peek in spec file and extract terrain type
    auto terrain_type = ChVehicleCosimTerrainNodeChrono::GetTypeFromSpecfile(terrain_specfile);
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::UNKNOWN) {
        MPI_Finalize();
        return 1;
    }

    // Terrain dimensions and spindle initial location
    double terrain_length = 6;
    double terrain_width = 2;
    ChVector<> init_loc(-terrain_length / 2 + 1, 0, 0.425);

// Check if required modules are enabled
#ifndef CHRONO_MULTICORE
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_OMP) {
        if (rank == 0)
            cout << "Chrono::Multicore is required for GRANULAR_OMP terrain type!" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }
#endif
#ifndef CHRONO_GPU
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_GPU) {
        if (rank == 0)
            cout << "Chrono::Gpu is required for GRANULAR_GPU terrain type!" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }
#endif
#ifndef CHRONO_FSI
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_SPH) {
        if (rank == 0)
            cout << "Chrono::FSI is required for GRANULAR_SPH terrain type!" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }
#endif

    // Prepare output directory.
    std::string out_dir_top = GetChronoOutputPath() + "RIG_COSIM";
    std::string out_dir = out_dir_top + "/" +                                             //
                          ChVehicleCosimTireNode::GetTireTypeAsString(tire_type) + "_" +  //
                          ChVehicleCosimTerrainNodeChrono::GetTypeAsString(terrain_type);
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir_top))) {
            cout << "Error creating directory " << out_dir_top << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
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
    int vis_output_steps = (int)std::ceil(1 / (vis_output_fps * step_size));

    // Initialize co-simulation framework (specify 1 tire node).
    cosim::InitializeFramework(1);

    // Create the node (a rig, tire, or terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Rig node    ] rank = " << rank << " running on: " << procname << endl;

        auto dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
        dbp_rig->SetDBPFilterWindow(dbp_filter_window);

        auto mbs = new ChVehicleCosimRigNode();
        mbs->SetVerbose(verbose);
        mbs->SetInitialLocation(init_loc);
        mbs->SetStepSize(step_size);
        mbs->SetNumThreads(1);
        mbs->SetTotalMass(total_mass);
        mbs->SetOutDir(out_dir, suffix);
        mbs->AttachDrawbarPullRig(dbp_rig);
        if (verbose)
            cout << "[Rig node    ] output directory: " << mbs->GetOutDirName() << endl;

        node = mbs;

    }  // if MBS_NODE_RANK

    if (rank == TIRE_NODE_RANK(0)) {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;
        switch (tire_type) {
            case ChVehicleCosimTireNode::TireType::RIGID: {
                auto tire = new ChVehicleCosimTireNodeRigid(0, tire_specfile);
                tire->SetVerbose(verbose);
                tire->SetStepSize(step_size);
                tire->SetNumThreads(1);
                tire->SetOutDir(out_dir, suffix);

                node = tire;
                break;
            }
            case ChVehicleCosimTireNode::TireType::FLEXIBLE: {
                auto tire = new ChVehicleCosimTireNodeFlexible(0, tire_specfile);
                tire->EnableTirePressure(true);
                tire->SetVerbose(verbose);
                tire->SetStepSize(step_size);
                tire->SetNumThreads(nthreads_tire);
                tire->SetOutDir(out_dir, suffix);
                if (renderRT)
                    tire->EnableRuntimeVisualization(render_fps, writeRT);
                if (renderPP)
                    tire->EnablePostprocessVisualization(render_fps);
                tire->SetCameraPosition(ChVector<>(0, 2 * terrain_width, 1.0));

                node = tire;
                break;
            }
            default:
                break;
        }

    }  // if TIRE_NODE_RANK

    if (rank == TERRAIN_NODE_RANK) {
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
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(0, 2 * terrain_width, 1.0));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                node = terrain;
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::SCM: {
                auto terrain = new ChVehicleCosimTerrainNodeSCM(terrain_specfile);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetNumThreads(nthreads_terrain);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(0, 2 * terrain_width, 1.0));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                if (use_checkpoint) {
                    terrain->SetInputFromCheckpoint("checkpoint_end.dat");
                }

                node = terrain;
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_OMP: {
#ifdef CHRONO_MULTICORE
                auto method = ChContactMethod::SMC;
                auto terrain = new ChVehicleCosimTerrainNodeGranularOMP(method, terrain_specfile);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetNumThreads(nthreads_terrain);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(0, 2 * terrain_width, 1.0));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                terrain->SetWallThickness(0.1);

                if (use_checkpoint) {
                    terrain->SetInputFromCheckpoint("checkpoint_settled.dat");
                } else {
                    if (fixed_settling_time)
                        terrain->SetSettlingTime(settling_time);
                    else
                        terrain->SetSettlingKineticEneryThreshold(KE_threshold);
                    terrain->EnableSettlingOutput(settling_output, output_fps);
                    terrain->Settle();
                    terrain->WriteCheckpoint("checkpoint_settled.dat");
                }

                node = terrain;
#endif
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_GPU: {
#ifdef CHRONO_GPU
                auto terrain = new ChVehicleCosimTerrainNodeGranularGPU(terrain_specfile);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(0, 2 * terrain_width, 1.0));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                if (use_checkpoint) {
                    terrain->SetInputFromCheckpoint("checkpoint_settled.dat");
                } else {
                    if (fixed_settling_time)
                        terrain->SetSettlingTime(settling_time);
                    else
                        terrain->SetSettlingKineticEneryThreshold(KE_threshold);
                    terrain->EnableSettlingOutput(settling_output, output_fps);
                    terrain->Settle();
                    terrain->WriteCheckpoint("checkpoint_settled.dat");
                }

                node = terrain;
#endif
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_SPH: {
#ifdef CHRONO_FSI
                auto terrain = new ChVehicleCosimTerrainNodeGranularSPH(terrain_specfile);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps, writeRT);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(0, 2 * terrain_width, 1.0));
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                node = terrain;
#endif
                break;
            }

            default:
                break;

        }  // switch terrain_type

    }  // if TERRAIN_NODE_RANK

    // Initialize systems
    // (perform initial inter-node data exchange)
    node->Initialize();

    // Perform co-simulation
    // (perform synchronization inter-node data exchange)
    int output_frame = 0;
    int vis_output_frame = 0;

    double t_start = MPI_Wtime();
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

        if (sim_output && is % output_steps == 0) {
            node->OutputData(output_frame);
            output_frame++;
        }

        if (vis_output && is % vis_output_steps == 0) {
            node->OutputVisualizationData(vis_output_frame);
            vis_output_frame++;
        }
    }
    double t_total = MPI_Wtime() - t_start;

    cout << "Node" << rank << " sim time: " << node->GetTotalExecutionTime() << " total time: " << t_total << endl;

    node->WriteCheckpoint("checkpoint_end.dat");

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     std::string& terrain_specfile,
                     std::string& tire_specfile,
                     int& nthreads_tire,
                     int& nthreads_terrain,
                     double& step_size,
                     bool& fixed_settling_time,
                     double& KE_threshold,
                     double& settling_time,
                     double& sim_time,
                     ChVehicleCosimDBPRigImposedSlip::ActuationType& act_type,
                     double& base_vel,
                     double& slip,
                     double& total_mass,
                     double& toe_angle,
                     double& dbp_filter_window,
                     bool& use_checkpoint,
                     double& output_fps,
                     double& vis_output_fps,
                     double& render_fps,
                     bool& sim_output,
                     bool& settling_output,
                     bool& vis_output,
                     bool& render,
                     bool& verbose,
                     std::string& suffix) {
    ChCLI cli(argv[0], "Single-wheel test rig simulation (run on 3 MPI ranks)");

    cli.AddOption<std::string>("Experiment", "terrain_specfile", "Terrain specification file [JSON format]");
    cli.AddOption<std::string>("Experiment", "tire_specfile", "Tire specification file [JSON format]");

    cli.AddOption<std::string>("Experiment", "actuation_type", "Actuation type (SET_LIN_VEL or SET_ANG_VEL)",
                               ChVehicleCosimDBPRigImposedSlip::GetActuationTypeAsString(act_type));
    cli.AddOption<double>("Experiment", "base_vel", "Base velocity [m/s or rad/s]", std::to_string(base_vel));
    cli.AddOption<double>("Experiment", "slip", "Longitudinal slip", std::to_string(slip));
    cli.AddOption<double>("Experiment", "total_mass", "Total mass [kg]", std::to_string(total_mass));
    cli.AddOption<double>("Experiment", "toe_angle", "Wheel toe angle [rad]", std::to_string(toe_angle));
    cli.AddOption<double>("Experiment", "filter_window", "Time window for running average filter",
                          std::to_string(dbp_filter_window));

    cli.AddOption<double>("Simulation", "settling_time", "Duration of settling phase [s]",
                          std::to_string(settling_time));
    cli.AddOption<double>("Simulation", "KE_threshold", "KE threshold for settling [J] (default: infinity)");
    cli.AddOption<double>("Simulation", "sim_time", "Simulation length after settling phase [s]",
                          std::to_string(sim_time));
    cli.AddOption<double>("Simulation", "step_size", "Integration step size [s]", std::to_string(step_size));

    cli.AddOption<int>("Simulation", "threads_tire", "Number of OpenMP threads for the tire node",
                       std::to_string(nthreads_tire));
    cli.AddOption<int>("Simulation", "threads_terrain", "Number of OpenMP threads for the terrain node",
                       std::to_string(nthreads_terrain));

    cli.AddOption<bool>("Simulation", "use_checkpoint", "Initialize from checkpoint file");

    cli.AddOption<bool>("Output", "quiet", "Disable verbose messages");
    cli.AddOption<bool>("Output", "no_output", "Disable generation of simulation output files");
    cli.AddOption<bool>("Output", "no_settling_output", "Disable generation of settling output files");
    cli.AddOption<bool>("Output", "no_vis_output", "Disable generation of post-processing visualization output files");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));
    cli.AddOption<double>("Output", "vis_output_fps", "Visualization output frequency [fps]",
                          std::to_string(vis_output_fps));
    cli.AddOption<std::string>("Output", "suffix", "Suffix for output directory names", suffix);

    cli.AddOption<bool>("Visualization", "no_render", "Disable run-time rendering");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));

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

    try {
        tire_specfile = cli.Get("tire_specfile").as<std::string>();
    } catch (std::domain_error&) {
        if (rank == 0) {
            cout << "\nERROR: Missing tire specification file!\n\n" << endl;
            cli.Help();
        }
        return false;
    }

    act_type =
        ChVehicleCosimDBPRigImposedSlip::GetActuationTypeFromString(cli.GetAsType<std::string>("actuation_type"));
    if (act_type == ChVehicleCosimDBPRigImposedSlip::ActuationType::UNKNOWN) {
        if (rank == 0) {
            cout << "\nERROR: Unrecognized actuation type!\n\n" << endl;
            cli.Help();
        }
        return false;
    }

    base_vel = cli.GetAsType<double>("base_vel");
    slip = cli.GetAsType<double>("slip");

    if (act_type == ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_LIN_VEL && slip > 0.95) {
        if (rank == 0) {
            cout << "\nERROR: Slip value " << slip << " too large for "
                 << ChVehicleCosimDBPRigImposedSlip::GetActuationTypeAsString(act_type) << " actuation mode!\n\n"
                 << endl;
            cli.Help();
        }
        return false;
    }

    if (cli.CheckOption("KE_threshold")) {
        KE_threshold = cli.GetAsType<double>("KE_threshold");
        fixed_settling_time = false;
    } else {
        settling_time = cli.GetAsType<double>("settling_time");
        fixed_settling_time = true;
    }

    sim_time = cli.GetAsType<double>("sim_time");
    step_size = cli.GetAsType<double>("step_size");

    total_mass = cli.GetAsType<double>("total_mass");
    toe_angle = cli.GetAsType<double>("toe_angle");

    dbp_filter_window = cli.GetAsType<double>("filter_window");

    verbose = !cli.GetAsType<bool>("quiet");
    render = !cli.GetAsType<bool>("no_render");
    sim_output = !cli.GetAsType<bool>("no_output");
    settling_output = !cli.GetAsType<bool>("no_settling_output");
    vis_output = !cli.GetAsType<bool>("no_vis_output");

    output_fps = cli.GetAsType<double>("output_fps");
    vis_output_fps = cli.GetAsType<double>("vis_output_fps");
    render_fps = cli.GetAsType<double>("render_fps");

    use_checkpoint = cli.GetAsType<bool>("use_checkpoint");

    nthreads_tire = cli.GetAsType<int>("threads_tire");
    nthreads_terrain = cli.GetAsType<int>("threads_terrain");

    suffix = cli.GetAsType<std::string>("suffix");

    return true;
}
