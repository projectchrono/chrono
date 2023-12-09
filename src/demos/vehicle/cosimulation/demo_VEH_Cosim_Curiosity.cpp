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
// Demo for Curiosity rover cosimulation on different terrain types.
// The robot is co-simulated with a terrain node and a number of tire nodes
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

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimCuriosityNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
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
using namespace chrono::curiosity;

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     std::string& terrain_specfile,
                     ChVector2<>& init_loc,
                     double& terrain_length,
                     double& terrain_width,
                     int& nthreads_tire,
                     int& nthreads_terrain,
                     double& step_size,
                     bool& fixed_settling_time,
                     double& KE_threshold,
                     double& settling_time,
                     double& sim_time,
                     double& output_fps,
                     double& vis_output_fps,
                     double& render_fps,
                     bool& verbose,
                     bool& use_DBP_rig,
                     bool& add_obstacles);

// =============================================================================

class MyDriver : public CuriosityDriver {
  public:
    MyDriver(double delay) : CuriosityDriver(), m_delay(delay) {}
    ~MyDriver() {}

    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }

    virtual void Update(double time) override {
        // Do not generate any driver inputs for a duration equal to m_delay.
        double eff_time = time - m_delay;
        if (eff_time < 0)
            return;

        double driving = 0;
        if (eff_time > 0.2)
            driving = 0.7;
        else
            driving = 3.5 * eff_time;

        double steering = 0;
        if (eff_time < 4)
            steering = 0;
        else
            steering = 0.75 * std::sin(CH_C_2PI * (eff_time - 4) / 50);

        for (int i = 0; i < 6; i++) {
            drive_speeds[i] = driving;
        }
        for (int i = 0; i < 4; i++) {
            steer_angles[i] = steering;
        }
    }

  private:
    double m_delay;
};

// =============================================================================

ChVehicleCosimTerrainNodeChrono::RigidObstacle CreateObstacle(const ChVector<>& pos) {
    ChVehicleCosimTerrainNodeChrono::RigidObstacle o;
    o.m_mesh_filename = "vehicle/hmmwv/hmmwv_tire_coarse_closed.obj";
    o.m_density = 1000;
    o.m_init_pos = pos;
    o.m_init_rot = Q_from_AngX(CH_C_PI_2);
    o.m_contact_mat = ChContactMaterialData();
    o.m_oobb_center = ChVector<>(0, 0, 0);
    o.m_oobb_dims = ChVector<>(1, 0.3, 1);
    return o;
}

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

    if (num_procs != 8) {
        if (rank == 0)
            std::cout << "\nCuriosity cosimulation code must be run on exactly 8 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Parse command line arguments
    std::string terrain_specfile;
    ChVector2<> init_loc(0, 0);
    double terrain_length = -1;
    double terrain_width = -1;
    int nthreads_tire = 1;
    int nthreads_terrain = 1;
    double step_size = 1e-4;
    bool fixed_settling_time = true;
    double KE_threshold = std::numeric_limits<double>::infinity();
    double settling_time = 0.4;
    double sim_time = 20;
    double output_fps = 100;
    double vis_output_fps = 100;
    double render_fps = 100;
    bool verbose = true;
    bool use_DBP_rig = false;
    bool add_obstacles = false;
    if (!GetProblemSpecs(argc, argv, rank, terrain_specfile, init_loc, terrain_length, terrain_width, nthreads_tire,
                         nthreads_terrain, step_size, fixed_settling_time, KE_threshold, settling_time, sim_time,
                         output_fps, vis_output_fps, render_fps, verbose, use_DBP_rig, add_obstacles)) {
        MPI_Finalize();
        return 1;
    }

    std::string suffix = "";
    bool renderRT = true;
    bool renderPP = true;

    // Peek in spec file and extract terrain type
    auto terrain_type = ChVehicleCosimTerrainNodeChrono::GetTypeFromSpecfile(terrain_specfile);
    if (terrain_type == ChVehicleCosimTerrainNodeChrono::Type::UNKNOWN) {
        MPI_Finalize();
        return 1;
    }

    // Peek in spec file and extract patch size
    auto size = ChVehicleCosimTerrainNodeChrono::GetSizeFromSpecfile(terrain_specfile);
    if (terrain_length < 0)
        terrain_length = size.x();
    if (terrain_width < 0)
        terrain_width = size.y();

    // Do not create obstacles if a DBP rig is attached
    if (use_DBP_rig) {
        add_obstacles = false;
    }

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
    std::string out_dir_top = GetChronoOutputPath() + "CURIOSITY_COSIM";
    std::string out_dir = out_dir_top + "/" + ChVehicleCosimTerrainNodeChrono::GetTypeAsString(terrain_type);
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

    // Initialize co-simulation framework (specify 6 tire nodes).
    cosim::InitializeFramework(6);

    // Create the node (rover, tire, or terrain node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Curiosity node] rank = " << rank << " running on: " << procname << endl;

        auto curiosity = new ChVehicleCosimCuriosityNode();
        ////curiosity->SetChassisFixed(true);
        if (use_DBP_rig) {
            auto act_type = ChVehicleCosimDBPRigImposedSlip::ActuationType::SET_ANG_VEL;
            double base_vel = 1;
            double slip = 0;
            auto dbp_rig = chrono_types::make_shared<ChVehicleCosimDBPRigImposedSlip>(act_type, base_vel, slip);
            curiosity->AttachDrawbarPullRig(dbp_rig);
        }
        auto driver = chrono_types::make_shared<MyDriver>(0.2);
        curiosity->SetDriver(driver);
        curiosity->SetIntegratorType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED, ChSolver::Type::BARZILAIBORWEIN);
        curiosity->SetVerbose(verbose);
        curiosity->SetInitialLocation(ChVector<>(init_loc.x(), init_loc.y(), 0.1));
        curiosity->SetInitialYaw(0);
        curiosity->SetStepSize(step_size);
        curiosity->SetNumThreads(1);
        curiosity->SetOutDir(out_dir, suffix);
        if (renderRT)
            curiosity->EnableRuntimeVisualization(render_fps);
        if (renderPP)
            curiosity->EnablePostprocessVisualization(render_fps);
        curiosity->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
        if (verbose)
            cout << "[Curiosity node] output directory: " << curiosity->GetOutDirName() << endl;

        node = curiosity;

    }  // if MBS_NODE_RANK

    if (rank >= TIRE_NODE_RANK(0) && rank <= TIRE_NODE_RANK(5)) {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;

        auto tire = new ChVehicleCosimTireNodeRigid(
            rank - 2, vehicle::GetDataFile("cosim/curiosity/Curiosity_RigidTire_cyl.json"));
        tire->SetVerbose(verbose);
        tire->SetStepSize(step_size);
        tire->SetNumThreads(1);
        tire->SetOutDir(out_dir, suffix);

        node = tire;
    }  // if TIRE_NODE_RANK

    if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        switch (terrain_type) {
            case ChVehicleCosimTerrainNodeChrono::Type::RIGID: {
                auto method = ChContactMethod::SMC;
                auto terrain = new ChVehicleCosimTerrainNodeRigid(terrain_specfile, method);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetProxyFixed(false);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (add_obstacles) {
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(1, 1, 0.5)));
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(-1, -1, 0.5)));
                }
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
                    terrain->EnableRuntimeVisualization(render_fps);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (add_obstacles) {
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(1, 1, 0.5)));
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(-1, -1, 0.5)));
                }
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                node = terrain;
                break;
            }

            case ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_OMP: {
#ifdef CHRONO_MULTICORE
                auto method = ChContactMethod::SMC;
                auto terrain = new ChVehicleCosimTerrainNodeGranularOMP(method, terrain_specfile);
                terrain->SetDimensions(terrain_length, terrain_width);
                terrain->SetWallThickness(0.1);
                terrain->SetVerbose(verbose);
                terrain->SetStepSize(step_size);
                terrain->SetNumThreads(nthreads_terrain);
                terrain->SetOutDir(out_dir, suffix);
                if (renderRT)
                    terrain->EnableRuntimeVisualization(render_fps);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (add_obstacles) {
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(1, 1, 0.5)));
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(-1, -1, 0.5)));
                }
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                if (fixed_settling_time)
                    terrain->SetSettlingTime(settling_time);
                else
                    terrain->SetSettlingKineticEneryThreshold(KE_threshold);

                terrain->Settle();
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
                    terrain->EnableRuntimeVisualization(render_fps);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (add_obstacles) {
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(1, 1, 0.5)));
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(-1, -1, 0.5)));
                }
                if (verbose)
                    cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

                if (fixed_settling_time)
                    terrain->SetSettlingTime(settling_time);
                else
                    terrain->SetSettlingKineticEneryThreshold(KE_threshold);

                terrain->Settle();
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
                    terrain->EnableRuntimeVisualization(render_fps);
                if (renderPP)
                    terrain->EnablePostprocessVisualization(render_fps);
                terrain->SetCameraPosition(ChVector<>(terrain_length / 2, 0, 2));
                if (add_obstacles) {
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(1, 1, 0.5)));
                    terrain->AddRigidObstacle(CreateObstacle(ChVector<>(-1, -1, 0.5)));
                }
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
        if (is % vis_output_steps == 0) {
            node->OutputVisualizationData(vis_output_frame);
            vis_output_frame++;
        }
    }

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
                     ChVector2<>& init_loc,
                     double& terrain_length,
                     double& terrain_width,
                     int& nthreads_tire,
                     int& nthreads_terrain,
                     double& step_size,
                     bool& fixed_settling_time,
                     double& KE_threshold,
                     double& settling_time,
                     double& sim_time,
                     double& output_fps,
                     double& vis_output_fps,
                     double& render_fps,
                     bool& verbose,
                     bool& use_DBP_rig,
                     bool& add_obstacles) {
    ChCLI cli(argv[0], "Curiosity co-simulation (run on 8 MPI ranks)");

    cli.AddOption<std::string>("Experiment", "terrain_specfile", "Terrain specification file [JSON format]");
    cli.AddOption<double>("Experiment", "terrain_length", "If positive, overwrite terrain length read from JSON file",
                          std::to_string(terrain_length));
    cli.AddOption<double>("Experiment", "terrain_width", "If positive, overwrite terrain width read from JSON file",
                          std::to_string(terrain_width));
    cli.AddOption<std::vector<double>>("Experiment", "init_loc", "Initial rover position (x,y)", "0,0");
    cli.AddOption<bool>("Experiment", "attach_rig", "Attache DBP rig");
    cli.AddOption<bool>("Experiment", "add_obstacles", "Add rigid obstacles");

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

    cli.AddOption<bool>("Output", "quiet", "Disable verbose messages");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));
    cli.AddOption<double>("Output", "vis_output_fps", "Visualization output frequency [fps]",
                          std::to_string(vis_output_fps));

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

    auto loc = cli.GetAsType<std::vector<double>>("init_loc");
    if (loc.size() != 2) {
        if (rank == 0) {
            cout << "\nERROR: Incorrect initial location!\n\n" << endl;
            cli.Help();
        }
        return false;
    }
    init_loc.x() = loc[0];
    init_loc.y() = loc[1];

    terrain_length = cli.GetAsType<double>("terrain_length");
    terrain_width = cli.GetAsType<double>("terrain_width");

    if (cli.CheckOption("KE_threshold")) {
        KE_threshold = cli.GetAsType<double>("KE_threshold");
        fixed_settling_time = false;
    } else {
        settling_time = cli.GetAsType<double>("settling_time");
        fixed_settling_time = true;
    }

    sim_time = cli.GetAsType<double>("sim_time");
    step_size = cli.GetAsType<double>("step_size");

    use_DBP_rig = cli.GetAsType<bool>("attach_rig");
    add_obstacles = cli.GetAsType<bool>("add_obstacles");

    verbose = !cli.GetAsType<bool>("quiet");

    output_fps = cli.GetAsType<double>("output_fps");
    vis_output_fps = cli.GetAsType<double>("vis_output_fps");
    render_fps = cli.GetAsType<double>("render_fps");

    nthreads_tire = cli.GetAsType<int>("threads_tire");
    nthreads_terrain = cli.GetAsType<int>("threads_terrain");

    return true;
}
