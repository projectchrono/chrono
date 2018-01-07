#include <mpi.h>
#include <omp.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "chrono_distributed/collision/ChAAPlaneCB.cpp"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

using namespace chrono;
using namespace chrono::collision;

#define MASTER 0

// ID values to identify command line arguments
enum { OPT_HELP, OPT_THREADS, OPT_X, OPT_Y, OPT_Z, OPT_TIME, OPT_MONITOR, OPT_OUTPUT_DIR, OPT_VERBOSE };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_HELP, "--help", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_THREADS, "-n", SO_REQ_CMB},
                                    {OPT_X, "-x", SO_REQ_CMB},
                                    {OPT_Y, "-y", SO_REQ_CMB},
                                    {OPT_Z, "-z", SO_REQ_CMB},
                                    {OPT_TIME, "-t", SO_REQ_CMB},
                                    {OPT_MONITOR, "-m", SO_NONE},
                                    {OPT_OUTPUT_DIR, "-o", SO_REQ_CMB},
                                    {OPT_VERBOSE, "-v", SO_NONE},
                                    SO_END_OF_OPTIONS};

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& num_threads,
                     double& time_end,
                     double& h_x,
                     double& h_y,
                     double& h_z,
                     bool& monitor,
                     bool& verbose,
                     bool& output_data,
                     std::string& outdir);
void ShowUsage();

// Granular Properties
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;
double gran_radius = 0.00125;  // 1.25mm radius
double rho = 4000;
double spacing = 2.001 * gran_radius;  // Distance between adjacent centers of particles

// Dimensions
double lowest_layer = 2 * spacing;  // Lowest possible CENTER of granular material TODO adapt for collision planes
int extra_container_layers = 3;     // TODO adapt for collision planes

// Oscillation
double period = 1;                   // TODO adjust
double amplitude = gran_radius * 4;  // TODO adjust
double lower_start;

// Simulation
double time_step = 1e-4;
double out_fps = 120;
unsigned int max_iteration = 100;
double tolerance = 1e-4;

void WriteCSV(std::ofstream* file, int timestep_i, ChSystemDistributed* sys) {
    std::stringstream ss_particles;

    int i = 0;
    auto bl_itr = sys->data_manager->body_list->begin();

    for (; bl_itr != sys->data_manager->body_list->end(); bl_itr++, i++) {
        if (sys->ddm->comm_status[i] != chrono::distributed::EMPTY) {
            ChVector<> pos = (*bl_itr)->GetPos();
            ChVector<> vel = (*bl_itr)->GetPos_dt();

            ss_particles << timestep_i << "," << (*bl_itr)->GetGid() << "," << pos.x() << "," << pos.y() << ","
                         << pos.z() << "," << vel.x() << "," << vel.y() << "," << vel.z() << "," << vel.Length() << ","
                         << (((*bl_itr)->GetBodyFixed()) ? 1 : 0) << std::endl;
        }
    }

    *file << ss_particles.str();
}

void Monitor(chrono::ChSystemParallel* system, int rank) {
    double TIME = system->GetChTime();
    double STEP = system->GetTimerStep();
    double BROD = system->GetTimerCollisionBroad();
    double NARR = system->GetTimerCollisionNarrow();
    double SOLVER = system->GetTimerSolver();
    double UPDT = system->GetTimerUpdate();
    double EXCH = system->data_manager->system_timer.GetTime("Exchange");
    int BODS = system->GetNbodies();
    int CNTC = system->GetNcontacts();
    double RESID = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetResidual();
    int ITER = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetTotalIterations();

    printf("%d|   %8.5f | %7.4f | E%7.4f | B%7.4f | N%7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n",  ////
           rank, TIME, STEP, EXCH, BROD, NARR, SOLVER, UPDT, BODS, CNTC, ITER, RESID);
}

void AddContainer(ChSystemDistributed* sys,
                  double h_x,
                  double h_y,
                  double height,
                  ChAAPlaneCB** bottom_wall,
                  ChAAPlaneCB** low_x_wall,
                  ChAAPlaneCB** high_x_wall,
                  ChAAPlaneCB** low_y_wall,
                  ChAAPlaneCB** high_y_wall) {
    // TODO Any of this body stuff needed for custom collision?
    int binId = -200;

    auto mat = std::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    auto container = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
    container->SetMaterialSurface(mat);
    container->SetMass(1);
    container->SetPos(ChVector<>(0));
    container->SetCollide(true);
    container->SetBodyFixed(true);
    container->GetCollisionModel()->ClearModel();
    container->GetCollisionModel()->BuildModel();

    // TODO little extra space on sides
    *bottom_wall =
        new ChAAPlaneCB(sys, container.get(), 2, 0, ChVector<>(0, 0, 1), -2 * h_x, 2 * h_x, -2 * h_y, 2 * h_y);
    *low_x_wall =
        new ChAAPlaneCB(sys, container.get(), 0, -h_x - 0.002, ChVector<>(1, 0, 0), -2 * h_y, 2 * h_y, -height, height);
    *high_x_wall =
        new ChAAPlaneCB(sys, container.get(), 0, h_x + 0.002, ChVector<>(-1, 0, 0), -2 * h_y, 2 * h_y, -height, height);
    *low_y_wall =
        new ChAAPlaneCB(sys, container.get(), 1, -h_y - 0.002, ChVector<>(0, 1, 0), -2 * h_x, 2 * h_x, -height, height);
    *high_y_wall =
        new ChAAPlaneCB(sys, container.get(), 1, h_y + 0.002, ChVector<>(0, -1, 0), -2 * h_x, 2 * h_x, -height, height);

    sys->RegisterCustomCollisionCallback(*bottom_wall);
    sys->RegisterCustomCollisionCallback(*low_x_wall);
    sys->RegisterCustomCollisionCallback(*high_x_wall);
    sys->RegisterCustomCollisionCallback(*low_y_wall);
    sys->RegisterCustomCollisionCallback(*high_y_wall);

    sys->AddBody(container);
    sys->IncrementGID();
}

inline std::shared_ptr<ChBody> CreateBall(const ChVector<>& pos,
                                          std::shared_ptr<ChMaterialSurfaceSMC> ballMat,
                                          int* ballId,
                                          double m,
                                          ChVector<> inertia,
                                          double radius) {
    auto ball = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
    ball->SetMaterialSurface(ballMat);

    ball->SetIdentifier(*ballId++);
    ball->SetMass(m);
    ball->SetInertiaXX(inertia);
    ball->SetPos(pos);
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetBodyFixed(false);
    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    utils::AddSphereGeometry(ball.get(), radius);
    ball->GetCollisionModel()->BuildModel();
    return ball;
}

size_t AddFallingBalls(ChSystemDistributed* sys, double h_x, double h_y, double gran_height) {
    ChVector<double> box_center(0, 0, lowest_layer + gran_height / 2);
    ChVector<double> half_dims(h_x, h_y, gran_height / 2.0001);

    utils::GridSampler<> sampler(spacing);
    // utils::HCPSampler<> sampler(gran_radius * 2.0);

    auto points = sampler.SampleBox(box_center, half_dims);

    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);

    // Create the falling balls
    int ballId = 0;
    double mass = rho * 4 / 3 * CH_C_PI * gran_radius * gran_radius * gran_radius;
    ChVector<> inertia = (2.0 / 5.0) * mass * gran_radius * gran_radius * ChVector<>(1, 1, 1);

    for (int i = 0; i < points.size(); i++) {
        if (sys->InSub(points[i])) {
            auto ball = CreateBall(points[i], ballMat, &ballId, mass, inertia, gran_radius);
            sys->AddBody(ball);
        }
        sys->IncrementGID();
    }

    return points.size();
}

// TODO need to expand domain decomp
double GetLowerWallPos(double cur_time) {
    return amplitude * std::sin(cur_time * 2 * CH_C_PI / period) + lower_start;
}

int main(int argc, char* argv[]) {
    int num_ranks;
    int my_rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    // Parse program arguments
    int num_threads;
    double time_end;
    double h_x;
    double h_y;
    double h_z;
    std::string outdir;
    bool verbose;
    bool monitor;
    bool output_data;
    if (!GetProblemSpecs(argc, argv, my_rank, num_threads, time_end, h_x, h_y, h_z, monitor, verbose, output_data,
                         outdir)) {
        MPI_Finalize();
        return 1;
    }

    lower_start = -h_x;

    // Output directory and files
    std::ofstream outfile;
    if (output_data) {
        // Create output directory
        if (my_rank == MASTER) {
            bool out_dir_exists = filesystem::path(outdir).exists();
            if (out_dir_exists) {
                std::cout << "Output directory already exists" << std::endl;
                MPI_Abort(MPI_COMM_WORLD, MPI_ERR_OTHER);
                return 1;
            } else if (filesystem::create_directory(filesystem::path(outdir)) && verbose) {
                std::cout << "Create directory = " << filesystem::path(outdir).make_absolute() << std::endl;
            } else {
                std::cout << "Error creating output directory" << std::endl;
                MPI_Abort(MPI_COMM_WORLD, MPI_ERR_OTHER);
                return 1;
            }
        }
    } else if (verbose && my_rank == MASTER) {
        std::cout << "Not writing data files" << std::endl;
    }

    if (verbose && my_rank == MASTER) {
        std::cout << "Number of threads:          " << num_threads << std::endl;
        std::cout << "Domain:                     " << 2 * h_x << " x " << 2 * h_y << " x " << 2 * h_z << std::endl;
        std::cout << "Simulation length:          " << time_end << std::endl;
        std::cout << "Monitor?                    " << monitor << std::endl;
        std::cout << "Output?                     " << output_data << std::endl;
        if (output_data)
            std::cout << "Output directory:           " << outdir << std::endl;
    }

    // Simple Cubic packing density computations:
    double open_X = 2 * h_x;
    double open_Y = 2 * h_y;
    double open_Z = 2 * h_z;
    int count_X = (int)(open_X / spacing);
    int count_Y = (int)(open_Y / spacing);
    int count_Z = (int)(open_Z / spacing);
    int balls_per_layer = count_X * count_Y;
    double gran_height = count_Z * spacing;

    // // Hexagonal Close packing
    // double volume_needed = (4 / 3 * CH_C_PI * gran_radius * gran_radius * gran_radius * num_bodies) / 0.74;
    // double gran_height = volume_needed / (h_x * h_y * 4);

    double height = lowest_layer + gran_height + extra_container_layers * spacing;

    // Create distributed system
    ChSystemDistributed my_sys(MPI_COMM_WORLD, gran_radius * 2, balls_per_layer * count_Z);  // TODO

    if (verbose) {
        if (my_rank == MASTER)
            std::cout << "Running on " << num_ranks << " MPI ranks" << std::endl;
        std::cout << "Rank: " << my_rank << " Node name: " << my_sys.node_name << std::endl;
    }

    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Domain decomposition
    ChVector<double> domlo(-h_x - 0.0001, -h_y - 0.0001, -0.0001);
    ChVector<double> domhi(h_x + 0.0001, h_y + 0.0001, height + 0.0001);
    my_sys.GetDomain()->SetSplitAxis(0);  // Split along the x-axis
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());

    if (verbose)
        my_sys.GetDomain()->PrintDomain();

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;

    int binX;
    int binY;
    int binZ = 1;
    int factor = 4;
    ChVector<> subhi = my_sys.GetDomain()->GetSubHi();
    ChVector<> sublo = my_sys.GetDomain()->GetSubLo();
    ChVector<> subsize = (subhi - sublo) / (2 * gran_radius);
    binX = (int)std::ceil(subsize.x()) / factor;
    binY = (int)std::ceil(subsize.y()) / factor;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(binX, binY, binZ);
    if (verbose)
        printf("Rank: %d   bins: %d %d %d\n", my_rank, binX, binY, binZ);

    // Create objects
    ChAAPlaneCB* bottom_wall;
    ChAAPlaneCB* low_x_wall;
    ChAAPlaneCB* high_x_wall;
    ChAAPlaneCB* low_y_wall;
    ChAAPlaneCB* high_y_wall;

    AddContainer(&my_sys, h_x, h_y, height, &bottom_wall, &low_x_wall, &high_x_wall, &low_y_wall, &high_y_wall);
    auto actual_num_bodies = AddFallingBalls(&my_sys, h_x, h_y, gran_height);
    MPI_Barrier(my_sys.GetMPIWorld());
    if (my_rank == MASTER)
        std::cout << "Total number of particles: " << actual_num_bodies << std::endl;

    // Once the directory has been created, all ranks can make their output files
    MPI_Barrier(MPI_COMM_WORLD);
    std::string out_file_name = outdir + "/Rank" + std::to_string(my_rank) + ".csv";
    outfile.open(out_file_name);
    outfile << "t,gid,x,y,z,vx,vy,vz,U,fixed\n";
    if (verbose)
        std::cout << "Rank: " << my_rank << "  Output file name: " << out_file_name << std::endl;

    // Run simulation for specified time
    int num_steps = (int)std::ceil(time_end / time_step);
    int out_steps = (int)std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    if (verbose && my_rank == MASTER)
        std::cout << "Starting Simulation" << std::endl;

    double t_start = MPI_Wtime();
    for (int i = 0; i < num_steps; i++) {
        my_sys.DoStepDynamics(time_step);
        time += time_step;

        if (i % out_steps == 0) {
            if (my_rank == MASTER)
                std::cout << "Time: " << time << "    elapsed: " << MPI_Wtime() - t_start << std::endl;
            if (output_data) {
                WriteCSV(&outfile, out_frame, &my_sys);
                out_frame++;
            }
        }
        double lower_wall_pos = GetLowerWallPos(time);
        low_x_wall->SetPos(lower_wall_pos);
        high_x_wall->SetPos(lower_wall_pos + 2 * h_x);

        if (monitor)
            Monitor(&my_sys, my_rank);
    }
    double elapsed = MPI_Wtime() - t_start;

    if (my_rank == MASTER)
        std::cout << "\n\nTotal elapsed time = " << elapsed << std::endl;

    if (output_data)
        outfile.close();

    MPI_Finalize();
    return 0;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& num_threads,
                     double& time_end,
                     double& h_x,
                     double& h_y,
                     double& h_z,
                     bool& monitor,
                     bool& verbose,
                     bool& output_data,
                     std::string& outdir) {
    // Initialize parameters.
    num_threads = -1;
    time_end = -1;
    h_x = -1;
    h_y = -1;
    h_z = -1;
    verbose = false;
    monitor = false;
    output_data = false;

    // Create the option parser and pass it the program arguments and the array of valid options.
    CSimpleOptA args(argc, argv, g_options);

    // Then loop for as long as there are arguments to be processed.
    while (args.Next()) {
        // Exit immediately if we encounter an invalid argument.
        if (args.LastError() != SO_SUCCESS) {
            if (rank == MASTER) {
                std::cout << "Invalid argument: " << args.OptionText() << std::endl;
                ShowUsage();
            }
            return false;
        }

        // Process the current argument.
        switch (args.OptionId()) {
            case OPT_HELP:
                if (rank == MASTER)
                    ShowUsage();
                return false;

            case OPT_THREADS:
                num_threads = std::stoi(args.OptionArg());
                break;

            case OPT_OUTPUT_DIR:
                output_data = true;
                outdir = args.OptionArg();
                break;

            case OPT_X:
                h_x = std::stod(args.OptionArg()) / 2.0;
                break;

            case OPT_Y:
                h_y = std::stod(args.OptionArg()) / 2.0;
                break;

            case OPT_Z:
                h_z = std::stod(args.OptionArg()) / 2.0;
                break;

            case OPT_TIME:
                time_end = std::stod(args.OptionArg());
                break;

            case OPT_MONITOR:
                monitor = true;
                break;

            case OPT_VERBOSE:
                verbose = true;
                break;
        }
    }

    // Check that required parameters were specified
    if (num_threads == -1 || time_end <= 0 || h_x < 0 || h_y < 0 || h_z < 0) {
        if (rank == MASTER) {
            std::cout << "Invalid parameter or missing required parameter." << std::endl;
            ShowUsage();
        }
        return false;
    }

    return true;
}

void ShowUsage() {
    std::cout << "Usage: mpirun -np <num_ranks> ./demo_DISTR_scaling [ARGS]" << std::endl;
    std::cout << "-n=<nthreads>   Number of OpenMP threads on each rank [REQUIRED]" << std::endl;
    std::cout << "-x=<xsize>      Patch dimension in X direction [REQUIRED]" << std::endl;
    std::cout << "-y=<ysize>      Patch dimension in Y direction [REQUIRED]" << std::endl;
    std::cout << "-z=<zsize>      Patch dimension in Z direction [REQUIRED]" << std::endl;
    std::cout << "-t=<end_time>   Simulation length [REQUIRED]" << std::endl;
    std::cout << "-o=<outdir>     Output directory (must not exist)" << std::endl;
    std::cout << "-m              Enable performance monitoring (default: false)" << std::endl;
    std::cout << "-v              Enable verbose output (default: false)" << std::endl;
    std::cout << "-h              Print usage help" << std::endl;
}
