#include <mpi.h>
#include <omp.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "../../chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "../../chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#define MASTER 0

// ID values to identify command line arguments
enum { OPT_HELP, OPT_THREADS, OPT_NUM_BODIES, OPT_X, OPT_Y, OPT_TIME, OPT_MONITOR, OPT_OUTPUT_DIR, OPT_VERBOSE };

// Table of CSimpleOpt::Soption structures. Each entry specifies:
// - the ID for the option (returned from OptionId() during processing)
// - the option as it should appear on the command line
// - type of the option
// The last entry must be SO_END_OF_OPTIONS
CSimpleOptA::SOption g_options[] = {{OPT_HELP, "--help", SO_NONE},
                                    {OPT_HELP, "-h", SO_NONE},
                                    {OPT_THREADS, "-n", SO_REQ_CMB},
                                    {OPT_NUM_BODIES, "-b", SO_REQ_CMB},
                                    {OPT_X, "-x", SO_REQ_CMB},
                                    {OPT_Y, "-y", SO_REQ_CMB},
                                    {OPT_TIME, "-t", SO_REQ_CMB},
                                    {OPT_MONITOR, "-m", SO_NONE},
                                    {OPT_OUTPUT_DIR, "-o", SO_REQ_CMB},
                                    {OPT_VERBOSE, "-v", SO_NONE},
                                    SO_END_OF_OPTIONS};

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& nthreads,
                     int& n_bodies,
                     bool& monitor,
                     bool& verbose,
                     bool& output_data,
                     std::string& out_dir);
void ShowUsage();

using namespace chrono;
using namespace chrono::collision;

// MPI
int my_rank;
int num_ranks;

// OpenMP
int num_threads = -1;

// Granular Properties
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;
double gran_radius = 0.00125;  // 1.25mm radius
double rho = 4000;
double mass = rho * 4 / 3 * CH_C_PI * gran_radius * gran_radius * gran_radius;  // (4000 kg/m^3)*(4/3 Pi (0.0025m)^3)
double spacing = 1.02 * (2 * gran_radius);  // Distance between adjacent centers of particles

// Dimensions
double h_x;                                              // Set by user - width of EMPTY space
double h_y;                                              // Set by user - width of EMPTY space
double height;                                           // Set by user implicitly
int count_X;                                             // Determined by width
int count_Y;                                             // Determined by depth
int count_Z;                                             // Set by user implicitly
double fill_radius = 0.01;                               // Radius used for sphereical decomposition of the container
double lowest_layer = fill_radius + 3 * spacing;  // Lowest possible CENTER of granular material
int extra_container_layers = 3;
double gran_height;  // Set by user implicitly - Total height of granular material

// Simulation
double time_step = 1e-4;
double time_end = -1;
double out_fps = 10;
unsigned int max_iteration = 100;
double tolerance = 1e-4;

// Options
bool verbose = false;
bool monitor = false;
bool output_data = false;

std::string outdir;
int num_bodies = -1;

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

void print(std::string msg) {
    if (my_rank == MASTER) {
        std::cout << msg << std::flush;
    }
}

void Monitor(chrono::ChSystemParallel* system) {
    double TIME = system->GetChTime();
    double STEP = system->GetTimerStep();
    double BROD = system->GetTimerCollisionBroad();
    double NARR = system->GetTimerCollisionNarrow();
    double SOLVER = system->GetTimerSolver();
    double UPDT = system->GetTimerUpdate();
    double EXCH = system->data_manager->system_timer.GetTime("Exchange");
    int BODS = system->GetNbodies();
    int CNTC = system->GetNcontacts();
    double RESID = 0;
    int REQ_ITS = 0;
    if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(system)) {
        RESID = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetResidual();
        REQ_ITS =
            std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetTotalIterations();
    }

    printf(
        "%d|   %8.5f | %7.4f | E%7.4f | B%7.4f | N%7.4f | %7.4f | %7.4f | %7d | %7d | %7d | "
        "%7.4f\n",
        my_rank, TIME, STEP, EXCH, BROD, NARR, SOLVER, UPDT, BODS, CNTC, REQ_ITS, RESID);
}

// Returns points to put spheres at
void BoxSphereDecomp(std::vector<ChVector<double>>& points, ChVector<double> min, ChVector<double> max, double radius) {
    double incr = radius;

    for (double x = min.x(); x <= max.x(); x += incr) {
        for (double y = min.y(); y <= max.y(); y += incr) {
            for (double z = min.z(); z <= max.z(); z += incr) {
                points.push_back(ChVector<double>(x, y, z));
            }
        }
    }
}

void AddContainerSphereDecomp(ChSystemDistributed* sys) {
    int binId = -200;

    auto mat = std::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    std::vector<ChVector<double>> bottom;
    std::vector<ChVector<double>> side1;
    std::vector<ChVector<double>> side2;
    std::vector<ChVector<double>> side3;
    std::vector<ChVector<double>> side4;

    // Leaves 2*h_x X 2*h_y open for granular material
    ChVector<> hdim(h_x + fill_radius, h_y + fill_radius, height);  // except z is actual height
    double hthick = 0.0;

    BoxSphereDecomp(bottom, ChVector<>(-hdim.x(), -hdim.y(), -hthick), ChVector<>(hdim.x(), hdim.y(), hthick),
                    fill_radius);
    BoxSphereDecomp(side1, ChVector<>(-hdim.x() - hthick, -hdim.y() - hthick, -hthick),
                    ChVector<>(-hdim.x() + hthick, hdim.y() + hthick, hdim.z()), fill_radius);
    BoxSphereDecomp(side2, ChVector<>(hdim.x() - hthick, -hdim.y() - hthick, -hthick),
                    ChVector<>(hdim.x() + hthick, hdim.y() + hthick, hdim.z()), fill_radius);
    BoxSphereDecomp(side3, ChVector<>(-hdim.x() - hthick, -hdim.y() - hthick, -hthick),
                    ChVector<>(hdim.x() + hthick, -hdim.y() + hthick, hdim.z()), fill_radius);
    BoxSphereDecomp(side4, ChVector<>(-hdim.x() - hthick, hdim.y() - hthick, -hthick),
                    ChVector<>(hdim.x() + hthick, hdim.y() + hthick, hdim.z()), fill_radius);

    for (auto itr = bottom.begin(); itr != bottom.end(); itr++) {
        auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
        bin->SetMaterialSurface(mat);
        bin->SetMass(1);
        bin->SetPos(*itr);
        bin->SetCollide(true);
        bin->SetBodyFixed(true);
        bin->GetCollisionModel()->ClearModel();
        bin->GetCollisionModel()->AddSphere(fill_radius);
        bin->GetCollisionModel()->BuildModel();
        sys->AddBody(bin);
        sys->IncrementGID();
    }
    for (auto itr = side1.begin(); itr != side1.end(); itr++) {
        auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
        bin->SetMaterialSurface(mat);
        bin->SetMass(1);
        bin->SetPos(*itr);
        bin->SetCollide(true);
        bin->SetBodyFixed(true);
        bin->GetCollisionModel()->ClearModel();
        bin->GetCollisionModel()->AddSphere(fill_radius);
        bin->GetCollisionModel()->BuildModel();
        sys->AddBody(bin);
        sys->IncrementGID();
    }
    for (auto itr = side2.begin(); itr != side2.end(); itr++) {
        auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
        bin->SetMaterialSurface(mat);
        bin->SetMass(1);
        bin->SetPos(*itr);
        bin->SetCollide(true);
        bin->SetBodyFixed(true);
        bin->GetCollisionModel()->ClearModel();
        bin->GetCollisionModel()->AddSphere(fill_radius);
        bin->GetCollisionModel()->BuildModel();
        sys->AddBody(bin);
        sys->IncrementGID();
    }
    for (auto itr = side3.begin(); itr != side3.end(); itr++) {
        auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
        bin->SetMaterialSurface(mat);
        bin->SetMass(1);
        bin->SetPos(*itr);
        bin->SetCollide(true);
        bin->SetBodyFixed(true);
        bin->GetCollisionModel()->ClearModel();
        bin->GetCollisionModel()->AddSphere(fill_radius);
        bin->GetCollisionModel()->BuildModel();
        sys->AddBody(bin);
        sys->IncrementGID();
    }
    for (auto itr = side4.begin(); itr != side4.end(); itr++) {
        auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
        bin->SetMaterialSurface(mat);
        bin->SetMass(1);
        bin->SetPos(*itr);
        bin->SetCollide(true);
        bin->SetBodyFixed(true);
        bin->GetCollisionModel()->ClearModel();
        bin->GetCollisionModel()->AddSphere(fill_radius);
        bin->GetCollisionModel()->BuildModel();
        sys->AddBody(bin);
        sys->IncrementGID();
    }
}

inline std::shared_ptr<ChBody> CreateBall(double x,
                                          double y,
                                          double z,
                                          std::shared_ptr<ChMaterialSurfaceSMC> ballMat,
                                          int* ballId,
                                          double m,
                                          ChVector<> inertia,
                                          double radius) {
    ChVector<> pos(x, y, z);

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

void AddFallingBalls(ChSystemDistributed* sys) {
	ChVector<double> box_center(0, 0, lowest_layer + gran_height / 2.0);
	ChVector<double> half_dims(h_x, h_y, gran_height / 2.0001);

	std::vector<ChVector<double>> pos_particles;
	utils::GridSampler<> sampler(spacing);
	// utils::HCPSampler<> sampler(gran_radius * 2.0);
	
	utils::Generator::PointVector points = sampler.SampleBox(box_center, half_dims);
	pos_particles.resize(points.size());
	
    for (int i = 0; i < points.size(); i++) {
        pos_particles[i] = ChVector<double>(points[i].x(), points[i].y(), points[i].z());
    }

    if (verbose)
        print(std::string("Adding ") + std::to_string(pos_particles.size()) + " total bodies\n");

    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);

    // Create the falling balls
    int ballId = 0;
    ChVector<> inertia = (2.0 / 5.0) * mass * gran_radius * gran_radius * ChVector<>(1, 1, 1);

    for (int i = 0; i < pos_particles.size(); i++) {
        if (sys->InSub(pos_particles[i])) {
            auto ball = CreateBall(pos_particles[i].x(), pos_particles[i].y(), pos_particles[i].z(), ballMat, &ballId,
                                   mass, inertia, gran_radius);
            sys->AddBody(ball);
        }
        sys->IncrementGID();
    }
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    if (!GetProblemSpecs(argc, argv, my_rank, num_threads, num_bodies, monitor, verbose, output_data, outdir)) {
        MPI_Finalize();
        return 1;
    }

    if (verbose) {
        print(std::string("num_threads = ") + std::to_string(num_threads) + "\nnum_bodies = " +
              std::to_string(num_bodies) + "\nh_x = " + std::to_string(h_x) + "\nh_y = " + std::to_string(h_y) +
              "\nmonitor = " + std::to_string(monitor) + "\nverbose = " + std::to_string(verbose) +
              "\noutput_data = " + std::to_string(output_data) + "\noutdir = " + outdir + "\n");
    }

    if (num_threads == -1 || num_bodies == -1 || time_end < 0) {
        ShowUsage();
        MPI_Finalize();
        return 1;
    }

    // Simple Cubic packing density computations:
    // Open length
    double open_X = 2 * h_x;
    // Open width
    double open_Y = 2 * h_y;

    count_X = open_X / spacing;
    count_Y = open_Y / spacing;

    int balls_per_layer = count_X * count_Y;

    if (verbose)
        print(std::string("Balls per layer: ") + std::to_string(balls_per_layer) + "\n");

    count_Z = (num_bodies + balls_per_layer - 1) / balls_per_layer;
    gran_height = count_Z * spacing;

    // // Hexagonal Close packing
    // double volume_needed = (4 / 3 * CH_C_PI * gran_radius * gran_radius * gran_radius * num_bodies) / 0.74;
    // double gran_height = volume_needed / (h_x * h_y * 4);

    height = lowest_layer + gran_height + extra_container_layers * spacing;

    std::ofstream outfile;
    if (output_data) {
        // Create output directory and output file
        std::string out_file_name = outdir + "Rank";
        out_file_name += std::to_string(my_rank) + ".csv";
        if (my_rank == MASTER) {
            bool out_dir_exists = filesystem::path(outdir).exists();
            if (out_dir_exists) {
                print("Output directory already exists\n");
                MPI_Abort(MPI_COMM_WORLD, MPI_ERR_OTHER);
                return 1;
            } else if (filesystem::create_directory(filesystem::path(outdir)) && verbose) {
                std::cout << "Create directory = " << filesystem::path(outdir).make_absolute() << std::endl;
            } else {
                print("Error creating output directory\n");
                MPI_Finalize();
                return 1;
            }
        }

        // Once the directory has been created, all ranks can make their output files
        MPI_Barrier(MPI_COMM_WORLD);
        if (verbose)
            print(std::string("Outfile: ") + out_file_name + "\n");

        outfile.open(out_file_name);
        outfile << "t,gid,x,y,z,vx,vy,vz,U,fixed\n";
    } else if (verbose) {
        print("Not writing data files\n");
    }

    if (verbose)
        print(std::string("Running on ") + std::to_string(num_ranks) + " MPI ranks.\n");

    ChSystemDistributed my_sys(MPI_COMM_WORLD, gran_radius * 2, balls_per_layer * count_Z);  // TODO

    if (verbose)
        std::cout << "Node " << my_sys.node_name << "\n";

    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(100, 50, 1);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    ChVector<double> domlo(-h_x - fill_radius - 0.0001, -h_y - fill_radius - 0.0001, -fill_radius - 0.0001);
    ChVector<double> domhi(h_x + fill_radius + 0.0001, h_y + fill_radius + 0.0001, height + 0.0001);
    my_sys.GetDomain()->SetSplitAxis(0);  // Split along the x-axis
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());

    if (verbose)
        my_sys.GetDomain()->PrintDomain();

    AddContainerSphereDecomp(&my_sys);
    AddFallingBalls(&my_sys);

    MPI_Barrier(my_sys.GetMPIWorld());

    if (verbose)
        print("Done adding bodies\n");

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    if (verbose)
        print("Starting Simulation\n");

    for (int i = 0; i < num_steps; i++) {
        my_sys.DoStepDynamics(time_step);

        // Output CSV file
        if (i % out_steps == 0) {
            if (verbose)
                print(std::string("time = ") + std::to_string(time) + "\n");

            out_frame++;
            if (output_data)
                WriteCSV(&outfile, out_frame, &my_sys);
        }

        if (monitor)
            Monitor(&my_sys);

        time += time_step;
    }

    if (output_data)
        outfile.close();

    MPI_Finalize();
    return 0;
}

bool GetProblemSpecs(int argc,
                     char** argv,
                     int rank,
                     int& nthreads,
                     int& n_bodies,
                     bool& monitor,
                     bool& verbose,
                     bool& output_data,
                     std::string& out_dir) {
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
                ShowUsage();
                return false;

            case OPT_THREADS:
                nthreads = std::stoi(args.OptionArg());
                break;

            case OPT_OUTPUT_DIR:
                output_data = true;
                out_dir = args.OptionArg();
                break;

            case OPT_NUM_BODIES:
                n_bodies = std::stoi(args.OptionArg());
                break;

            case OPT_X:
                h_x = std::stod(args.OptionArg()) / 2.0;
                break;

            case OPT_Y:
                h_y = std::stod(args.OptionArg()) / 2.0;
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

    return true;
}

void ShowUsage() {
    print(std::string("Usage: mpirun -np <num_ranks> ./demo_DISTR_scaling ") +
          "-n=<nthreads> -b=<approx num_bodies> -x=<open length in x> -y=<open length in y> -t=<end time>\n"
          "\t-o=<outdir (with /)>\n"
          "\t-m -- monitor\n"
          "\t-h -- help\n"
          "\t-v -- verbose\n");
}