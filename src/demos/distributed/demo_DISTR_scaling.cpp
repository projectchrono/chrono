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
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#define MASTER 0

using namespace chrono;
using namespace chrono::collision;

// MPI
int my_rank;
int num_ranks;

// OpenMP
int num_threads;

// Granular Properties
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;
double gran_radius = 0.00125;  // 2.5mm radius
double rho = 4000;
double mass = rho * 4 / 3 * 3.14159265 * gran_radius * gran_radius * gran_radius;  // (4000 kg/m^3)*(4/3 Pi (0.0025m)^3)
double spacing = 1.02 * (2 * gran_radius);

// Dimensions
double h_x = 0.5;  // 1 m^2 base
double h_y = 0.5;
double height;                                           // Set by user implicitly
int count_X;                                             // Determined by width
int count_Y;                                             // Determined by depth
int count_Z;                                             // Set by user implicitly
double fill_radius = 0.01;                               // Radius used for sphereical decomposition of the container
double lowest_layer = fill_radius + 1.01 * gran_radius;  // Lowest possible CENTER of granular material
int extra_container_layers = 10;

// Simulation
double time_step = 1e-4;           // TODO
double time_end = 6;               // TODO
double out_fps = 10;               // TODO
unsigned int max_iteration = 100;  // TODO
double tolerance = 1e-3;           // TODO

void WriteCSV(std::ofstream* file, int timestep_i, ChSystemDistributed* sys) {
    std::stringstream ss_particles;

    int i = 0;
    auto bl_itr = sys->data_manager->body_list->begin();

    for (; bl_itr != sys->data_manager->body_list->end(); bl_itr++, i++) {
        if (sys->ddm->comm_status[i] != chrono::distributed::EMPTY) {
            ChVector<> pos = (*bl_itr)->GetPos();
            ChVector<> vel = (*bl_itr)->GetPos_dt();

            ss_particles << timestep_i << "," << (*bl_itr)->GetGid() << "," << pos.x() << "," << pos.y() << ","
                         << pos.z() << "," << vel.x() << "," << vel.y() << "," << vel.z() << "," << vel.Length()
                         << std::endl;
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

    double B1 = system->data_manager->system_timer.GetTime("B1");
    double B2 = system->data_manager->system_timer.GetTime("B2");
    double B3 = system->data_manager->system_timer.GetTime("B3");
    double B4 = system->data_manager->system_timer.GetTime("B4");
    double B5 = system->data_manager->system_timer.GetTime("B5");

    double A = system->data_manager->system_timer.GetTime("A");

    double NARR = system->GetTimerCollisionNarrow();
    double SOLVER = system->GetTimerSolver();
    double UPDT = system->GetTimerUpdate();
    double SEND = system->data_manager->system_timer.GetTime("Send");
    double RECV = system->data_manager->system_timer.GetTime("Recv");
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
        "%d|   %8.5f | %7.4f | E%7.4f | S%7.4f | R%7.4f | B%7.4f | B1%7.4f | B2%7.4f | B3%7.4f | B4%7.4f | B5%7.4f | "
        "A%7.4f | N%7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f\n",
        my_rank, TIME, STEP, EXCH, SEND, RECV, BROD, B1, B2, B3, B4, B5, A, NARR, SOLVER, UPDT, BODS, CNTC, REQ_ITS,
        RESID);
}

void OutputData(ChSystemDistributed* sys, int out_frame, double time) {
    std::cout << "time = " << time << std::flush << std::endl;
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

    ChVector<> hdim(h_x, h_y, height);  // except z is actual height
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

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemDistributed* sys) {
    ChVector<> subhi(sys->GetDomain()->GetSubHi());
    ChVector<> sublo(sys->GetDomain()->GetSubLo());
    const int split_axis = sys->GetDomain()->GetSplitAxis();
    const double ghost = sys->GetGhostLayer();

    // Common material
    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    int ballId = 0;
    ChVector<> inertia = (2.0 / 5.0) * mass * gran_radius * gran_radius * ChVector<>(1, 1, 1);

    // TODO TODO: Generate all positions in this loop (get rid of continues), have a function
    // in system that checks if in the subdomain using only the postion vector, have an incrementor
    // for num_bodies_global, get rid of the increment in AddBody, get rid of AddBodyTrust?, get rid of
    // appending the rank number to the gid
    int count = 0;
    int count_down_Z = count_Z;
    for (double z = lowest_layer; count_down_Z > 0; z += spacing, count_down_Z--) {
        print(std::string("Layers remaining: ") + std::to_string(count_down_Z) + "\n");
        for (double x = -h_x + fill_radius + gran_radius; x <= h_x - fill_radius - gran_radius; x += spacing) {
            for (double y = -h_y + fill_radius + gran_radius; y <= h_y - fill_radius - gran_radius; y += spacing) {
                ChVector<double> pos(x, y, z);
                if (sys->InSub(pos)) {
                    auto ball = CreateBall(x, y, z, ballMat, &ballId, mass, inertia, gran_radius);
                    sys->AddBody(ball);
                    count++;
                }
                sys->IncrementGID();
            }
        }
    }
    std::cout << "Done adding " << count << " bodies on rank " << my_rank << "\n";
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    bool monitor;
    bool output_data;

    int num_threads = 1;
    std::string outdir;
    int num_bodies;

    double open_X =
        (2 * h_x - 2 * fill_radius - 2 * 1.02 * gran_radius);  // Open length in which a sphere CENTERED can be placed
    double open_Y = (2 * h_y - 2 * fill_radius - 2 * 1.02 * gran_radius);  // Open width " " "

    count_X = open_X / (2 * 1.02 * gran_radius);
    count_Y = open_Y / (2 * 1.02 * gran_radius);

    int balls_per_layer = count_X * count_Y;
    std::cout << "Balls per layer: " << balls_per_layer << std::endl;

    if (argc == 6) {
        num_threads = atoi(argv[1]);
        monitor = (bool)atoi(argv[2]);
        outdir = std::string(argv[3]);
        num_bodies = atoi(argv[4]);
        output_data = (bool)atoi(argv[5]);
    } else {
        std::cout << "Usage: mpirun -np <num_ranks> " << argv[0]
                  << " <nthreads> <monitor flag> <outdir (with /)> "
                     "<approx num_bodies> <output_data flag>\n";
        return 1;
    }

    count_Z = (num_bodies + balls_per_layer - 1) / balls_per_layer;
    height = lowest_layer + (count_Z + extra_container_layers) * spacing;

    std::ofstream outfile;
    if (output_data) {
        std::string outfile_name = outdir + "Rank";
        outfile_name += std::to_string(my_rank) + ".csv";
        std::cout << "Outfile: " << outfile_name << "\n";
        outfile.open(outfile_name);

        outfile << "t,g,x,y,z,vx,vy,vz,U\n";
    } else {
        std::cout << "Not writing data files\n";
    }

    std::cout << "Running on " << num_ranks << " MPI ranks.\n";

    ChSystemDistributed my_sys(MPI_COMM_WORLD, gran_radius * 2, balls_per_layer * count_Z);  // TODO

    std::cout << "Node " << my_sys.node_name << "\n";
    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    ChVector<double> domlo(-h_x - 0.0001, -h_y - 0.0001, -fill_radius - 0.0001);
    ChVector<double> domhi(h_x + 0.0001, h_y + 0.0001, height + 0.0001);
    my_sys.GetDomain()->SetSplitAxis(1);
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());
    my_sys.GetDomain()->PrintDomain();

    AddContainerSphereDecomp(&my_sys);
    AddFallingBalls(&my_sys);

    MPI_Barrier(my_sys.GetMPIWorld());
    print("Done adding bodies\n");

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    print("Starting Simulation\n");
    for (int i = 0; i < num_steps; i++) {
        my_sys.DoStepDynamics(time_step);

        // Output CSV file
        if (i % out_steps == 0) {
            OutputData(&my_sys, out_frame, time);
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