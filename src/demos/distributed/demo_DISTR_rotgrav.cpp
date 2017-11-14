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

int my_rank;
int num_ranks;

int num_threads;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 0;

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1) * count_Z
int count_X = 111;  // low x -35.52
int count_Y = 112;  // low y -35.84

int count_Z = 200;  // height: 67

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;

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
        std::cout << msg;
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
    double incr = radius;  // TODO tune

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

    ChVector<> hdim(38, 38, 68);  // except z is actual height
    double hthick = 0.0;
    double fill_radius = 1;  // TODO: ?

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
    }
}

inline std::shared_ptr<ChBody> CreateBall(double x,
                                          double y,
                                          double z,
                                          std::shared_ptr<ChMaterialSurfaceSMC> ballMat,
                                          int* ballId,
                                          double mass,
                                          ChVector<> inertia,
                                          double radius) {
    ChVector<> pos(0.32 * x, 0.32 * y, z);

    auto ball = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
    ball->SetMaterialSurface(ballMat);

    ball->SetIdentifier(*ballId++);
    ball->SetMass(mass);
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
    // int num_bodies_local = ; TODO large heap allocation

    // Common material
    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    int ballId = 0;
    double mass = 1;
    double radius = 0.15;
    double spacing = 2 * radius + 0.02;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    int count_down_Z = count_Z;
    for (double z = 3; count_down_Z > 0; z += spacing, count_down_Z--) {
		print(std::string("Layers remaining: ") + std::to_string(count_down_Z) + "\n");
        if (split_axis == 2 && z >= sublo.z() && z < subhi.z())
            continue;
        for (double x = spacing * -count_X; x <= spacing * count_X; x += spacing) {
            if (split_axis == 0 && x >= sublo.x() && x < subhi.x())
                continue;
            for (double y = spacing * -count_Y; y <= spacing * count_Y; y += spacing) {
                if (split_axis == 1 && y >= sublo.y() && y < subhi.y())
                    continue;

                auto ball = CreateBall(x, y, z, ballMat, &ballId, mass, inertia, radius);
                sys->AddBodyTrust(ball);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    bool monitor;

    int num_threads = 1;
    std::string outdir;
    /*
        // Make the program wait while attaching debuggers
        if (my_rank == 0) {
            int foo;
            std::cout << "Enter something too continue..." << std::endl;
            std::cin >> foo;
        }
        MPI_Barrier(MPI_COMM_WORLD);
    */
    if (argc > 3) {
        num_threads = atoi(argv[1]);
        monitor = (bool)atoi(argv[2]);
        outdir = std::string(argv[3]);
    } else {
        std::cout << "Usage: mpirun -np <num_ranks> ./demo_DISTR_rotgrav <nthreads> <monitorflag> <outdir (with /)>\n";
        return 1;
    }

    std::string outfile_name = outdir + "Rank";
    outfile_name += std::to_string(my_rank) + ".csv";
    std::cout << "Outfile name: " << outfile_name << "\n";
    std::ofstream outfile;
    outfile.open(outfile_name);

    outfile << "t,g,x,y,z,vx,vy,vz,U\n";

    omp_set_num_threads(num_threads);

    std::cout << "Running on " << num_ranks << " MPI ranks.\n";

    double time_step = 1e-4;  // TODO ?
    double time_end = 15;     // TODO

    double out_fps = 10;  // TODO

    unsigned int max_iteration = 100;
    double tolerance = 1e-3;

    ChSystemDistributed my_sys(MPI_COMM_WORLD, 0.25, 11000000);  // TODO

    std::cout << "Node " << my_sys.node_name << "\n";
    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(0, 0, -9.8));  // TODO

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    ChVector<double> domlo(-38, -38, -1);  // TODO
    ChVector<double> domhi(38, 38, 70);    // TODO
    my_sys.GetDomain()->SetSplitAxis(1);   // TODO
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());
    my_sys.GetDomain()->PrintDomain();

    AddContainerSphereDecomp(&my_sys);
    AddFallingBalls(&my_sys);
    my_sys.UpdateRigidBodies();  // NOTE: Must be called to finish body sharing

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    for (int i = 0; i < num_steps; i++) {
        my_sys.DoStepDynamics(time_step);

        if (i % out_steps == 0) {
            OutputData(&my_sys, out_frame, time);
            out_frame++;
            WriteCSV(&outfile, out_frame, &my_sys);
        }
        time += time_step;
    }

    outfile.close();

    MPI_Finalize();
    return 0;
}
