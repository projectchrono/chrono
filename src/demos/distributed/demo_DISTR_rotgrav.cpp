#include <mpi.h>
#include <omp.h>
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

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
int count_X;  // 10  // 20
int count_Y;  // 10  // 4

int count_Z;

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;

void WriteCSV(std::ofstream* file, int timestep, ChSystemDistributed* sys);

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

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemDistributed* sys) {
    // IDs for the two bodies
    int binId = -200;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(Y);
    mat->SetFriction(mu);
    mat->SetRestitution(cr);

    // Create the containing bin (4 x 4 x 1)
    auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(4, 4, 15);  // 5,5,10
    double hthick = 0.1;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemDistributed* sys) {
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
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    // TODO generate randomly. Need to seed though.
    for (double z = 10; count_Z > 0; z += 0.35, count_Z--) {
        for (int ix = -count_X; ix <= count_X; ix++) {
            for (int iy = -count_Y; iy <= count_Y; iy++) {
                ChVector<> pos(0.35 * ix, 0.35 * iy, z);  //.4*ix, .4*iy,z

                auto ball =
                    std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
                ball->SetMaterialSurface(ballMat);

                ball->SetIdentifier(ballId++);
                ball->SetMass(mass);
                ball->SetInertiaXX(inertia);
                ball->SetPos(pos);
                ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
                ball->SetBodyFixed(false);
                ball->SetCollide(true);

                ball->GetCollisionModel()->ClearModel();
                utils::AddSphereGeometry(ball.get(), radius);
                ball->GetCollisionModel()->BuildModel();

                sys->AddBody(ball);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);

    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    /* Make the program wait while attaching debuggers */
    if (my_rank == 0) {
        int foo;
        std::cout << "Enter something too continue..." << std::endl;
        std::cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);

    std::string outfile_name = "../granular/Rank";
    outfile_name += my_rank + ".csv";

    std::ofstream outfile;
    outfile.open(outfile_name);

    outfile << "g,t,x,y,z,vx,vy,vz,U\n";

    bool monitor;

    int num_threads = 1;
    if (argc > 5) {
        num_threads = atoi(argv[1]);
        monitor = (bool)atoi(argv[2]);
        count_X = atoi(argv[3]);
        count_Y = atoi(argv[4]);
        count_Z = atoi(argv[5]);
    } else {
        std::cout << "Usage: ./exe <nthreads> <monitorflag> <count_X> <count_Y> <count_Z>\n";
        return 1;
    }

    omp_set_num_threads(num_threads);

    int thread_count = 0;
#pragma omp parallel reduction(+ : thread_count)
    { thread_count++; }

    std::cout << "Running on " << num_ranks << " MPI ranks.\n";
    std::cout << "Running on " << thread_count << " OpenMP threads.\n";

    double time_step = 1e-4;
    double time_end = 10000;

    double out_fps = 10;

    unsigned int max_iteration = 100;
    double tolerance = 1e-3;

    print("Constructing the system...\n");
    ChSystemDistributed my_sys(MPI_COMM_WORLD, 0.25, 100000, std::string("../out") + std::to_string(my_rank) + ".txt");

    my_sys.data_manager->shape_data.id_rigid.reserve(1000);
    std::cout << "Addr of id_rigid[88] " << &(my_sys.data_manager->shape_data.id_rigid[88]) << "\n";

    std::cout << "Node " << my_sys.node_name << "\n";

    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(5, 0.01, -9.8));

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    ChVector<double> domlo(-5, -5, -1);
    ChVector<double> domhi(5, 5, 25);
    my_sys.GetDomain()->SetSplitAxis(0);
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());
    my_sys.GetDomain()->PrintDomain();

    AddContainer(&my_sys);
    AddFallingBalls(&my_sys);

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    std::cout << "out_steps: " << out_steps << "\n";
    int out_frame = 0;
    double time = 0;

    int checkpoint = std::ceil(3 / time_step);
    for (int i = 0; i < num_steps; i++) {
        if (i % out_steps == 0) {
            OutputData(&my_sys, out_frame, time);
            out_frame++;
            WriteCSV(&outfile, time_step, &my_sys);

            real3 min = my_sys.data_manager->measures.collision.rigid_min_bounding_point;
            real3 max = my_sys.data_manager->measures.collision.rigid_max_bounding_point;
            std::cout << "Min: " << min[0] << " " << min[1] << " " << min[2] << " Max: " << max[0] << " " << max[1]
                      << " " << max[2] << "\n";
        }

        if (i % checkpoint == 0) {
            ChVector<double> g(my_sys.Get_G_acc());
            my_sys.Set_G_acc(ChVector<>(-1 * g.x(), g.y(), g.z()));
        }

        if (monitor)
            Monitor(&my_sys);
        my_sys.DoStepDynamics(time_step);

        uint lowest_local_id;
        if (my_sys.GetLowestZ(&lowest_local_id) < 0) {
            GetLog() << "Lowest Local " << lowest_local_id << "\n";
            int waste_time = 10230;
            my_sys.CheckIds();
        }

        time += time_step;
    }

    outfile.close();
    MPI_Finalize();

    return 0;
}

void WriteCSV(std::ofstream* file, int timestep, ChSystemDistributed* sys) {
    std::stringstream ss_particles;

    std::vector<std::shared_ptr<ChBody>>::iterator bl_itr = sys->data_manager->body_list->begin();
    int i = 0;
    for (; bl_itr != sys->data_manager->body_list->end(); bl_itr++, i++) {
        if (sys->ddm->comm_status[i] != chrono::distributed::EMPTY) {
            ChVector<> pos = (*bl_itr)->GetPos();
            ChVector<> vel = (*bl_itr)->GetPos_dt();

            ss_particles << (*bl_itr)->GetGid() << "," << timestep << "," << pos.x() << "," << pos.y() << "," << pos.z()
                         << "," << vel.x() << "," << vel.y() << "," << vel.z() << "," << vel.Length() << std::endl;
        }
    }

    *file << ss_particles.str();
}