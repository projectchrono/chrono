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

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
int count_X;
int count_Y;

int count_Z;

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

    // Create the containing bin
    auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(30, 30, 20);
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

// Returns points to put spheres at
void BoxSphereDecomp(std::vector<ChVector<double>>& points, ChVector<double> min, ChVector<double> max, double radius) {
	double incr = radius; // TODO tune
	
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

    ChVector<> hdim(5, 5, 15);  // except z...?
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
                ChVector<> pos(0.35 * ix, 0.35 * iy, z);

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

    bool monitor;

    int num_threads = 1;
    std::string outdir;

    if (argc > 6) {
        num_threads = atoi(argv[1]);
        monitor = (bool)atoi(argv[2]);
        count_X = atoi(argv[3]);
        count_Y = atoi(argv[4]);
        count_Z = atoi(argv[5]);
        outdir = std::string(argv[6]);
    } else {
        std::cout << "Usage: mpirun -np <num_ranks> ./demo_DISTR_rotgrav <nthreads> <monitorflag> <count_X> <count_Y> <count_Z> "
                     "<outdir (without /)>\n";
        return 1;
    }

    std::string outfile_name = outdir + "/Rank";
    outfile_name += std::to_string(my_rank) + ".csv";
    std::cout << "Outfile name: " << outfile_name << "\n";
    std::ofstream outfile;
    outfile.open(outfile_name);

    outfile << "t,g,x,y,z,vx,vy,vz,U\n";

    omp_set_num_threads(num_threads);

    int thread_count = 0;

    std::cout << "Running on " << num_ranks << " MPI ranks.\n";
    std::cout << "Running on " << thread_count << " OpenMP threads.\n";

    double time_step = 1e-4; // TODO ?
    double time_end = 30;

    double out_fps = 10;

    unsigned int max_iteration = 100;
    double tolerance = 1e-3;

    ChSystemDistributed my_sys(MPI_COMM_WORLD, 0.25, 300000); // TODO

    std::cout << "Node " << my_sys.node_name << "\n";
    my_sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    my_sys.Set_G_acc(ChVector<double>(5, 0.01, -9.8)); // TODO

    // Set solver parameters
    my_sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    my_sys.GetSettings()->solver.tolerance = tolerance;

    my_sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    my_sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    my_sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    my_sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    ChVector<double> domlo(-6, -6, -1); // TODO
    ChVector<double> domhi(6, 6, 25); // TODO
    my_sys.GetDomain()->SetSplitAxis(0);
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());
    my_sys.GetDomain()->PrintDomain();

    AddContainerSphereDecomp(&my_sys);
    AddFallingBalls(&my_sys);

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    int checkpoint = std::ceil(3 / time_step); // TODO
    for (int i = 0; i < num_steps; i++) {
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

        if (i % out_steps == 0) {
            OutputData(&my_sys, out_frame, time);
            out_frame++;
            WriteCSV(&outfile, i, &my_sys);
			
            real3 min = my_sys.data_manager->measures.collision.rigid_min_bounding_point;
            real3 max = my_sys.data_manager->measures.collision.rigid_max_bounding_point;
            std::cout << "Min: " << min[0] << " " << min[1] << " " << min[2] << " Max: " << max[0] << " " << max[1]
                      << " " << max[2] << "\n";
        }

        time += time_step;
    }

    outfile.close();
    MPI_Finalize();

    return 0;
}