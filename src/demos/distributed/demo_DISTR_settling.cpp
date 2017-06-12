#include <mpi.h>
#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>
#include <memory>

#include "../../chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "../../chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

using namespace chrono;
using namespace chrono::collision;

int my_rank;
int num_ranks;

int num_threads;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 1 * CH_C_PI / 20;

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;

void Monitor(chrono::ChSystemParallel* system) {
    double TIME = system->GetChTime();
    double STEP = system->GetTimerStep();
    double BROD = system->GetTimerCollisionBroad();
    double NARR = system->GetTimerCollisionNarrow();
    double SOLVER = system->GetTimerSolver();
    double UPDT = system->GetTimerUpdate();
    double SEND = system->data_manager->system_timer.GetTime("Send");
    double RECV = system->data_manager->system_timer.GetTime("Recv");
    int BODS = system->GetNbodies();
    int CNTC = system->GetNcontacts();
    double RESID = 0;
    int REQ_ITS = 0;
    if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(system)) {
        RESID = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetResidual();
        REQ_ITS =
            std::static_pointer_cast<chrono::ChIterativeSolverParallel>(system->GetSolver())->GetTotalIterations();
    }

    printf(" %3d | %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f | %7.4f | %7.4f\n", my_rank,
           TIME, STEP, BROD, NARR, SOLVER, UPDT, BODS, CNTC, REQ_ITS, RESID, SEND, RECV);
}

void OutputData(ChSystemDistributed* sys, int out_frame, double time) {
    sys->WriteCSV(out_frame, "../granular");  // TODO
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

    ChVector<> hdim(0.1, 0.5, 1);
    double hthick = 0.005;

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
void AddFallingBalls(ChSystemDistributed* sys, int num_balls) {
    // Common material
    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    int ballId = 0;
    double mass = 0.00408407;
    double radius = 0.0025;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

    ChVector<double> lo(-0.094, -0.494, 0.5);
    ChVector<double> hi(0.094, 0.494, 0);

    int balls_per_layer = std::floor((hi[0] - lo[0]) / 0.006) * std::floor((hi[1] - lo[1]) / 0.006);
    std::cout << "Balls per layer: " << balls_per_layer << "\n";
    int num_layers = std::ceil(num_balls / balls_per_layer);
    std::cout << "Num layers: " << num_layers << "\n";

    // z < 1.7
    double z;
    int layer;
    for (z = 0.5, layer = 0; layer < num_layers; z += 0.006, layer++) {
        for (double y = -0.494; y <= 0.494; y += 0.006) {
            for (double x = -0.094; x <= 0.094; x += 0.006) {
                ChVector<> pos(x, y, z);

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

    int num_threads = 1;
    if (argc > 1) {
        num_threads = atoi(argv[1]);
    }

    int num_balls = 100000;  // Default 10,000
    omp_set_num_threads(num_threads);

    if (argc > 2 && atoi(argv[2]) <= 1000000) {
        num_balls = atoi(argv[2]);
    }

    int thread_count = 0;
#pragma omp parallel reduction(+ : thread_count)
    { thread_count++; }

    std::cout << "Running on " << num_ranks << " MPI ranks.\n";
    std::cout << "Running on " << thread_count << " OpenMP threads.\n";

    double time_step = 1e-4;
    double time_end = 10;
    double out_fps = 50;
    unsigned int max_iteration = 100;
    double tolerance = 1e-3;

    ChSystemDistributed my_sys(MPI_COMM_WORLD, 0.005, 100000);
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

    ChVector<double> domlo(-0.15, -0.51, -0.01);
    ChVector<double> domhi(0.15, 0.51, 1.75);
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());
    my_sys.GetDomain()->PrintDomain();

    AddContainer(&my_sys);
    AddFallingBalls(&my_sys, num_balls);

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    for (int i = 0; i < num_steps; i++) {
        if (i % out_steps == 0) {
            OutputData(&my_sys, out_frame, time);
            out_frame++;
        }

        Monitor(&my_sys);
        my_sys.DoStepDynamics(time_step);
        time += time_step;
    }

    MPI_Finalize();
    return 0;
}
