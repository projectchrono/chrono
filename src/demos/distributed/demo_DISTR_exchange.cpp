#include <mpi.h>
#include <omp.h>
#include <cstdio>
#include <vector>
#include <cmath>
#include <string>

#include "../../chrono_distributed/physics/ChSystemDistributed.h"
#include "../../chrono_distributed/collision/ChCollisionModelDistributed.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#define MASTER 0

using namespace chrono;
using namespace chrono::collision;

int my_rank;
int num_ranks;

int num_threads;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 1 * CH_C_PI / 20;

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
int count_X = 2;
int count_Y = 2;

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;

const char* out_folder = "../EXCHANGE";

void print(std::string msg);

void OutputData(ChSystemDistributed* sys, int out_frame, double time) {
    std::string filename = "data" + std::to_string(out_frame) + ".csv";
    std::string filedir = "../granular";
    sys->WriteCSV(filedir, filename);

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

    ChVector<> hdim(10, 10, 10);  // 5,5,10
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
void AddFallingBall(ChSystemDistributed* sys) {
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

    ChVector<> pos(0, 0, 51.5);

    auto ball = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
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

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    int num_threads = 1;
    if (argc > 1) {
        num_threads = atoi(argv[1]);
    }

    double time_step = 1e-3;
    double time_end = 10;
    double out_fps = 50;
    unsigned int max_iteration = 100;
    double tolerance = 1e-3;

    ChSystemDistributed my_sys(MPI_COMM_WORLD, 1.0, 100000, std::string("../out") + std::to_string(my_rank) + ".txt");

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

    ChVector<double> domlo(-5, -5, 0);
    ChVector<double> domhi(5, 5, 100);

    my_sys.GetDomain()->SetSplitAxis(0);
    my_sys.GetDomain()->SetSimDomain(domlo.x(), domhi.x(), domlo.y(), domhi.y(), domlo.z(), domhi.z());

    my_sys.GetDomain()->PrintDomain();

    AddContainer(&my_sys);
    AddFallingBall(&my_sys);

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    std::string fileprefix = "../exchangepar";
    for (int i = 0; i < num_steps; i++) {
        if (i % out_steps == 0) {
            OutputData(&my_sys, out_frame, time);
            out_frame++;
            // my_sys.PrintBodyStatus();
            // my_sys.PrintShapeData();
        }
        my_sys.DoStepDynamics(time_step);
        time += time_step;
    }

    MPI_Finalize();
    return 0;
}

void print(std::string msg) {
    if (my_rank == MASTER)
        std::cout << msg;
}
