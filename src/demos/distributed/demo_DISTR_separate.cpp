#include <mpi.h>
#include <omp.h>
#include <memory>
#include <string>

#include "../../chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "../../chrono_distributed/physics/ChSystemDistributed.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/core/ChFileutils.h"

#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"

using namespace chrono;
using namespace collision;
using namespace postprocess;

int ballId = 0;
int fileCounter = 0;

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 1 * CH_C_PI / 20;

// Material properties (same on bin and balls)
float Y = 2e6f;
float mu = 0.4f;
float cr = 0.4f;

const char* out_folder = "../BALLS_DEM/POVRAY";

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
    bin->SetPos(ChVector<>(50, 10, 1));
    bin->SetRot(Q_from_AngY(tilt_angle));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(50, 10, 1);
    double hthick = 0.1;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), hdim, ChVector<>(50, 10, 1));
    bin->GetCollisionModel()->BuildModel();

    std::shared_ptr<ChBoxShape> shape(new ChBoxShape);
    shape->GetBoxGeometry().Pos = bin->GetPos();
    shape->GetBoxGeometry().Size = hdim;
    bin->AddAsset(shape);

    auto bodycolor = std::make_shared<ChColorAsset>();
    bodycolor->SetColor(ChColor(0.3, 0.3, 0.6));
    bin->AddAsset(bodycolor);

    sys->AddBody(bin);
}

void AddBalls0(ChSystemDistributed* sys) {
    // Common material
    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);
    for (double z = 3; z < 6; z++) {
        for (double x = 20; x < 23; x++) {
            for (double y = 1; y < 5; y++) {
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

                std::shared_ptr<ChSphereShape> shape(new ChSphereShape);
                shape->GetSphereGeometry().center.Set(pos);
                shape->GetSphereGeometry().rad = radius;
                ball->AddAsset(shape);

                auto bodycolor = std::make_shared<ChColorAsset>();
                bodycolor->SetColor(ChColor(0.3, 0.3, 0.6));
                ball->AddAsset(bodycolor);

                sys->AddBody(ball);
            }
        }
    }
}
void AddBalls1(ChSystemDistributed* sys) {
    // Common material
    auto ballMat = std::make_shared<ChMaterialSurfaceSMC>();
    ballMat->SetYoungModulus(Y);
    ballMat->SetFriction(mu);
    ballMat->SetRestitution(cr);
    ballMat->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling balls
    double mass = 1;
    double radius = 0.15;
    ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);
    for (double z = 3; z < 6; z++) {
        for (double x = 65; x < 68; x++) {
            for (double y = 1; y <= 5; y++) {
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

                std::shared_ptr<ChSphereShape> shape(new ChSphereShape);
                shape->GetSphereGeometry().center.Set(pos);
                shape->GetSphereGeometry().rad = radius;
                ball->AddAsset(shape);

                auto bodycolor = std::make_shared<ChColorAsset>();
                bodycolor->SetColor(ChColor(0.3, 0.3, 0.6));
                ball->AddAsset(bodycolor);

                sys->AddBody(ball);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);
    int my_rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);
    ChSystemDistributed sys(MPI_COMM_WORLD, 1, 10000, std::string("../out") + std::to_string(my_rank) + ".txt");
    sys.GetDomain()->SetSimDomain(0, 100, 0, 20, 0, 20);

    int num_threads = 0;
#pragma omp parallel reduction(+ : num_threads)
    { num_threads++; }

    double time_step = 1e-3;
    double time_end = 2;

    double out_fps = 50;

    unsigned int max_iteration = 100;
    double tolerance = 1e-3;
    sys.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

    sys.Set_G_acc(ChVector<double>(0, 0, -9.8));

    // Set solver parameters
    sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    sys.GetSettings()->solver.tolerance = tolerance;

    sys.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_R;
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    sys.GetSettings()->solver.adhesion_force_model = ChSystemSMC::AdhesionForceModel::Constant;

    AddContainer(&sys);
    AddBalls0(&sys);
    AddBalls1(&sys);

    // Run simulation for specified time
    int num_steps = std::ceil(time_end / time_step);
    int out_steps = std::ceil((1 / time_step) / out_fps);
    int out_frame = 0;
    double time = 0;

    for (int i = 0; i < num_steps; i++) {
        if (i % out_steps == 0) {
            OutputData(&sys, out_frame, time);
            out_frame++;
            sys.PrintBodyStatus();
        }
        sys.DoStepDynamics(time_step);
        time += time_step;
    }

    MPI_Finalize();
    return 0;
}
