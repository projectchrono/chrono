// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// =============================================================================

#include <algorithm>

#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
//
#include "chrono_opengl/ChOpenGLWindow.h"
//
#include "chrono_parallel/physics/ChSystemParallel.h"
//
#include "chrono_thirdparty/filesystem/path.h"
//
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

using namespace chrono;
using namespace chrono::vehicle;

int main() {
    // Create system
    // -------------

    ChSystemParallelNSC system;
    double step_size = 5e-3;
    double tire_step_size = 1e-4;
    system.ChangeSolverType(SolverType::APGD);

    ////ChSystemParallelSMC system;
    ////double step_size = 1e-4;
    ////double tire_step_size = 1e-4;

    int threads = 8;
    int max_threads = CHOMPfunctions::GetNumProcs();
    if (threads > max_threads)
        threads = max_threads;
    system.SetParallelThreadNumber(threads);
    CHOMPfunctions::SetNumThreads(threads);
#pragma omp parallel
#pragma omp master
    std::cout << "Using " << CHOMPfunctions::GetNumThreads() << " threads" << std::endl;
    system.GetSettings()->perform_thread_tuning = false;

    system.GetSettings()->solver.tolerance = 1e-5;
    system.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system.GetSettings()->solver.max_iteration_normal = 0;
    system.GetSettings()->solver.max_iteration_sliding = 150;
    system.GetSettings()->solver.max_iteration_spinning = 0;
    system.GetSettings()->solver.max_iteration_bilateral = 50;
    system.GetSettings()->solver.compute_N = false;
    system.GetSettings()->solver.alpha = 0;
    system.GetSettings()->solver.cache_step_length = true;
    system.GetSettings()->solver.use_full_inertia_tensor = false;
    system.GetSettings()->solver.contact_recovery_speed = 1000;
    system.GetSettings()->solver.bilateral_clamp_speed = 1e8;
    system.GetSettings()->min_threads = threads;

    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = std::make_shared<hmmwv::HMMWV_WheelLeft>("Wheel");
    auto tire = std::make_shared<hmmwv::HMMWV_RigidTire>("Rigid tire");

    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, &system);

    ////rig.SetGravitationalAcceleration(0);
    rig.SetNormalLoad(2000);

    ////rig.SetCamberAngle(+15 * CH_C_DEG_TO_RAD);

    rig.SetTireStepsize(tire_step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    ////rig.SetTerrainRigid(0.8, 2e7, 0, 10.0);
    rig.SetTerrainGranular(0.02, 6, 2000.0, 0.9, 10.0e3, 1e7);

    rig.SetTimeDelay(0.15);

    // Set collision detection parameters
    // ----------------------------------

    double collision_envelope;
    ChVector<int> collision_bins;
    rig.GetSuggestedCollisionSettings(collision_envelope, collision_bins);
    system.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system.GetSettings()->collision.bins_per_axis = vec3(collision_bins.x(), collision_bins.y(), collision_bins.z());
    system.GetSettings()->collision.fixed_bins = true;

    switch (system.GetContactMethod()) {
        case ChMaterialSurface::NSC:
            system.GetSettings()->collision.collision_envelope = collision_envelope;
            break;
        case ChMaterialSurface::SMC:
            system.GetSettings()->collision.collision_envelope = 0;
            break;
    }

    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(std::make_shared<ChFunction_Const>(10.0));
    ////rig.Initialize();

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(std::make_shared<ChFunction_Const>(1.0));
    ////rig.Initialize();

    // Scenario: imobilized wheel
    rig.SetLongSpeedFunction(std::make_shared<ChFunction_Const>(0.0));
    rig.SetAngSpeedFunction(std::make_shared<ChFunction_Const>(0.0));
    rig.Initialize();

    // Scenario: prescribe all motion functions
    ////rig.SetLongSpeedFunction(std::make_shared<ChFunction_Const>(0.2));
    ////rig.SetAngSpeedFunction(std::make_shared<ChFunction_Const>(10.0));
    ////rig.SetSlipAngleFunction(std::make_shared<ChFunction_Sine>(0, 0.6, 0.2));
    ////rig.Initialize();

    // Scenario: specified longitudinal slip
    ////rig.Initialize(0.2, 1.0);

    // Display settings
    // ----------------

    std::cout << "Total rig mass: " << rig.GetTotalMass() << std::endl;
    std::cout << "Applied load:   " << rig.GetAppliedLoad() << std::endl;

    // Initialize OpenGL
    // -----------------

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Granular terrain demo", &system);
    gl_window.SetCamera(ChVector<>(0, 3, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::SOLID);

    // Perform the simulation
    // ----------------------

    system.SetupInitial();

    while (gl_window.Active()) {
        rig.Advance(step_size);

        double body_x = rig.GetPos().x();
        double buffer_dist = 0;
        ChVector<> cam_loc(body_x + buffer_dist, 3, -0.5);
        ChVector<> cam_point(body_x + buffer_dist, 0, -0.5);
        gl_window.SetCamera(cam_loc, cam_point, ChVector<>(0, 0, 1), 0.05f);
        gl_window.Render();

        ////system.GetContactContainer()->ComputeContactForces();
        ////std::cout << system.GetChTime() << std::endl;
        ////auto long_slip = tire->GetLongitudinalSlip();
        ////auto slip_angle = tire->GetSlipAngle();
        ////auto camber_angle = tire->GetCamberAngle();
        ////std::cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << std::endl;
        ////auto tforce = rig.GetTireForce();
        ////auto frc = tforce.force;
        ////auto pnt = tforce.point;
        ////auto trq = tforce.moment;
        ////std::cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
        ////std::cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
        ////std::cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
    }

    return 0;
}
