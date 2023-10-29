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

#include "chrono_models/vehicle/hmmwv/tire/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

using namespace chrono;
using namespace chrono::vehicle;

int main() {
    // Create sys
    // -------------

    ChSystemMulticoreNSC sys;
    double step_size = 5e-3;
    double tire_step_size = 1e-4;
    sys.ChangeSolverType(SolverType::APGD);

    ////ChSystemMulticoreSMC sys;
    ////double step_size = 1e-4;
    ////double tire_step_size = 1e-4;

    sys.SetNumThreads(8);

    sys.GetSettings()->solver.tolerance = 1e-5;
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = 0;
    sys.GetSettings()->solver.max_iteration_sliding = 150;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = 50;
    sys.GetSettings()->solver.compute_N = false;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.cache_step_length = true;
    sys.GetSettings()->solver.use_full_inertia_tensor = false;
    sys.GetSettings()->solver.contact_recovery_speed = 1000;
    sys.GetSettings()->solver.bilateral_clamp_speed = 1e8;

    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");
    auto tire = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("Rigid tire");

    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, &sys);

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
    sys.GetSettings()->collision.narrowphase_algorithm = collision::ChNarrowphase::Algorithm::HYBRID;
    sys.GetSettings()->collision.broadphase_grid = collision::ChBroadphase::GridType::FIXED_RESOLUTION;
    sys.GetSettings()->collision.bins_per_axis = vec3(collision_bins.x(), collision_bins.y(), collision_bins.z());

    switch (sys.GetContactMethod()) {
        case ChContactMethod::NSC:
            sys.GetSettings()->collision.collision_envelope = collision_envelope;
            break;
        case ChContactMethod::SMC:
            sys.GetSettings()->collision.collision_envelope = 0;
            break;
    }

    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10.0));

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(1.0));

    // Scenario: imobilized wheel (same scenario could be obtained using ChTireTestRig::Mode::DROP in Initialize())
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.0));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.0));

    // Scenario: prescribe all motion functions
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.2));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10.0));
    ////rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunction_Sine>(0, 0.6, 0.2));

    // Scenario: specified longitudinal slip
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(1.0);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    // Display settings
    // ----------------

    std::cout << "Total rig mass: " << rig.GetMass() << std::endl;

    // Initialize OpenGL
    // -----------------

    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Granular terrain demo");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::SOLID);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, 3, 0), ChVector<>(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    // Perform the simulation
    // ----------------------

    while (vis.Run()) {
        rig.Advance(step_size);

        double body_x = rig.GetPos().x();
        double buffer_dist = 0;
        ChVector<> cam_loc(body_x + buffer_dist, 3, -0.5);
        ChVector<> cam_point(body_x + buffer_dist, 0, -0.5);
        vis.UpdateCamera(cam_loc, cam_point);
        vis.Render();

        ////sys.GetContactContainer()->ComputeContactForces();
        ////std::cout << sys.GetChTime() << std::endl;
        ////auto long_slip = tire->GetLongitudinalSlip();
        ////auto slip_angle = tire->GetSlipAngle();
        ////auto camber_angle = tire->GetCamberAngle();
        ////std::cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << std::endl;
        ////auto tforce = rig.ReportTireForce();
        ////auto frc = tforce.force;
        ////auto pnt = tforce.point;
        ////auto trq = tforce.moment;
        ////std::cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
        ////std::cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
        ////std::cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
    }

    return 0;
}
