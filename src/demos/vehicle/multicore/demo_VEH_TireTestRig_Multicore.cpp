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

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

using namespace chrono;
using namespace chrono::vehicle;

// Terrain type (RIGID or GRANULAR)
ChTireTestRig::TerrainType terrain_type = ChTireTestRig::TerrainType::GRANULAR;

int main() {
    double step_size = 5e-3;
    double tire_step_size = 1e-4;

    // Create sys
    // -------------

    ChSystemMulticoreNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
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

    ////rig.SetCamberAngle(+15 * CH_DEG_TO_RAD);

    rig.SetTireStepsize(tire_step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    ChTireTestRig::TerrainPatchSize size;
    size.length = 3.0;
    size.width = 1.0;
    size.depth = 0.24;

    if (terrain_type == ChTireTestRig::TerrainType::RIGID) {
        ChTireTestRig::TerrainParamsRigid params;
        params.friction = 0.8f;
        params.restitution = 0;
        params.Young_modulus = 2e7f;

        rig.SetTerrainRigid(size, params);
    } else {
        ChTireTestRig::TerrainParamsGranular params;
        params.radius = 0.02;
        params.density = 2000;
        params.friction = 0.9;
        params.cohesion = 1e4;
        params.Young_modulus = 1e7;

        rig.SetTerrainGranular(size, params);
    }

    rig.SetTimeDelay(0.15);

    // Set collision detection parameters
    // ----------------------------------

    double collision_envelope;
    ChVector3i collision_bins;
    rig.GetSuggestedCollisionSettings(collision_envelope, collision_bins);
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    sys.GetSettings()->collision.broadphase_grid = ChBroadphase::GridType::FIXED_RESOLUTION;
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
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10.0));

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(1.0));

    // Scenario: imobilized wheel (same scenario could be obtained using ChTireTestRig::Mode::DROP in Initialize())
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.0));

    // Scenario: prescribe all motion functions
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0.2));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(10.0));
    ////rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunctionSine>(0.2, 0.6));

    // Scenario: specified longitudinal slip
    ////rig.SetConstantLongitudinalSlip(0.2, 0.1);

    // Initialize the tire test rig
    rig.SetTimeDelay(1.0);
    rig.Initialize(ChTireTestRig::Mode::TEST);

    // Display settings
    // ----------------

    std::cout << "Total rig mass: " << rig.GetMass() << std::endl;

    // Initialize run-time visualization
    // ---------------------------------

#ifdef CHRONO_VSG
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Granular terrain demo");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0, 3, 0), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyBox();
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();
#endif

    // Perform the simulation
    // ----------------------

    while (vis->Run()) {
        rig.Advance(step_size);

        double body_x = rig.GetPos().x();
        double buffer_dist = 0;
        ChVector3d cam_loc(body_x + buffer_dist, 3, -0.5);
        ChVector3d cam_point(body_x + buffer_dist, 0, -0.5);
        vis->UpdateCamera(cam_loc, cam_point);
        vis->Render();

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
