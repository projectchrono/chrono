// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
//
// Chrono::Multicore test program using penalty method for frictional contact.
//
// The model simulated here consists of a number of spherical objects falling
// onto a mixer blade attached through a revolute joint to the ground.
//
// The global reference frame has Z up.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> AddContainer(ChSystemMulticoreSMC* sys) {
    // Create a common material
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetYoungModulus(2e5f);
    mat->SetFriction(0.4f);
    mat->SetRestitution(0.1f);

    // Create the containing bin (2 x 2 x 1)
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetMass(1);
    bin->SetPos(ChVector3d(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->EnableCollision(true);
    bin->SetFixed(true);

    utils::AddBoxContainer(bin, mat,                                 //
                           ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT),  //
                           ChVector3d(2, 2, 1), 0.2,                 //
                           ChVector3i(2, 2, -1));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->DisallowCollisionsWith(2);

    sys->AddBody(bin);

    // The rotating mixer body (1.6 x 0.2 x 0.4)
    auto mixer = chrono_types::make_shared<ChBody>();
    mixer->SetMass(10.0);
    mixer->SetInertiaXX(ChVector3d(50, 50, 50));
    mixer->SetPos(ChVector3d(0, 0, 0.205));
    mixer->SetFixed(false);
    mixer->EnableCollision(true);

    ChVector3d hsize(0.8, 0.1, 0.2);

    utils::AddBoxGeometry(mixer.get(), mat, hsize);
    mixer->GetCollisionModel()->SetFamily(2);

    sys->AddBody(mixer);

    // Create a motor between the two bodies, constrained to rotate at 90 deg/s
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(mixer, bin, ChFrame<>(ChVector3d(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, CH_PI / 2));
    sys->AddLink(motor);

    return mixer;
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a unfiorm rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticoreSMC* sys) {
    // Common material
    auto ballMat = chrono_types::make_shared<ChContactMaterialSMC>();
    ballMat->SetYoungModulus(2e5f);
    ballMat->SetFriction(0.4f);
    ballMat->SetRestitution(0.1f);

    // Create the falling balls
    double mass = 1;
    double radius = 0.1;
    ChVector3d inertia = (2.0 / 5.0) * mass * radius * radius * ChVector3d(1, 1, 1);

    for (int ix = -2; ix < 3; ix++) {
        for (int iy = -2; iy < 3; iy++) {
            ChVector3d pos(0.4 * ix, 0.4 * iy, 1);

            auto ball = chrono_types::make_shared<ChBody>();
            ball->SetMass(mass);
            ball->SetInertiaXX(inertia);
            ball->SetPos(pos);
            ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
            ball->SetFixed(false);
            ball->EnableCollision(true);

            utils::AddSphereGeometry(ball.get(), ballMat, radius);

            sys->AddBody(ball);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-4;

    uint max_iteration = 50;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemMulticoreSMC sys;

    // Set associated collision detection system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set number of threads
    sys.SetNumThreads(8);

    // Set gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));

    // Settings for the broad-phase collision detection
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Select the narrow phase collision algorithm
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    // Set tolerance and maximum number of iterations for bilateral constraint solver
    sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    sys.GetSettings()->solver.tolerance = tolerance;

    // Create the fixed and moving bodies
    // ----------------------------------

    auto mixer = AddContainer(&sys);
    AddFallingBalls(&sys);

    // Perform the simulation
    // ----------------------

#ifdef CHRONO_VSG
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Mixer SMC");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->AddCamera(ChVector3d(0.6, -2, 3), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->SetUseSkyBox(true);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(0.75f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI / 6);
    vis->SetShadows(true);
    vis->SetWireFrameMode(false);
    vis->Initialize();

    while (vis->Run()) {
        sys.DoStepDynamics(time_step);
        vis->Render();

        ////auto frc = mixer->GetAppliedForce();
        ////auto trq = mixer->GetAppliedTorque();
        ////std::cout << sys.GetChTime() << "  force: " << frc << "  torque: " << trq << std::endl;
    }
#else
    // Run simulation for specified time
    double time_end = 1;
    int num_steps = (int)std::ceil(time_end / time_step);
    double time = 0;
    for (int i = 0; i < num_steps; i++) {
        sys.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
