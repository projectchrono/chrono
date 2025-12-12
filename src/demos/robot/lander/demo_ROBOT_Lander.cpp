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
// Authors: Huzaifa Mustafa Unjhawala
// =============================================================================
//
// Demo code for a lunar lander simulation
// - Simple cylinder body with 4 rigidly attached legs
// - Drops onto a rigid platform under lunar gravity
// - Uses NSC contact model with Bullet collision detection
// - Visualized with VSG
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/core/ChRealtimeStep.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "demos/SetChronoSolver.h"

#include "model/Lander.h"

using namespace chrono;

// Lunar gravity acceleration (m/s^2)
const double LUNAR_GRAVITY = 1.62;

// Platform parameters
const double PLATFORM_SIZE_X = 10.0;  // meters
const double PLATFORM_SIZE_Y = 10.0;  // meters
const double PLATFORM_SIZE_Z = 0.5;   // meters

// Drop height (2m above the legs) - used to calculate initial velocity if USE_DROP_HEIGHT_MODE is true
const double DROP_HEIGHT = 2.0;

// Initial velocity mode switch: true = calculate from drop height (v = sqrt(2*g*h)), false = use INITIAL_VELOCITY
// directly
const bool USE_DROP_HEIGHT_MODE = true;  // Set to false to use INITIAL_VELOCITY directly

// Initial velocity (m/s) - used when USE_DROP_HEIGHT_MODE is false (negative = downward)
const double INITIAL_VELOCITY = -2.5;  // m/s downward

// Clearance above ground when starting simulation (m)
const double GROUND_CLEARANCE = 0.01;  // meters

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the physical system
    ChSystemNSC sys;
    // ChSystemSMC sys;
    // Simulation loop
    double step_size = 0.0001;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    SetChronoSolver(sys, ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Set lunar gravity (downward in Z direction)
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -LUNAR_GRAVITY));

    // Create contact material
    auto contact_material = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_material->SetFriction(0.7f);
    contact_material->SetRestitution(0.05f);
    // contact_material->SetCompliance(1e-5f);  // Inverse of stiffness K
    // contact_material->SetDampingF(0.02);     // Units is time
    // auto contact_material = chrono_types::make_shared<ChContactMaterialSMC>();
    // contact_material->SetFriction(0.7f);
    // contact_material->SetRestitution(0.05f);
    // contact_material->SetKn(1e10f);
    // contact_material->SetGn(1e8f);

    // =============================================================================
    // Create the lander using the Lander class
    // =============================================================================
    Lander lander(&sys, contact_material);

    // Optionally configure lander parameters (using defaults from demo if not set)
    // lander.SetCylinderParameters(4.0, 1.0, 2000.0);
    // lander.SetLegParameters(1.5, 0.05, CH_PI / 6.0, 0.6, 10.0);
    // lander.SetFootpadParameters(5.0 * 0.05, 0.02, 0.02, 2.0);
    // lander.SetUseFootpads(true);
    // lander.SetUseSphericalJoint(false);

    // Position the lander: place it just above the ground (GROUND_CLEARANCE)
    // The lowest point (footpad or leg end) will be at platform_top + GROUND_CLEARANCE
    double platform_top = PLATFORM_SIZE_Z / 2.0;
    ChVector3d lowest_point_pos(0, 0, platform_top);
    ChFrame<> lander_pos(lowest_point_pos, QUNIT);

    // Initialize the lander at the specified position
    lander.Initialize(lander_pos, GROUND_CLEARANCE);

    // =============================================================================
    // Set initial velocity to simulate drop from height
    // =============================================================================
    if (USE_DROP_HEIGHT_MODE) {
        // Calculate velocity from drop height: v = sqrt(2 * g * h)
        lander.SetInitialVelocityFromDropHeight(DROP_HEIGHT, LUNAR_GRAVITY);
    } else {
        // Use direct velocity value (should be negative for downward)
        lander.SetInitialVelocity(ChVector3d(0, 0, INITIAL_VELOCITY));
    }

    // =============================================================================
    // Create the rigid platform
    // =============================================================================
    auto platform = chrono_types::make_shared<ChBody>();
    platform->SetName("Platform");
    platform->SetFixed(true);  // Fixed platform
    platform->SetPos(ChVector3d(0, 0, 0));

    // Add collision shape for platform
    auto platform_collision = chrono_types::make_shared<ChCollisionShapeBox>(contact_material, PLATFORM_SIZE_X,
                                                                             PLATFORM_SIZE_Y, PLATFORM_SIZE_Z);
    platform->AddCollisionShape(platform_collision, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    platform->EnableCollision(true);

    // Add visual shape for platform
    auto platform_visual =
        chrono_types::make_shared<ChVisualShapeBox>(PLATFORM_SIZE_X, PLATFORM_SIZE_Y, PLATFORM_SIZE_Z);
    platform_visual->SetColor(ChColor(0.5f, 0.5f, 0.5f));  // Dark gray
    platform->AddVisualShape(platform_visual, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));

    sys.Add(platform);

    // =============================================================================
    // Create VSG visualization
    // =============================================================================

    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    auto vis_VSG = chrono_types::make_shared<ChVisualSystemVSG>();
    vis_VSG->AttachSystem(&sys);
    vis_VSG->SetWindowTitle("Lunar Lander Simulation");
    vis_VSG->SetWindowSize(ChVector2i(1280, 720));
    vis_VSG->SetWindowPosition(ChVector2i(100, 100));
    vis_VSG->SetCameraVertical(CameraVerticalDir::Z);
    vis_VSG->AddCamera(ChVector3d(8, -8, 6), ChVector3d(0, 0, 2));
    vis_VSG->SetCameraAngleDeg(40.0);
    vis_VSG->SetLightIntensity(1.0f);
    vis_VSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis_VSG->SetBackgroundColor(ChColor(0.1f, 0.1f, 0.15f));  // Dark blue-gray (space-like)
    vis_VSG->EnableSkyBox();
    vis_VSG->EnableShadows();
    vis_VSG->Initialize();

    vis = vis_VSG;
#else
    std::cout << "VSG not available. Cannot visualize." << std::endl;
    return 1;
#endif

    std::cout << "\nStarting simulation..." << std::endl;
    std::cout << "Lander mass: " << lander.GetLanderMass() << " kg" << std::endl;
    std::cout << "Cylinder: length=" << lander.GetCylinderLength() << " m, diameter=" << lander.GetCylinderDiameter()
              << " m" << std::endl;
    std::cout << "Lunar gravity: " << LUNAR_GRAVITY << " m/s^2" << std::endl;
    std::cout << "Initial position: " << GROUND_CLEARANCE << " m above platform" << std::endl;
    if (USE_DROP_HEIGHT_MODE) {
        std::cout << "Velocity mode: Calculated from drop height" << std::endl;
        std::cout << "Drop height: " << DROP_HEIGHT << " m (simulated)" << std::endl;
        double initial_velocity_z = -std::sqrt(2.0 * LUNAR_GRAVITY * DROP_HEIGHT);
        std::cout << "Initial velocity: " << initial_velocity_z << " m/s (downward)" << std::endl;
    } else {
        std::cout << "Velocity mode: Direct specification" << std::endl;
        std::cout << "Initial velocity: " << INITIAL_VELOCITY << " m/s (downward)" << std::endl;
    }
    std::cout << "Footpad joint type: "
              << (lander.GetUseSphericalJoint() ? "Spherical (rotating)" : "Rigid lock (fixed)") << std::endl;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step_size);

        // Print status every second
        if (std::abs(std::fmod(sys.GetChTime(), 1.0)) < step_size) {
            ChVector3d pos = lander.GetBody()->GetPos();
            ChVector3d vel = lander.GetBody()->GetPosDt();
            std::cout << "Time: " << sys.GetChTime() << " s, "
                      << "Position: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ") m, "
                      << "Velocity: " << vel.Length() << " m/s" << std::endl;
        }
    }

    return 0;
}
