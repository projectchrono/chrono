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
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> AddContainer(ChSystemMulticoreNSC* sys) {
    // IDs for the two bodies
    int binId = -200;
    int mixerId = -201;

    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    // Create the containing bin (2 x 2 x 1)
    auto bin = std::shared_ptr<ChBody>(sys->NewBody());
    bin->SetIdentifier(binId);
    bin->SetMass(100);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(1, 1, 0.5);
    double hthick = 0.1;

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hthick, hdim.y(), hdim.z()),
                          ChVector<>(hdim.x() + hthick, 0, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, -hdim.y() - hthick, hdim.z()));
    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(hdim.x(), hthick, hdim.z()),
                          ChVector<>(0, hdim.y() + hthick, hdim.z()));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);

    // The rotating mixer body (1.6 x 0.2 x 0.4)
    auto mixer = std::shared_ptr<ChBody>(sys->NewBody());
    mixer->SetIdentifier(mixerId);
    mixer->SetMass(10.0);
    mixer->SetInertiaXX(ChVector<>(50, 50, 50));
    mixer->SetPos(ChVector<>(0, 0, 0.205));
    mixer->SetBodyFixed(false);
    mixer->SetCollide(true);

    ChVector<> hsize(0.8, 0.1, 0.2);

    mixer->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(mixer.get(), mat, hsize);
    mixer->GetCollisionModel()->SetFamily(2);
    mixer->GetCollisionModel()->BuildModel();

    sys->AddBody(mixer);

    // Create a motor between the two bodies, constrained to rotate at 90 deg/s
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(mixer, bin, ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, CH_C_PI / 2));
    sys->AddLink(motor);

    return mixer;
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemMulticore* sys) {
    // Shared contact materials
    auto ball_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ball_mat->SetFriction(0.4f);
    auto cyl_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create the falling objects
    for (int ix = -2; ix < 3; ix++) {
        for (int iy = -2; iy < 3; iy++) {
            ChVector<> b_pos(0.4 * ix, 0.4 * iy, 1);
            ChVector<> c_pos(0.4 * ix, 0.4 * iy, 1.4);

            auto ball = chrono_types::make_shared<ChBodyEasySphere>(0.1, 2000, ball_mat,
                                                                    collision::ChCollisionSystemType::CHRONO);
            ball->SetPos(b_pos);
            sys->AddBody(ball);

            auto cyl = chrono_types::make_shared<ChBodyEasyCylinder>(0.1, 0.05, 2000, cyl_mat,
                                                                     collision::ChCollisionSystemType::CHRONO);
            cyl->SetPos(c_pos);
            sys->AddBody(cyl);
        }
    }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;

    uint max_iteration = 30;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemMulticoreNSC msystem;

    // Set number of threads
    msystem.SetNumThreads(8);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    msystem.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystem.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
    msystem.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.contact_recovery_speed = 10000;
    msystem.ChangeSolverType(SolverType::APGD);
    msystem.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    msystem.GetSettings()->collision.collision_envelope = 0.01;
    msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Enable active bounding box
    ////msystem.GetSettings()->collision.use_aabb_active = true;
    ////msystem.GetSettings()->collision.aabb_min = real3(-1, -1, -1.5);
    ////msystem.GetSettings()->collision.aabb_max = real3(+1, +1, +1.5);

    // Create the fixed and moving bodies
    // ----------------------------------

    auto mixer = AddContainer(&msystem);
    AddFallingBalls(&msystem);

    // Perform the simulation
    // ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.AttachSystem(&msystem);
    gl_window.Initialize(1280, 720, "mixerNSC");
    gl_window.SetCamera(ChVector<>(0, -3, 2), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    while (gl_window.Active()) {
        gl_window.DoStepDynamics(time_step);
        gl_window.Render();

        ////auto frc = mixer->GetAppliedForce();
        ////auto trq = mixer->GetAppliedTorque();
        ////std::cout << msystem.GetChTime() << "  force: " << frc << "  torque: " << trq << std::endl;
    }
#else
    // Run simulation for specified time
    double time_end = 1;
    int num_steps = (int)std::ceil(time_end / time_step);
    double time = 0;
    for (int i = 0; i < num_steps; i++) {
        msystem.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
