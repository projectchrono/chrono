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
// Authors: Radu Serban, Victor Bertolazzo
// =============================================================================
//
// Chrono::Multicore test program using NSC method for rolling frictional contact.
//
// The global reference frame has Z up.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"

#include "chrono_opengl/ChVisualSystemOpenGL.h"

using namespace chrono;

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Parameters
    double radius = 0.5;

    double time_step = 0.01;

    uint max_iteration_normal = 0;
    uint max_iteration_sliding = 0;
    uint max_iteration_spinning = 100;
    uint max_iteration_bilateral = 0;

    // Create the multicore sys and set associated collision detection system
    ChSystemMulticoreNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    // Set number of threads
    sys.SetNumThreads(1);

    // Set solver settings
    sys.ChangeSolverType(SolverType::APGD);

    sys.GetSettings()->solver.solver_mode = SolverMode::SPINNING;
    sys.GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    sys.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    sys.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    sys.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.contact_recovery_speed = 1e32;
    sys.GetSettings()->solver.use_full_inertia_tensor = false;
    sys.GetSettings()->solver.tolerance = 0;

    sys.GetSettings()->collision.collision_envelope = 0.1 * radius;
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create the container body
    auto container = chrono_types::make_shared<ChBody>();
    sys.Add(container);
    container->SetPos(ChVector3d(0, 0, 0));
    container->SetFixed(true);
    container->EnableCollision(true);

    // Set rolling and friction coefficients for the container.
    // By default, the composite material will use the minimum value for an interacting collision pair.
    auto bin_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    bin_mat->SetFriction(0.6f);
    bin_mat->SetRollingFriction(1);
    bin_mat->SetSpinningFriction(1);

    utils::AddBoxGeometry(container.get(), bin_mat, ChVector3d(20, 1, 20), ChVector3d(0, -1, 0));
    utils::AddBoxGeometry(container.get(), bin_mat, ChVector3d(1, 2, 20.99), ChVector3d(-10, 0, 0));
    utils::AddBoxGeometry(container.get(), bin_mat, ChVector3d(1, 2, 20.99), ChVector3d(10, 0, 0));
    utils::AddBoxGeometry(container.get(), bin_mat, ChVector3d(20.99, 2, 1), ChVector3d(0, 0, -10));
    utils::AddBoxGeometry(container.get(), bin_mat, ChVector3d(20.99, 2, 1), ChVector3d(0, 0, 10));

    // Create some spheres that roll horizontally, with increasing rolling friction values
    double density = 1000;
    double mass = density * (4.0 / 3.0) * CH_PI * std::pow(radius, 3);
    double inertia = (2.0 / 5.0) * mass * std::pow(radius, 2);
    double initial_angspeed = 10;
    double initial_linspeed = initial_angspeed * radius;

    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
        mat->SetFriction(0.4f);
        mat->SetRollingFriction(((float)bi / 10) * 0.05f);

        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(mass);
        ball->SetInertiaXX(ChVector3d(inertia));

        // Initial position and velocity
        ball->SetPos(ChVector3d(-7, radius - 0.5, -5 + bi * radius * 2.5));
        ball->SetPosDt(ChVector3d(initial_linspeed, 0, 0));
        ball->SetAngVelParent(ChVector3d(0, 0, -initial_angspeed));

        // Contact geometry
        ball->EnableCollision(true);
        utils::AddSphereGeometry(ball.get(), mat, radius);

        // Add to the sys
        sys.Add(ball);
    }

    // Create some spheres that spin in place, with increasing spinning friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
        mat->SetFriction(0.4f);
        mat->SetSpinningFriction(((float)bi / 10) * 0.02f);

        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(mass);
        ball->SetInertiaXX(ChVector3d(inertia));

        // Initial position and velocity
        ball->SetPos(ChVector3d(-8, 1 + radius - 0.5, -5 + bi * radius * 2.5));
        ball->SetPosDt(ChVector3d(0, 0, 0));
        ball->SetAngVelParent(ChVector3d(0, 20, 0));

        // Contact geometry
        ball->EnableCollision(true);
        utils::AddSphereGeometry(ball.get(), mat, radius);

        // Add to the sys
        sys.Add(ball);
    }

    // Create the visualization window
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Friction test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(10, 10, 20), ChVector3d(0, 0, 0));
    vis.SetCameraVertical(CameraVerticalDir::Y);

    // Simulate sys
    while (vis.Run()) {
        sys.DoStepDynamics(time_step);
        vis.Render();
        ////std::cout << "num contacts: " << sys.GetNumContacts() << "\n\n";
    }

    return 0;
}