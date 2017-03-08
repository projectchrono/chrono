
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Victor Bertolazzo
// =============================================================================
//
// ChronoParallel test program using DEM-C method for rolling frictional contact.
//
// The global reference frame has Z up.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
    // ----------------
    // Parameters
    // ----------------
    double radius = 0.5;

    double time_step = 1e-3;
    int num_threads = 1;

    uint max_iteration_normal = 0;
    uint max_iteration_sliding = 0;
    uint max_iteration_spinning = 100;
    uint max_iteration_bilateral = 0;
    
    // ------------------------
    // Create the parallel system
    // --------------------------

    ChSystemParallelDVI system;
    system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Set number of threads
    system.SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);
////#pragma omp parallel
////#pragma omp master
////    {
////        // Sanity check: print number of threads in a parallel region
////        std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl;
////    }

    // Set solver settings
    system.ChangeSolverType(SolverType::APGD);

    system.GetSettings()->perform_thread_tuning = false;

    system.GetSettings()->solver.solver_mode = SolverMode::SPINNING;
    system.GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    system.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    system.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    system.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
    system.GetSettings()->solver.alpha = 0;
    system.GetSettings()->solver.contact_recovery_speed = 0.1;
    system.GetSettings()->solver.use_full_inertia_tensor = false;
    system.GetSettings()->solver.tolerance = 0.1;

    system.GetSettings()->collision.collision_envelope = 0.05 * radius;
    system.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
    system.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

    // Create the container body.
    // Note that Chrono::Parallel uses the *minimum* of the spinning and rolling friction values for a contacting pair.
    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->GetMaterialSurface()->SetFriction(0.4f);
    container->GetMaterialSurface()->SetRollingFriction(1);
    container->GetMaterialSurface()->SetSpinningFriction(1);

    container->SetCollide(true);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), ChVector<>(10, 10, 1), ChVector<>(0, 0, 0));
    utils::AddBoxGeometry(container.get(), ChVector<>(1, 10, 2), ChVector<>(10, 0, 0)); 
    utils::AddBoxGeometry(container.get(), ChVector<>(1, 10, 2), ChVector<>(-10, 0, 0));
    utils::AddBoxGeometry(container.get(), ChVector<>(10, 1, 2), ChVector<>(0, -10, 0));
    utils::AddBoxGeometry(container.get(), ChVector<>(10, 1, 2), ChVector<>(0, 10, 0));
    container->GetCollisionModel()->BuildModel();

    // Create some spheres that roll horizontally, with increasing rolling friction values
    double density = 1000;
    double mass = density * (4.0 / 3.0) * CH_C_PI * pow(radius, 3);
    double inertia = (2.0 / 5.0) * mass * pow(radius, 2);
    double initial_angspeed = 10;
    double initial_linspeed = initial_angspeed * radius;

    for (int bi = 0; bi < 10; bi++) {
        auto ball = std::shared_ptr<ChBody>(system.NewBody());
        ball->SetIdentifier(bi);
        ball->SetMass(mass);
        ball->SetInertiaXX(ChVector<>(inertia));

        // Initial position and velocity
        ball->SetPos(ChVector<>(-6, -5 + bi * radius * 2.5, 1 + radius));
        ball->SetPos_dt(ChVector<>(initial_linspeed, 0, 0));
        ball->SetWvel_par(ChVector<>(0, initial_angspeed, 0));

        // Contact geometry
        ball->SetCollide(true);
        ball->GetCollisionModel()->ClearModel();
        utils::AddSphereGeometry(ball.get(), radius);
        ball->GetCollisionModel()->BuildModel();

        // Sliding and rolling friction coefficients
        ball->GetMaterialSurface()->SetFriction(0.4f);
        ball->GetMaterialSurface()->SetRollingFriction((bi / 10.0f) * 0.05f);

        // Add to the system
        system.Add(ball);
    }

    // Create some spheres that spin in place, with increasing spinning friction values
	for (int bi = 0; bi < 10; bi++) {
		auto ball = std::shared_ptr<ChBody>(system.NewBody());
		ball->SetIdentifier(bi);
		ball->SetMass(mass);
		ball->SetInertiaXX(ChVector<>(inertia));

		// Initial position and velocity
		ball->SetPos(ChVector<>(-8, -5 + bi * radius * 2.5, 1 + radius));
		ball->SetPos_dt(ChVector<>(0, 0, 0));
		ball->SetWvel_par(ChVector<>(0, 0, 20));

		// Contact geometry
		ball->SetCollide(true);
		ball->GetCollisionModel()->ClearModel();
		utils::AddSphereGeometry(ball.get(), radius);
		ball->GetCollisionModel()->BuildModel();

		// Sliding and rolling friction coefficients
		ball->GetMaterialSurface()->SetFriction(0.4f);
        ball->GetMaterialSurface()->SetSpinningFriction((bi / 10.0f) * 0.02f);

        // Add to the system
		system.Add(ball);
	}


#ifdef CHRONO_OPENGL
    // -------------------------------
    // Create the visualization window
    // -------------------------------

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Settling test", &system);
    gl_window.SetCamera(ChVector<>(10, 10, 20), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

    // ---------------
    // Simulate system
    // ---------------

    double time_end = 20.0;
    double time_out = 2.5;
    bool output = false;

    while (system.GetChTime() < time_end) {
        system.DoStepDynamics(time_step);

        if (!output && system.GetChTime() >= time_out) {
            for (int i = 1; i <= 10; i++) {
                auto pos = system.Get_bodylist()->at(i)->GetPos();
                std::cout << pos.x() << std::endl;
            }
            output = true;
        }

#ifdef CHRONO_OPENGL
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        if (gl_window.Active()) {
            gl_window.Render();
        } else {
            return 1;
        }
#endif
    }

    return 0;
}