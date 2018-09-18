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
// Authors: Radu Serban
// =============================================================================
//
// Test terrain contact parameters for RoboSimian simulations
//
// =============================================================================

#include <cmath>
#include <cstdio>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "granular.h"

using namespace chrono;
using namespace robosimian;

using std::cout;
using std::endl;

// =============================================================================

// Simulation settings
ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;
////double step_size = 1e-4;  // for SMC
double step_size = 1e-2;  // for NSC
double duration_settling = 0.2;
double duration_simulation = 0.3;
int nthreads = 2;
bool render = true;

// Granular material properties
double radius = 0.0075;
double density = 2000;
double cohesion = 20e3;
double friction = 0.4f;

// Terrain patch
double patch_length = 0.5;
double patch_width = 0.5;
unsigned int num_layers = 15;

// =============================================================================

int main(int argc, char* argv[]) {
    // -------------
    // Create system
    // -------------

    ChSystemParallel* sys;
    switch (contact_method) {
        case ChMaterialSurface::NSC: {
            auto my_sys = new ChSystemParallelNSC;
            cout << "System type: NSC" << endl;

            my_sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
            my_sys->GetSettings()->solver.max_iteration_normal = 0;
            my_sys->GetSettings()->solver.max_iteration_sliding = 100;
            my_sys->GetSettings()->solver.max_iteration_spinning = 0;
            my_sys->GetSettings()->solver.compute_N = false;
            my_sys->GetSettings()->solver.alpha = 0;
            my_sys->GetSettings()->solver.contact_recovery_speed = 1000;
            my_sys->GetSettings()->collision.collision_envelope = 0.01;

            my_sys->ChangeSolverType(SolverType::APGD);

            sys = my_sys;
            break;
        }
        case ChMaterialSurface::SMC: {
            auto my_sys = new ChSystemParallelSMC;
            cout << "System type: SMC" << endl;

            my_sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
            my_sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::OneStep;

            sys = my_sys;
            break;
        }
    }

    sys->Set_G_acc(ChVector<double>(0, 0, -9.8));

    int max_threads = CHOMPfunctions::GetNumProcs();
    if (nthreads > max_threads)
        nthreads = max_threads;
    sys->SetParallelThreadNumber(nthreads);
    CHOMPfunctions::SetNumThreads(nthreads);

    sys->GetSettings()->solver.tolerance = 1e-3;
    sys->GetSettings()->solver.max_iteration_bilateral = 100;
    sys->GetSettings()->solver.cache_step_length = true;
    sys->GetSettings()->solver.use_full_inertia_tensor = false;
    sys->GetSettings()->solver.bilateral_clamp_speed = 1e8;
    sys->GetSettings()->min_threads = nthreads;

    sys->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    // -----------------
    // Initialize OpenGL
    // -----------------

    if (render) {
        opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
        gl_window.Initialize(1280, 720, "RoboSimian terrain test", sys);
        gl_window.SetCamera(ChVector<>(0.4, -0.4, -0.1), ChVector<>(0, 0, -0.1), ChVector<>(0, 0, 1), 0.05f);
        gl_window.SetRenderMode(opengl::SOLID);
    }

    // -----------------
    // Create wheel body
    // -----------------

    double mass = 30;
    ChVector<> inertia_xx(0.006378, 0.009155, 0.006377);
    ChVector<> location(0, 0, 0);

    auto wheel = std::shared_ptr<ChBody>(sys->NewBodyAuxRef());
    wheel->SetIdentifier(0);
    wheel->SetMass(mass);
    wheel->SetInertiaXX(inertia_xx);
    wheel->SetPos(location);
    wheel->SetBodyFixed(true);
    wheel->SetCollide(true);

    double cyl_length = 0.123;
    double cyl_radius = 0.12;
    auto cyl_shape = std::make_shared<ChCylinderShape>();
    cyl_shape->GetCylinderGeometry().rad = cyl_radius;
    cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl_length / 2, 0);
    cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl_length / 2, 0);
    wheel->AddAsset(cyl_shape);

    wheel->GetCollisionModel()->ClearModel();
    wheel->GetCollisionModel()->AddCylinder(cyl_radius, cyl_radius, cyl_length / 2);
    wheel->GetCollisionModel()->BuildModel();

    sys->AddBody(wheel);

    // --------------
    // Create terrain
    // --------------

    GroundGranularA ground(sys);
    ////GroundGranularB ground(sys);

    ground.SetParticleProperties(radius, density, friction, cohesion);
    ground.SetPatchProperties(patch_length, patch_width, num_layers);

    ground.Initialize(location.x() - patch_length / 2, location.z() - 0.15, step_size);
    auto terrain_bottom = ground.GetBottomHeight();
    auto terrain_init_top = ground.GetTopHeight();
    auto terrain_settled_top = terrain_init_top;

    cout << "Generated " << ground.GetNumParticles() << " particles" << endl;
    cout << "Bottom: " << terrain_bottom << endl;
    cout << "Top:    " << terrain_init_top.first << "  " << terrain_init_top.second << endl;

    // ---------------------------------
    // Run simulation for specified time
    // ---------------------------------

    // Timed events
    double time_release = duration_settling;
    double time_end = time_release + duration_simulation;

    int sim_frame = 0;
    bool wheel_released = false;

    while (true) {
        double time = sys->GetChTime();

        if (time >= time_end) {
            cout << "Reached final time: " << time << endl;
            break;
        }

        if (!wheel_released && time > time_release) {
            terrain_settled_top = ground.GetTopHeight();
            wheel->SetPos(wheel->GetPos() - ChVector<>(0, 0, terrain_init_top.first - terrain_settled_top.first));
            wheel->SetBodyFixed(false);
            wheel_released = true;
            cout << "Time: " << time << "  RELEASE WHEEL" << endl;
            cout << "Terrain height: " << terrain_settled_top.first << "  " << terrain_settled_top.second << endl;
        }

        sys->DoStepDynamics(step_size);

        if (render) {
            opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
            if (gl_window.Active()) {
                gl_window.Render();
            } else {
                break;
            }
        }

        sim_frame++;
    }

    cout << "Wheel center height: " << wheel->GetPos().z() << endl;
    cout << "Wheel bottom point:  " << wheel->GetPos().z() - cyl_radius << endl;
    cout << "Wheel sinkage:       " << terrain_settled_top.second - (wheel->GetPos().z() - cyl_radius) << endl;

    return 0;
}
