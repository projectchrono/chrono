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
// Chrono::Multicore test program using a container of uniform particles (3DOF).
// Uses NSC (complementarity-based) method for frictional contact.
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
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_multicore/physics/Ch3DOFContainer.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

double time_step = 1e-3;
real diameter = 0.016;

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemMulticoreNSC* sys) {
    // Create a common material
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    ChVector<> hdim(.55, .6, .55);

    utils::CreateBoxContainer(sys, 0, mat, hdim, 0.05, Vector(0, 0, 0), Q_from_AngAxis(-10, VECT_Y), true, false, true,
                              true);
}

// -----------------------------------------------------------------------------
// Create the 3DOF particles with spherical contact
// -----------------------------------------------------------------------------
void AddParticles(ChSystemMulticoreNSC* sys) {
    auto particle_container = chrono_types::make_shared<ChParticleContainer>();
    sys->Add3DOFContainer(particle_container);

    particle_container->contact_cohesion = 0;
    particle_container->kernel_radius = diameter;
    particle_container->mass = .1;
    particle_container->contact_mu = 0;
    particle_container->mu = 0;
    particle_container->alpha = .1;
    particle_container->compliance = 1e-6;
    particle_container->cohesion = 0;
    particle_container->contact_recovery_speed = .3;
    particle_container->collision_envelope = diameter * .01;

    real radius = .1;
    real3 origin(0, 0, -.2);

    std::vector<real3> pos_particles;
    std::vector<real3> vel_particles;

    utils::GridSampler<> sampler(diameter);
    utils::Generator::PointVector points = sampler.SampleSphere(ChVector<>(0, 0, 0), radius);

    pos_particles.resize(points.size());
    vel_particles.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_particles[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_particles[i] = real3(0, 0, 0);
    }

    particle_container->UpdatePosition(0);
    particle_container->AddBodies(pos_particles, vel_particles);
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
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
    msystem.GetSettings()->solver.max_iteration_normal = 0;
    msystem.GetSettings()->solver.max_iteration_sliding = 40;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = 0;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.use_full_inertia_tensor = false;
    msystem.GetSettings()->solver.contact_recovery_speed = 100000;
    msystem.GetSettings()->solver.cache_step_length = true;

    msystem.ChangeSolverType(SolverType::BB);
    msystem.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    AddParticles(&msystem);

    msystem.GetSettings()->collision.collision_envelope = (diameter * .05);
    msystem.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);
    msystem.SetLoggingLevel(LoggingLevel::LOG_TRACE, true);
    msystem.SetLoggingLevel(LoggingLevel::LOG_INFO, true);

    AddContainer(&msystem);

// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.AttachSystem(&msystem);
    gl_window.Initialize(1280, 720, "Particles NSC");
    gl_window.SetCamera(ChVector<>(0, -2, -1), ChVector<>(0, 0, -1), ChVector<>(0, 0, 1), .2f);
    gl_window.SetRenderMode(opengl::WIREFRAME);
    gl_window.Pause();

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

    while (true) {
        if (gl_window.Active()) {
            gl_window.DoStepDynamics(time_step);
            gl_window.Render();
        } else {
            break;
        }
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
