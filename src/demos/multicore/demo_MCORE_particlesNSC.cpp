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
#include "chrono_opengl/ChVisualSystemOpenGL.h"
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

    utils::CreateBoxContainer(sys, 0, mat,                      //
                              ChVector<>(1.1, 1.2, 1.1), 0.05,  //
                              VNULL, Q_from_AngY(-10),          //
                              true, true, true);
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

    ChSystemMulticoreNSC sys;

    // Set number of threads
    sys.SetNumThreads(8);

    // Set gravitational acceleration
    sys.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = 0;
    sys.GetSettings()->solver.max_iteration_sliding = 40;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = 0;
    sys.GetSettings()->solver.tolerance = tolerance;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.use_full_inertia_tensor = false;
    sys.GetSettings()->solver.contact_recovery_speed = 100000;
    sys.GetSettings()->solver.cache_step_length = true;

    sys.ChangeSolverType(SolverType::BB);
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    AddParticles(&sys);

    sys.GetSettings()->collision.collision_envelope = (diameter * .05);
    sys.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);

    AddContainer(&sys);

    // Perform the simulation
#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Particles NSC");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, -2, -1), ChVector<>(0, 0, -1));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    // Uncomment the following two lines for the OpenGL manager to automatically
    // run the simulation in an infinite loop.
    // gl_window.StartDrawLoop(time_step);
    // return 0;

    while (vis.Run()) {
        sys.DoStepDynamics(time_step);
        vis.Render();
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
