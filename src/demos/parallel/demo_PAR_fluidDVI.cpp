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
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel test program using penalty method for frictional contact.
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

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_parallel/physics/Ch3DOFContainer.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;
double time_step = 1e-3;
ChFluidContainer* fluid_container;
// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelDVI* sys) {
    // IDs for the two bodies
    int binId = -200;
    int mixerId = -201;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurface>();
    mat->SetFriction(0.4f);

    ChVector<> hdim(.55, .6, .55);

    utils::CreateBoxContainer(sys, 0, mat, hdim, 0.05, Vector(0, 0, .3), Q_from_AngAxis(-10, VECT_Y), true, false, true,
                              true);
}

// -----------------------------------------------------------------------------
// Create the fluid in the shape of a sphere.
// -----------------------------------------------------------------------------
void AddFluid(ChSystemParallelDVI* sys) {
    fluid_container = new ChFluidContainer(sys);

    fluid_container->tau = time_step * 4;
    fluid_container->contact_cohesion = 0;
    fluid_container->epsilon = 1e-3;
    fluid_container->kernel_radius = .016 * 2;
    fluid_container->mass = .007 * 5.5;
    fluid_container->viscosity = .01;
    fluid_container->enable_viscosity = false;

    fluid_container->contact_mu = 0;
    fluid_container->rho = 1000;

    // msystem.GetSettings()->fluid.max_interactions = 30;
    fluid_container->artificial_pressure = true;
    fluid_container->artificial_pressure_k = .01;
    fluid_container->artificial_pressure_dq = .2 * fluid_container->kernel_radius;
    fluid_container->artificial_pressure_n = 4;
    fluid_container->collision_envelope = 0;  // fluid_container->kernel_ra dius * .05;

    real radius = .2;  //*5
    real dens = 30;
    real3 num_fluid = real3(10, 10, 10);
    real3 origin(0, 0, -.2);
    real vol;

    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;

    double dist = fluid_container->kernel_radius * .9;
    utils::HCPSampler<> sampler(dist);
    vol = dist * dist * dist * .8;
#if 1
    utils::Generator::PointVector points = sampler.SampleSphere(ChVector<>(0, 0, 0), radius);
// vol = 4.0 / 3.0 * CH_C_PI * pow(radius, 3) / real(points.size());

#else
    ChVector<> hdim(.5, .5, .5);
    utils::Generator::PointVector points = sampler.SampleBox(ChVector<>(0, 0, -hdim.z()), hdim);
// vol = hdim.x() * hdim.y() * hdim.z() / real(points.size());
#endif

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_fluid[i] = real3(0, 0, 0);
    }

    fluid_container->mass = fluid_container->rho * vol;
    std::cout << "fluid_mass: " << fluid_container->mass << std::endl;
    fluid_container->UpdatePosition(0);
    fluid_container->AddBodies(pos_fluid, vel_fluid);
}
// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    int threads = 8;

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;

    double time_end = 1;

    double out_fps = 50;

    uint max_iteration = 30;
    real tolerance = 1e-3;

    // Create system
    // -------------

    ChSystemParallelDVI msystem;
    // omp_set_num_threads(4);
    // Set number of threads.
    //    int max_threads = 2;//CHOMPfunctions::GetNumProcs();
    //    if (threads > max_threads)
    //        threads = max_threads;
    //    msystem.SetParallelThreadNumber(threads);
    //    CHOMPfunctions::SetNumThreads(threads);

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
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    AddFluid(&msystem);

    msystem.GetSettings()->collision.collision_envelope = (fluid_container->kernel_radius * .05);
    msystem.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);
    msystem.SetLoggingLevel(LoggingLevel::LOG_TRACE, true);
    msystem.SetLoggingLevel(LoggingLevel::LOG_INFO, true);
    // Create the fixed and moving bodies
    // ----------------------------------
    AddContainer(&msystem);

// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "fluidDVI", &msystem);
    gl_window.SetCamera(ChVector<>(0, -2, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), .2f);
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
    int num_steps = (int)std::ceil(time_end / time_step);

    double time = 0;
    for (int i = 0; i < num_steps; i++) {
        msystem.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
