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
// Authors: Hammad Mazhar
// =============================================================================
//
// Chrono::Multicore test program using a fluid container.
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
double kernel_radius = .016 * 2;

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

    utils::CreateBoxContainer(sys, 0, mat, hdim, 0.05, Vector(0, 0, .3), Q_from_AngAxis(-10, VECT_Y), true, false, true,
                              true);
}

// -----------------------------------------------------------------------------
// Create the fluid in the shape of a sphere.
// -----------------------------------------------------------------------------
void AddFluid(ChSystemMulticoreNSC* sys) {
    auto fluid_container = chrono_types::make_shared<ChFluidContainer>();
    sys->Add3DOFContainer(fluid_container);

    fluid_container->tau = time_step * 4;
    fluid_container->contact_cohesion = 0;
    fluid_container->epsilon = 1e-3;
    fluid_container->kernel_radius = kernel_radius;
    fluid_container->mass = .007 * 5.5;
    fluid_container->viscosity = .01;
    fluid_container->enable_viscosity = false;

    fluid_container->contact_mu = 0;
    fluid_container->rho = 1000;

    // sys.GetSettings()->fluid.max_interactions = 30;
    fluid_container->artificial_pressure = true;
    fluid_container->artificial_pressure_k = .01;
    fluid_container->artificial_pressure_dq = .2 * kernel_radius;
    fluid_container->artificial_pressure_n = 4;
    fluid_container->collision_envelope = 0;  // fluid_container->kernel_ra dius * .05;

    real radius = .2;  //*5
    real3 origin(0, 0, -.2);
    real vol;

    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;

    double dist = kernel_radius * .9;
    utils::HCPSampler<> sampler(dist);
    vol = dist * dist * dist * .8;
    utils::Generator::PointVector points = sampler.SampleSphere(ChVector<>(0, 0, 0), radius);

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

    AddFluid(&sys);

    sys.GetSettings()->collision.collision_envelope = (kernel_radius * .05);
    sys.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);

    // Create the fixed and moving bodies
    AddContainer(&sys);

    // Perform the simulation
#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Fluid NSC");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, -2.5, 0), ChVector<>(0, 0, 0));
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
