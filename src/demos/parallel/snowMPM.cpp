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
// Authors: Radu Serban, Hammad Mazhar
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

#include <stdio.h>
#include <vector>
#include <cmath>

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include <fstream>
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;
ChMPMContainer* mpm_container;
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
    ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
    mat->SetFriction(0.4f);

    // Create the containing bin (2 x 2 x 1)
    ChSharedBodyPtr bin(new ChBody(new ChCollisionModelParallel));
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(.5, .5, 0.05);

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hdim.y, hdim.z), ChVector<>(0, 0, -hdim.z));
    // utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hdim.y, hdim.z), ChVector<>(0, 0, hdim.z*10));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the fluid in the shape of a sphere.
// -----------------------------------------------------------------------------
void AddFluid(ChSystemParallelDVI* sys) {
    mpm_container = new ChMPMContainer(sys);

    real youngs_modulus = 1.4e8;
    real poissons_ratio = 0.2;

    mpm_container->theta_c = 2.5e-2;
    mpm_container->theta_s = 7.5e-3;
    mpm_container->lambda = youngs_modulus * poissons_ratio / ((1. + poissons_ratio) * (1. - 2. * poissons_ratio));
    mpm_container->mu = youngs_modulus / (2. * (1. + poissons_ratio));
    mpm_container->alpha = .95;
    mpm_container->hardening_coefficient = 10.0;

    real initial_density = 1000;
    mpm_container->mass = .004;
    mpm_container->max_iterations = 20;
    mpm_container->kernel_radius = .005;
    mpm_container->contact_recovery_speed = 10000;

    real radius = mpm_container->kernel_radius * 10;  //*5
    real dens = 30;
    real3 num_fluid = real3(10, 10, 10);
    real3 origin(0, 0, .2);
    real vol;

    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;
#if 1
    double dist = mpm_container->kernel_radius * .9;
    utils::HCPSampler<> sampler(dist);
    utils::Generator::PointVector points = sampler.SampleSphere(ChVector<>(-.1, 0, 0), radius);

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x, points[i].y, points[i].z) + origin;
        vel_fluid[i] = real3(-3, 0, 0);
    }
    mpm_container->UpdatePosition(0);
    mpm_container->AddNodes(pos_fluid, vel_fluid);

    points = sampler.SampleSphere(ChVector<>(.1, 0, 0), radius);

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x, points[i].y, points[i].z) + origin;
        vel_fluid[i] = real3(-3, 0, 0);
    }
    mpm_container->AddNodes(pos_fluid, vel_fluid);

#else
    std::ifstream ifile("snowMPMinit.dat");
    while (ifile.fail() == false) {
        real m;
        real3 p, v;
        ifile >> m;
        if (ifile.fail() == false) {
            ifile >> p.x >> p.y >> p.z;
            ifile >> v.x >> v.y >> v.z;
        }
        if (ifile.fail() == false) {
            pos_fluid.push_back(p);
            vel_fluid.push_back(v);
        }
    }
    //    pos_fluid.push_back(real3(.5, .5, .5));
    //    vel_fluid.push_back(real3(0, -3, 0));
    //
    //    pos_fluid.push_back(real3(.5, .8, .5));
    //    vel_fluid.push_back(real3(0, -2, 0));
    fluid_container->UpdatePosition(0);
    fluid_container->AddFluid(pos_fluid, vel_fluid);
#endif
}
// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    int threads = 8;

    // Simulation parameters
    // ---------------------

    double gravity = 9.81;
    double time_step = 1e-3;
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
    msystem.GetSettings()->solver.solver_mode = SLIDING;
    msystem.GetSettings()->solver.max_iteration_normal = 0;
    msystem.GetSettings()->solver.max_iteration_sliding = 40;
    msystem.GetSettings()->solver.max_iteration_spinning = 0;
    msystem.GetSettings()->solver.max_iteration_bilateral = 0;
    msystem.GetSettings()->solver.tolerance = tolerance;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.use_full_inertia_tensor = false;
    msystem.GetSettings()->collision.use_two_level = false;
    msystem.GetSettings()->solver.contact_recovery_speed = 10000;
    msystem.GetSettings()->solver.cache_step_length = true;
    msystem.ChangeSolverType(BB);
    msystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

    AddFluid(&msystem);

    msystem.GetSettings()->collision.collision_envelope = (mpm_container->kernel_radius * .05);
    msystem.GetSettings()->collision.bins_per_axis = int3(2, 2, 2);
    msystem.SetLoggingLevel(LOG_TRACE, true);
    msystem.SetLoggingLevel(LOG_INFO, true);
    // Create the fixed and moving bodies
    // ----------------------------------

    AddContainer(&msystem);

    // This initializes all of the MPM stuff
    msystem.Initialize();
// Perform the simulation
// ----------------------
//#undef CHRONO_OPENGL
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "snowMPM", &msystem);
    gl_window.SetCamera(ChVector<>(0, -.4, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), .1);
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
    int num_steps = std::ceil(time_end / time_step);

    double time = 0;
    for (int i = 0; i < num_steps; i++) {
        msystem.DoStepDynamics(time_step);
        time += time_step;
    }
#endif

    return 0;
}
