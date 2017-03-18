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

#include <fstream>
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
double time_step = 1e-3;
using namespace chrono;
using namespace chrono::collision;
#define USE_RIGID 1

#if USE_RIGID
Ch3DOFRigidContainer* mpm_container;
#else
ChFluidContainer* mpm_container;
#endif
// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
void AddBody(ChSystemParallelDVI* sys) {
    // IDs for the two bodies
    int binId = -200;
    int mixerId = -201;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurface>();
    mat->SetFriction(0.4f);

    // Create the containing bin (2 x 2 x 1)
    auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(binId);
    bin->SetMass(100);
    bin->SetPos(ChVector<>(0, 0, 1));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(false);

    ChVector<> hdim(.5, .5, 0.05);

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hdim.z()), ChVector<>(0, 0, -hdim.z()));
    // utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x(), hdim.y(), hdim.z()), ChVector<>(0, 0, hdim.z()*10));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    bin->GetCollisionModel()->BuildModel();
    bin->SetInertiaXX(utils::CalcBoxGyration(hdim).Get_Diag() * 100);

    sys->AddBody(bin);
}

void AddContainer(ChSystemParallelDVI* sys) {
    // IDs for the two bodies
    int binId = -200;
    int mixerId = -201;

    // Create a common material
    auto mat = std::make_shared<ChMaterialSurface>();
    mat->SetFriction(0.4f);

    // Create the containing bin (2 x 2 x 1)
    auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
    bin->SetMaterialSurface(mat);
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngAxis(-45, VECT_Y));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    ChVector<> hdim(.1, .1, .1);

    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hdim.z()), ChVector<>(0, 0, 0));
    // utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x(), hdim.y(), hdim.z()), ChVector<>(0, 0, hdim.z()*10));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);
    bin->GetCollisionModel()->BuildModel();

    sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the fluid in the shape of a sphere.
// -----------------------------------------------------------------------------
void AddFluid(ChSystemParallelDVI* sys) {
#if USE_RIGID
    mpm_container = new Ch3DOFRigidContainer(sys);
#else
    mpm_container = new ChFluidContainer(sys);

#endif
    real youngs_modulus = 1.4e6;
    real poissons_ratio = 0.2;
    real rho = 400;
#if USE_RIGID
    mpm_container->mu = 0;
    mpm_container->alpha = 0;
    mpm_container->cohesion = 0;

#else
    mpm_container->tau = time_step * 4;
    mpm_container->epsilon = 1e-3;
    mpm_container->rho = rho;
    mpm_container->viscosity = false;
    mpm_container->artificial_pressure = false;
#endif

    mpm_container->theta_c = 1;
    mpm_container->theta_s = 1;
    mpm_container->lame_lambda = youngs_modulus * poissons_ratio / ((1. + poissons_ratio) * (1. - 2. * poissons_ratio));
    mpm_container->lame_mu = youngs_modulus / (2. * (1. + poissons_ratio));
    mpm_container->youngs_modulus = youngs_modulus;
    mpm_container->nu = poissons_ratio;
    mpm_container->alpha_flip = .95;
    mpm_container->hardening_coefficient = 10.0;
    // mpm_container->rho = 400;
    mpm_container->mass = .01;
    mpm_container->mpm_iterations = 30;
    mpm_container->kernel_radius = .016 * 2;
    mpm_container->collision_envelope = mpm_container->kernel_radius * 0.05;
    mpm_container->contact_recovery_speed = 10;
    mpm_container->contact_cohesion = 0;
    mpm_container->contact_mu = 0;
    mpm_container->max_velocity = 10;
    mpm_container->compliance = 0;

    // mpm_container->tau = time_step * 4;
    // mpm_container->epsilon = 1e-3;
    real radius = .2;  //*5
    real dens = 30;
    real3 num_fluid = real3(10, 10, 10);
    real3 origin(0, 0, .1);
    real vol;

    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;

#if 1
#if USE_RIGID
    double dist = mpm_container->kernel_radius;

#else
    double dist = mpm_container->kernel_radius * 0.9;

#endif

    vol = dist * dist * dist * .8;
    mpm_container->mass = rho * vol;

    utils::HCPSampler<> sampler(dist);
    utils::Generator::PointVector points =
        sampler.SampleSphere(ChVector<>(0, 0, radius + radius * .5), radius);  // ChVector<>(radius, radius, radius));

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_fluid[i] = real3(0, 0, -5);
    }
    mpm_container->UpdatePosition(0);
    mpm_container->AddBodies(pos_fluid, vel_fluid);

    points = sampler.SampleBox(ChVector<>(1, 0, 0), ChVector<>(radius, radius, radius));

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_fluid[i] = real3(-6, 0, 0);
    }
// mpm_container->AddNodes(pos_fluid, vel_fluid);

#else
    pos_fluid.resize(1);
    vel_fluid.resize(1);

    pos_fluid[0] = real3(0, 0, 0);
    vel_fluid[0] = real3(0, 0, -1);
    mpm_container->AddNodes(pos_fluid, vel_fluid);

    pos_fluid[0] = real3(.05, 0, 0);
    vel_fluid[0] = real3(-1, 0, 0);
// mpm_container->AddNodes(pos_fluid, vel_fluid);
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
    msystem.GetSettings()->solver.tolerance = 0;
    msystem.GetSettings()->solver.alpha = 0;
    msystem.GetSettings()->solver.use_full_inertia_tensor = false;
    msystem.GetSettings()->solver.contact_recovery_speed = 100;
    msystem.GetSettings()->solver.cache_step_length = true;
    msystem.ChangeSolverType(SolverType::BB);
    msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

    AddFluid(&msystem);

    msystem.GetSettings()->collision.collision_envelope = (mpm_container->kernel_radius * .05);
    msystem.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);
    msystem.SetLoggingLevel(LoggingLevel::LOG_TRACE, true);
    msystem.SetLoggingLevel(LoggingLevel::LOG_INFO, true);
    // Create the fixed and moving bodies
    // ----------------------------------

    AddContainer(&msystem);
    AddBody(&msystem);
    // This initializes all of the MPM stuff
    msystem.Initialize();
// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "snowMPM", &msystem);
    gl_window.SetCamera(ChVector<>(0, -.4, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), .1f);
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
