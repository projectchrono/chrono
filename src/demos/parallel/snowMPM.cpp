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

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelMPM* sys) {
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
void AddFluid(ChSystemParallelMPM* sys) {
    ChFluidContainer* fluid_container = new ChFluidContainer(sys);

    real radius = sys->GetSettings()->fluid.kernel_radius * 5;  //*5
    real dens = 30;
    real3 num_fluid = real3(10, 10, 10);
    real3 origin(0, 0, .2);
    real vol;

    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;
#if 0
    double dist = sys->GetSettings()->fluid.kernel_radius * .75;
    utils::HCPSampler<> sampler(dist);
    utils::Generator::PointVector points = sampler.SampleSphere(ChVector<>(0, 0, 0), radius);

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x, points[i].y, points[i].z) + origin;
        vel_fluid[i] = real3(0, 0, 0);
    }
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
            pos_fluid.push_back(p);
            vel_fluid.push_back(v);
        }
//    pos_fluid.push_back(real3(.5, .5, .5));
//    vel_fluid.push_back(real3(0, -3, 0));
//
//    pos_fluid.push_back(real3(.5, .8, .5));
//    vel_fluid.push_back(real3(0, -2, 0));


#endif
    fluid_container->UpdatePosition(0);
    fluid_container->AddFluid(pos_fluid, vel_fluid);
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

    ChSystemParallelMPM msystem;
    // omp_set_num_threads(4);
    // Set number of threads.
    //    int max_threads = 2;//CHOMPfunctions::GetNumProcs();
    //    if (threads > max_threads)
    //        threads = max_threads;
    //    msystem.SetParallelThreadNumber(threads);
    //    CHOMPfunctions::SetNumThreads(threads);

    // Set gravitational acceleration
    msystem.Set_G_acc(ChVector<>(0, -gravity, 0));

    real youngs_modulus = (real)1.4e5;
    real poissons_ratio = (real).2;

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
    msystem.GetSettings()->solver.contact_recovery_speed = 100;
    msystem.GetSettings()->solver.cache_step_length = true;

    msystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
    msystem.GetSettings()->fluid.kernel_radius = .005;
    msystem.GetSettings()->fluid.mass = .007 * 5.5;
    msystem.GetSettings()->fluid.density = 1000;
    msystem.GetSettings()->mpm.theta_c = (real)2.5e-2;
    msystem.GetSettings()->mpm.theta_s = (real)7.5e-3;
    msystem.GetSettings()->mpm.lambda =
        youngs_modulus * poissons_ratio / (((real)1. + poissons_ratio) * ((real)1. - (real)2. * poissons_ratio));
    msystem.GetSettings()->mpm.mu = youngs_modulus / ((real)2. * ((real)1. + poissons_ratio));
    msystem.GetSettings()->mpm.alpha = (real).95;
    msystem.GetSettings()->mpm.hardening_coefficient = (real)10.;

    real initial_density = (real)4e2;
    msystem.GetSettings()->mpm.mass = 400;

    msystem.GetSettings()->collision.collision_envelope = (msystem.GetSettings()->fluid.kernel_radius * .05);
    msystem.GetSettings()->collision.bins_per_axis = int3(2, 2, 2);
    // msystem.SetLoggingLevel(LOG_TRACE, true);
    // msystem.SetLoggingLevel(LOG_INFO, true);
    // Create the fixed and moving bodies
    // ----------------------------------

    AddContainer(&msystem);
    AddFluid(&msystem);
    // This initializes all of the MPM stuff
    msystem.Initialize();
// Perform the simulation
// ----------------------
//#undef CHRONO_OPENGL
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "snowMPM", &msystem);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
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
