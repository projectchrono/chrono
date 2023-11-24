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
// Chrono::Multicore test program using an MPM container (3DOF), either a particle
// or a fluid container
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
#include <fstream>

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

#define USE_RIGID 1

double time_step = 1e-3;
double kernel_radius = .016 * 2;

void AddBody(ChSystemMulticoreNSC* sys) {
    int binId = -200;

    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);

    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(Q_from_AngAxis(-45, VECT_Y));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    utils::AddBoxGeometry(bin.get(), mat, ChVector<>(0.2, 0.2, 0.2), ChVector<>(0, 0, 0));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(2);

    sys->AddBody(bin);
}

// Create the MPM container with spherical contact
void AddMPMContainer(ChSystemMulticoreNSC* sys) {
#if USE_RIGID
    auto mpm_container = chrono_types::make_shared<ChParticleContainer>();
#else
    auto mpm_container = chrono_types::make_shared<ChFluidContainer>();
#endif
    sys->Add3DOFContainer(mpm_container);

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
    mpm_container->kernel_radius = kernel_radius;
    mpm_container->collision_envelope = kernel_radius * 0.05;
    mpm_container->contact_recovery_speed = 10;
    mpm_container->contact_cohesion = 0;
    mpm_container->contact_mu = 0;
    mpm_container->max_velocity = 10;
    mpm_container->compliance = 0;

    // mpm_container->tau = time_step * 4;
    // mpm_container->epsilon = 1e-3;
    real radius = .2;  //*5
    real3 origin(0, 0, .1);

    std::vector<real3> pos_fluid;
    std::vector<real3> vel_fluid;

#if USE_RIGID
    double dist = kernel_radius;
#else
    double dist = kernel_radius * 0.9;
#endif

    real vol = dist * dist * dist * .8;
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
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create system
    ChSystemMulticoreNSC sys;

    // Set associated collision detection system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set number of threads
    sys.SetNumThreads(8);

    // Set gravitational acceleration
    double gravity = 9.81;
    sys.Set_G_acc(ChVector<>(0, 0, -gravity));

    // Set solver parameters
    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_normal = 0;
    sys.GetSettings()->solver.max_iteration_sliding = 40;
    sys.GetSettings()->solver.max_iteration_spinning = 0;
    sys.GetSettings()->solver.max_iteration_bilateral = 0;
    sys.GetSettings()->solver.tolerance = 0;
    sys.GetSettings()->solver.alpha = 0;
    sys.GetSettings()->solver.use_full_inertia_tensor = false;
    sys.GetSettings()->solver.contact_recovery_speed = 100;
    sys.GetSettings()->solver.cache_step_length = true;
    sys.ChangeSolverType(SolverType::BB);
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;

    AddMPMContainer(&sys);

    sys.GetSettings()->collision.collision_envelope = kernel_radius * .05;
    sys.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);

    // Create the fixed body
    AddBody(&sys);

    // This initializes all of the MPM stuff
    sys.Initialize();

    // Perform the simulation
#ifdef CHRONO_OPENGL
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(&sys);
    vis.SetWindowTitle("Snow MPM");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector<>(0, -.4, 0), ChVector<>(0, 0, 0));
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
