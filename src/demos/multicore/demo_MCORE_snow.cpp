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
// Chrono::Multicore test program using a 3DOF container (particle or fluid).
//
// The global reference frame has Z up.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>
#include <fstream>

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_multicore/physics/Ch3DOFContainer.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

#define USE_RIGID 1

double time_step = 1e-3;
double kernel_radius = .016 * 2;

void AddBody(ChSystemMulticoreNSC* sys) {
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetMass(1);
    bin->SetPos(ChVector3d(0, 0, 0));
    bin->SetRot(QuatFromAngleY(-45 * CH_DEG_TO_RAD));
    bin->EnableCollision(true);
    bin->SetFixed(true);

    utils::AddBoxGeometry(bin.get(), mat, ChVector3d(0.2, 0.2, 0.2), ChVector3d(0, 0, 0));
    bin->GetCollisionModel()->SetFamily(1);
    bin->GetCollisionModel()->DisallowCollisionsWith(2);

    sys->AddBody(bin);
}

// Create the 3DOF container with spherical contact
void AddContainer(ChSystemMulticoreNSC* sys) {
#if USE_RIGID
    auto container = chrono_types::make_shared<ChParticleContainer>();
#else
    auto container = chrono_types::make_shared<ChFluidContainer>();
#endif
    sys->Add3DOFContainer(container);

    real youngs_modulus = 1.4e6;
    real poissons_ratio = 0.2;
    real rho = 400;

#if USE_RIGID
    container->mu = 0;
    container->alpha = 0;
    container->cohesion = 0.1;
#else
    container->tau = time_step * 4;
    container->epsilon = 1e-3;
    container->rho = rho;
    container->viscosity = false;
    container->artificial_pressure = false;
#endif

    container->theta_c = 1;
    container->theta_s = 1;
    container->lame_lambda = youngs_modulus * poissons_ratio / ((1. + poissons_ratio) * (1. - 2. * poissons_ratio));
    container->lame_mu = youngs_modulus / (2. * (1. + poissons_ratio));
    container->youngs_modulus = youngs_modulus;
    container->nu = poissons_ratio;
    container->alpha_flip = .95;
    container->hardening_coefficient = 10.0;
    ////container->rho = 400;
    container->mass = .01;
    container->kernel_radius = kernel_radius;
    container->collision_envelope = kernel_radius * 0.05;
    container->contact_recovery_speed = 10;
    container->contact_cohesion = 0;
    container->contact_mu = 0;
    container->max_velocity = 10;
    container->compliance = 0;

    ////container->tau = time_step * 4;
    ////container->epsilon = 1e-3;
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
    container->mass = rho * vol;

    utils::ChHCPSampler<> sampler(dist);
    utils::ChGenerator::PointVector points =
        sampler.SampleSphere(ChVector3d(0, 0, radius + radius * .5), radius);  // ChVector3d(radius, radius, radius));

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_fluid[i] = real3(0, 0, -5);
    }
    container->UpdatePosition(0);
    container->AddBodies(pos_fluid, vel_fluid);

    points = sampler.SampleBox(ChVector3d(1, 0, 0), ChVector3d(radius, radius, radius));

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
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create system
    ChSystemMulticoreNSC sys;

    // Set associated collision detection system
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    // Set number of threads
    sys.SetNumThreads(8);

    // Set gravitational acceleration
    double gravity = 9.81;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));

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

    AddContainer(&sys);

    sys.GetSettings()->collision.collision_envelope = kernel_radius * .05;
    sys.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);

    // Create the fixed body
    AddBody(&sys);

    // Initialize system
    sys.Initialize();

    // Perform the simulation
#ifdef CHRONO_VSG
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Snow");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0.1, -2, 0.0), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyBox();
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(-CH_PI_2, CH_PI_4);
    vis->Initialize();

    while (vis->Run()) {
        sys.DoStepDynamics(time_step);
        vis->Render();
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
