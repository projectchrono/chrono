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
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;

#define USE_RIGID 1

double time_step = 1e-3;
double kernel_radius = .016 * 2;

std::shared_ptr<ChBody> AddBody(ChSystemMulticoreNSC* sys) {
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

    return bin;
}

std::shared_ptr<ChParticleContainer> AddSnowball(ChSystemMulticoreNSC* sys) {
#if USE_RIGID
    auto snow_ball = chrono_types::make_shared<ChParticleContainer>();
#else
    auto snow_ball = chrono_types::make_shared<ChFluidContainer>();
#endif
    sys->Add3DOFContainer(snow_ball);

    real youngs_modulus = 1.4e6;
    real poissons_ratio = 0.2;
    real rho = 400;

#if USE_RIGID
    snow_ball->mu = 0;
    snow_ball->alpha = 0;
    snow_ball->cohesion = 0.1;
#else
    snow_ball->tau = time_step * 4;
    snow_ball->epsilon = 1e-3;
    snow_ball->rho = rho;
    snow_ball->viscosity = false;
    snow_ball->artificial_pressure = false;
#endif

    snow_ball->theta_c = 1;
    snow_ball->theta_s = 1;
    snow_ball->lame_lambda = youngs_modulus * poissons_ratio / ((1. + poissons_ratio) * (1. - 2. * poissons_ratio));
    snow_ball->lame_mu = youngs_modulus / (2. * (1. + poissons_ratio));
    snow_ball->youngs_modulus = youngs_modulus;
    snow_ball->nu = poissons_ratio;
    snow_ball->alpha_flip = .95;
    snow_ball->hardening_coefficient = 10.0;
    ////snow_ball->rho = 400;
    snow_ball->mass = .01;
    snow_ball->kernel_radius = kernel_radius;
    snow_ball->collision_envelope = kernel_radius * 0.05;
    snow_ball->contact_recovery_speed = 10;
    snow_ball->contact_cohesion = 0;
    snow_ball->contact_mu = 0;
    snow_ball->max_velocity = 10;
    snow_ball->compliance = 0;

    ////snow_ball->tau = time_step * 4;
    ////snow_ball->epsilon = 1e-3;
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
    snow_ball->mass = rho * vol;

    utils::ChHCPSampler<> sampler(dist);
    utils::ChGenerator::PointVector points = sampler.SampleSphere(ChVector3d(0, 0, radius + radius * .5), radius);  // ChVector3d(radius, radius, radius));

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_fluid[i] = real3(0, 0, -5);
    }
    snow_ball->UpdatePosition(0);
    snow_ball->AddBodies(pos_fluid, vel_fluid);

    points = sampler.SampleBox(ChVector3d(1, 0, 0), ChVector3d(radius, radius, radius));

    pos_fluid.resize(points.size());
    vel_fluid.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        pos_fluid[i] = real3(points[i].x(), points[i].y(), points[i].z()) + origin;
        vel_fluid[i] = real3(-6, 0, 0);
    }

    return snow_ball;
}

ChVector3d CalculateCOM(std::shared_ptr<ChParticleContainer> snow_ball) {
    ChVector3d com = VNULL;
    auto n = snow_ball->GetNumParticles();
    for (uint i = 0; i < n; i++) {
        auto p = snow_ball->GetPos(i);
        com += ChVector3d(p.x, p.y, p.z);
    }
    com /= n;
    return com;
}

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

    auto snow_ball = AddSnowball(&sys);

    sys.GetSettings()->collision.collision_envelope = kernel_radius * .05;
    sys.GetSettings()->collision.bins_per_axis = vec3(2, 2, 2);

    // Create the fixed body
    auto bin = AddBody(&sys);

    // Initialize system
    sys.Initialize();

#ifdef CHRONO_VSG
    // Create run-time visualization
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Snow");
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0.1, -2, 0.0), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyTexture(SkyMode::BOX);
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(-CH_PI_2, CH_PI_4);
    vis->Initialize();
#else
    double time_end = 1;
#endif

    // Set output
    // ----------

    const std::string out_dir = GetChronoOutputPath() + "MCORE_SNOW";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    std::string out_file = out_dir + "/position.txt";

    // Perform the simulation
    // ----------------------

    double time = 0;
    ChWriterCSV csv(" ");

    while (true) {
#ifdef CHRONO_VSG
        if (!vis->Run())
            break;
        vis->Render();
#else
        if (time > time_end)
            break;
#endif

        sys.DoStepDynamics(time_step);
        time += time_step;

        auto com = CalculateCOM(snow_ball);
        csv << time << com << std::endl;
    }

    csv.WriteToFile(out_file);

#ifdef CHRONO_POSTPROCESS
    // Plot results
    // ------------

    postprocess::ChGnuPlot gplot(out_dir + "/position.gpl");
    gplot.SetGrid();
    std::string speed_title = "Snowball COM";
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("vertical position (m)");
    gplot.Plot(out_file, 1, 4, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif

    return 0;
}
