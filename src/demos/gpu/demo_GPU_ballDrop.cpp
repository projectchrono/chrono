// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Luning Fang
// =============================================================================
// Impact test: Granular material settling in a cylindrical container
// to generate the bed for balldrop test. Once particles are settled,
// a projectile modeled as boundary condition is dropped
// with impact velocity 1m/s
// =============================================================================

#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_gpu/physics/ChSystemGpu.h"

using namespace chrono;
using namespace chrono::gpu;

void setMatreialProperty(ChSystemGpu& gran_sys) {
    double cor_p = 0.5;  // use cor_p = 0.9 for sand or glass beads
    double cor_w = 0.5;
    double youngs_modulus = 1e8;
    double mu_s2s = 0.16;
    double mu_s2w = 0.45;
    double mu_roll = 0.09;
    double poisson_ratio = 0.24;

    gran_sys.UseMaterialBasedModel(true);
    gran_sys.SetYoungModulus_SPH(youngs_modulus);
    gran_sys.SetYoungModulus_WALL(youngs_modulus);
    gran_sys.SetRestitution_SPH(cor_p);
    gran_sys.SetRestitution_WALL(cor_w);
    gran_sys.SetPoissonRatio_SPH(poisson_ratio);
    gran_sys.SetPoissonRatio_WALL(poisson_ratio);
    gran_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);
    gran_sys.SetRollingCoeff_SPH2SPH(mu_roll);
    gran_sys.SetRollingCoeff_SPH2WALL(mu_roll);
    gran_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gran_sys.SetStaticFrictionCoeff_SPH2SPH(mu_s2s);
    gran_sys.SetStaticFrictionCoeff_SPH2WALL(mu_s2w);
}

int main(int argc, char* argv[]) {
    // unit gcm
    double sphere_radius = 0.5f;
    double sphere_density = 2.48;
    double box_X = 31.5;
    double box_Y = 31.5;
    double box_Z = 30.0;

    double grav_X = 0.0f;
    double grav_Y = 0.0f;
    double grav_Z = -980.0f;

    float step_size = 1e-5f;
    float time_settle = 1.5f;
    float time_impact = 0.5f;
    float time_end = time_settle + time_impact;

    ChSystemGpu gran_sys(sphere_radius, sphere_density, ChVector<float>(box_X, box_Y, box_Z));
    gran_sys.SetBDFixed(true);

    // create cylinder containter
    ChVector<float> cyl_center(0.0f, 0.0f, 0.0f);
    float cyl_rad = std::min(box_X, box_Y) / 2.0f;
    gran_sys.CreateBCCylinderZ(cyl_center, cyl_rad, false, true);

    // generate a cloud of particles
    std::vector<chrono::ChVector<float>> body_points;
    utils::PDSampler<float> sampler(2.001 * sphere_radius);
    ChVector<float> sampler_center(0.0f, 0.0f, 0.0f);
    body_points = sampler.SampleCylinderZ(sampler_center, cyl_rad - 4 * sphere_radius, box_Z / 2 - 4 * sphere_radius);
    auto numSpheres = body_points.size();
    std::cout << "Numbers of particles created: " << numSpheres << std::endl;
    gran_sys.SetParticles(body_points);

    // set up material properties
    setMatreialProperty(gran_sys);

    // set up projectile radius, mass and impact velocity
    float projectile_radius = 5.0f;
    float projectile_mass = 1000;
    ChVector<float> projectile_pos(0, 0, box_Z / 2.0f - projectile_radius);
    ChVector<float> projectile_impact_velo(0.0, 0.0, -100.0f);
    size_t projectile_id = gran_sys.CreateBCSphere(projectile_pos, projectile_radius, true, true, projectile_mass);

    gran_sys.SetGravitationalAcceleration(ChVector<float>(grav_X, grav_Y, grav_Z));
    gran_sys.SetPsiFactors(32.0f, 16.0f);
    gran_sys.SetFixedStepSize(step_size);
    gran_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::FORWARD_EULER);

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + "/ballDrop/";
    filesystem::create_directory(filesystem::path(out_dir));

    gran_sys.SetParticleOutputFlags(ABSV);
    gran_sys.SetParticleOutputMode(CHGPU_OUTPUT_MODE::CSV);

    gran_sys.Initialize();
    // Fix projectile until granular particles are settled
    gran_sys.DisableBCbyID(projectile_id);

    double t = 0.0f;
    int curr_step_settle = 0;

    double frame_step_settle = 0.01;
    double frame_step_impact = 5e-4;
    char filename[200];

    // settling phase
    while (t < time_settle) {
        gran_sys.AdvanceSimulation(frame_step_settle);

        // write particle output
        sprintf(filename, "%s/settling%06d.csv", out_dir.c_str(), curr_step_settle);
        gran_sys.WriteParticleFile(std::string(filename));

        curr_step_settle++;
        t += frame_step_settle;
    }

    double max_particle_z = gran_sys.GetMaxParticleZ();

    // set projectile position and velocity
    projectile_pos.z() = max_particle_z + sphere_radius + projectile_radius;
    gran_sys.EnableBCbyID(projectile_id);
    gran_sys.SetBCSpherePosition(projectile_id, projectile_pos);
    gran_sys.SetBCSphereVelocity(projectile_id, projectile_impact_velo);

    ChVector<float> bc_pos(0.0f, 0.0f, 0.0f);
    ChVector<float> bc_velo(0.0f, 0.0f, 0.0f);
    ChVector<float> bc_force(0.0f, 0.0f, 0.0f);

    // impact phase
    while (t < time_end) {
        gran_sys.AdvanceSimulation(frame_step_impact);

        t += frame_step_impact;

        bc_pos = gran_sys.GetBCSpherePosition(projectile_id);
        bc_velo = gran_sys.GetBCSphereVelocity(projectile_id);
        gran_sys.GetBCReactionForces(projectile_id, bc_force);

        // output projectile position, velocity and acceleration in z direction
        printf("%e, %e, %e, %e\n", t, bc_pos.z(), bc_velo.z(), bc_force.z() / projectile_mass + grav_Z);
    }

    return 0;
}
