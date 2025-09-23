// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
// Validation test: one particle rolling on the boundary surface
// This test will check the rolling friction model implemented in chrono::dem
// When interaction is between Particle vs Boundary Wall
// =============================================================================

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono_dem/physics/ChSystemDem.h"

#include "ut_dem_utils.h"

using namespace chrono;
using namespace chrono::dem;

TEST(demFrictionRolling, check) {
    float density = 1.53f;
    float radius = 0.5f;
    float g = 980.f;
    float mu_s = 0.5f;
    float mu_r = 0.0008f;

    float precision_KE = 1e-3f;
    float precision_pos = 1e-3f;

    float mass = 4.f / 3.f * (float)CH_PI * pow(radius, 3.f) * density;
    float penetration = pow(mass * abs(-g) / 1e7f, 2.f / 3.f);

    float inertia = 2.f / 5.f * mass * pow(radius, 2.f);
    float settled_pos = -20.f / 2.f + radius - penetration;

    // setup simulation
    ChSystemDem dem_sys(radius, density, ChVector3f(20.f, 20.f, 20.f));

    // set normal force model
    dem_sys.SetKn_SPH2SPH(1e7);
    dem_sys.SetKn_SPH2WALL(1e7);
    dem_sys.SetGn_SPH2SPH(1e4);
    dem_sys.SetGn_SPH2WALL(1e4);

    // set tangential force model
    dem_sys.SetKt_SPH2SPH(1e7);
    dem_sys.SetKt_SPH2WALL(1e7);
    dem_sys.SetGt_SPH2SPH(1e4);
    dem_sys.SetGt_SPH2WALL(1e4);

    // set sliding friction model
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CHUNG);
    dem_sys.SetStaticFrictionCoeff_SPH2SPH(mu_s);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(mu_s);

    // set rolling friction model
    dem_sys.SetRollingMode(CHDEM_ROLLING_MODE::SCHWARTZ);
    dem_sys.SetRollingCoeff_SPH2SPH(mu_r);
    dem_sys.SetRollingCoeff_SPH2WALL(mu_r);

    dem_sys.SetPsiFactors(32, 16);

    // set gravity
    dem_sys.SetGravitationalAcceleration(ChVector3d(0.f, 0.f, -g));

    // add only one ball
    std::vector<ChVector3f> body_point = {ChVector3f(0, 0, settled_pos + 0.02)};
    std::vector<ChVector3f> velocity = {ChVector3f(1.0, 0.0, 0.0)};
    dem_sys.SetParticles(body_point, velocity);

    float step_size = 1e-4f;
    float curr_time = 0;
    float end_time = 3.0f;
    float time_start_check = 0.1f;
    bool settled = false;

    dem_sys.SetFixedStepSize(step_size);
    dem_sys.SetBDFixed(true);
    dem_sys.Initialize();

    while (curr_time < end_time) {
        dem_sys.AdvanceSimulation(step_size);
        curr_time += step_size;

        std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << std::flush;
        if (curr_time > time_start_check) {
            ChVector3f pos = dem_sys.GetParticlePosition(0);
            float vel = dem_sys.GetParticleVelocity(0).Length();
            float omg = dem_sys.GetParticleAngVelocity(0).Length();
            float KE = 0.5f * mass * vel * vel + 0.5f * inertia * omg * omg;
            std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << "  " << KE << std::flush;

            // stop simulation if the kinetic energy falls below threshold
            if (KE < precision_KE) {
                settled = true;
                break;
            }
        } else {
            std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << std::flush;
        }
    }

    // check whether the ball settles
    ASSERT_TRUE(settled);

    // check position x, y ,z components
    ChVector3d end_pos = dem_sys.GetParticlePosition(0);
    ASSERT_TRUE(end_pos.x() > 0.0f);
    ASSERT_NEAR(end_pos.y(), 0.0f, precision_pos);
    ASSERT_NEAR(end_pos.z(), settled_pos, precision_pos);
}
