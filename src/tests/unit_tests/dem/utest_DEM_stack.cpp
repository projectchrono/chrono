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
// Validation test: stacking unit test consisting of 5 particles
// This test will check the end positions of all 5 particles
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

TEST(demStack, check) {
    float density = 1.53f;
    float radius = 0.5f;
    float g = 980.f;
    float mu_s = 0.5f;
    float mu_r = 0.0008f;

    float precision_KE = 1e-5f;
    float precision_pos = 1e-2f;

    float mass = 4.f / 3.f * (float)CH_PI * pow(radius, 3.f) * density;
    float penetration = pow(mass * abs(-g) / 1e7f, 2.f / 3.f);

    float inertia = 2.f / 5.f * mass * pow(radius, 2.f);
    float settled_pos = -100.f / 2.0f + radius - penetration;

    // Setup simulation
    ChSystemDem dem_sys(radius, density, ChVector3f(100.f, 100.f, 100.f));
    dem_sys.SetGravitationalAcceleration(ChVector3d(0, 0, -g));
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CHUNG);

    // set normal force model
    dem_sys.SetKn_SPH2SPH(1e7);
    dem_sys.SetKn_SPH2WALL(1e7);
    dem_sys.SetGn_SPH2SPH(2e4);
    dem_sys.SetGn_SPH2WALL(2e4);

    // set tangential force model
    dem_sys.SetKt_SPH2SPH(2e6);
    dem_sys.SetKt_SPH2WALL(1e6);
    dem_sys.SetGt_SPH2SPH(50);
    dem_sys.SetGt_SPH2WALL(50);
    dem_sys.SetStaticFrictionCoeff_SPH2SPH(mu_s);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(mu_s);

    // set rolling friction model
    dem_sys.SetRollingMode(CHDEM_ROLLING_MODE::SCHWARTZ);
    dem_sys.SetRollingCoeff_SPH2SPH(mu_r);
    dem_sys.SetRollingCoeff_SPH2WALL(mu_r);

    dem_sys.SetPsiFactors(32, 16);

    // set up balls for simulation
    std::vector<ChVector3f> body_points;
    std::vector<ChVector3f> velocity;
    for (int i = 0; i < 5; i++) {
        body_points.push_back(ChVector3f(0.f, 0.f, settled_pos + radius * 3.f * i));
        velocity.push_back(ChVector3f(0.0f, 0.0f, 0.0f));
    }

    dem_sys.SetParticles(body_points, velocity);

    float step_size = 1e-4f;
    float curr_time = 0.f;
    float end_time = 3.f;
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
            float KE = 0.f;
            for (int i = 0; i < 5; i++) {
                float vel = dem_sys.GetParticleVelocity(i).Length();
                float omg = dem_sys.GetParticleAngVelocity(i).Length();
                KE += 0.5f * mass * vel * vel + 0.5f * inertia * omg * omg;
            }

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

    // check whether the balls settle
    ASSERT_TRUE(settled);

    // check end position with theoretical values
    for (int i = 0; i < 5; i++) {
        ASSERT_NEAR(dem_sys.GetParticlePosition(i).x(), 0, precision_pos);
        ASSERT_NEAR(dem_sys.GetParticlePosition(i).y(), 0, precision_pos);
        ASSERT_NEAR(dem_sys.GetParticlePosition(i).z(), settled_pos + radius * 2 * i - penetration * i, precision_pos);
    }
}
