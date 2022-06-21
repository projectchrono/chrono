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
// Authors: Radu Serban, Cecily Sunday
// =============================================================================
//
//  This project simulates a box with a non-zero initial horizontal velocity
//  sliding across a horizontal plate. See Xiang et al. (2009) Test 0.
//  The purpose of this test is to validate the implementation of sliding
//  friction in ChIterativeSolverMulticoreSMC.
//
// =============================================================================

#include "gtest/gtest.h"

#define SMC_SEQUENTIAL
#include "../utest_SMC.h"

// Test system parameterized by SMC contact force model
class SlidingGravityTest : public ::testing::TestWithParam<ChSystemSMC::ContactForceModel> {
  protected:
    SlidingGravityTest() {
        auto fmodel = GetParam();

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  // Default 2e5
        float p_ratio = 0.3f;      // Default 0.3f
        s_frict = 0.5f;            // Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.5f;      // Default 0.6f
        float roll_frict = 0.0f;   // Usually around 1E-3
        float spin_frict = 0.0f;   // Usually around 1E-3
        float cor_in = 0.0f;       // Default 0.4f
        float ad = 0.0f;           // Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;        // Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     // Magnitude of the adhesion in the SPerko adhesion model

        // For the Flores model, set cor to a lrger value
        if (fmodel == ChSystemSMC::ContactForceModel::Flores)
            cor_in = 0.1f;

        auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
        mat->SetYoungModulus(y_modulus);
        mat->SetPoissonRatio(p_ratio);
        mat->SetSfriction(s_frict);
        mat->SetKfriction(k_frict);
        mat->SetRollingFriction(roll_frict);
        mat->SetSpinningFriction(spin_frict);
        mat->SetRestitution(cor_in);
        mat->SetAdhesion(ad);
        mat->SetAdhesionMultDMT(adDMT);
        mat->SetAdhesionSPerko(adSPerko);

        // Create a multicore SMC system and set the system parameters
        sys = new ChSystemSMC();
        time_step = 2.0E-5;
        gravity = -9.81;
        SetSimParameters(sys, ChVector<>(0, gravity, 0), fmodel, ChSystemSMC::TangentialDisplacementModel::OneStep);

        sys->SetNumThreads(2);

        // Add the wall to the system
        double wmass = 10.0;
        ChVector<> wsize(8, 1, 3);
        ChVector<> wpos(0, -wsize.y() / 2 - 0.5, 0);
        ChVector<> init_wv(0, 0, 0);

        auto wall = AddWall(-1, sys, mat, wsize, wmass, wpos, init_wv, true);

        // Add the block to the system
        double bmass = 1.0;
        ChVector<> bsize(0.5, 0.5, 0.5);
        ChVector<> bpos(0, bsize.y() / 2 - 0.49, 0);
        ChVector<> init_bv(0, 0, 0);

        body = AddWall(0, sys, mat, bsize, bmass, bpos, init_bv, false);

        // Let the block settle of the plate before giving it a push
        double t_end = 2;
        while (sys->GetChTime() < t_end) {
            sys->DoStepDynamics(time_step);

            if (CalcKE(sys, 1.0E-9)) {
                std::cout << "[settling] KE falls below threshold after " << sys->GetChTime() << " s\n";
                break;
            }
        }
    }

    ~SlidingGravityTest() { delete sys; }

    ChSystemSMC* sys;
    std::shared_ptr<ChBody> body;
    double time_step;
    double gravity;
    float s_frict;
};

TEST_P(SlidingGravityTest, sliding) {
    // Get current body position and calculate analytical travel distance.
    double init_pos = body->GetPos().x();

    // Give the block a push in the horizontal direction
    ChVector<> init_bv(5, 0, 0);
    body->SetPos_dt(init_bv);

    double t_start = sys->GetChTime();
    double t_end = t_start + 2;
    bool stopped = false;
    while (sys->GetChTime() < t_end) {
        sys->DoStepDynamics(time_step);

        if (CalcKE(sys, 1.0E-8)) {
            std::cout << "[sliding] KE falls below threshold after " << sys->GetChTime() - t_start << " s\n";
            stopped = true;
            break;
        }
    }

    // Check that the body actually stopped
    ASSERT_TRUE(stopped);

    // Analytical travel distance (reference)
    double d_ref = std::abs(init_bv.Length2() / (2 * (double)s_frict * gravity));

    // Check actual travel distance against reference. Test passes if difference below 0.5%
    double d_sim = body->GetPos().x() - init_pos;
    double d_err = std::abs((d_ref - d_sim) / d_ref) * 100;
    std::cout << ForceModel_name(GetParam()) << "  " << d_ref << "  " << d_sim << "  " << d_err << "\n";
    ASSERT_LT(d_err, 0.5);
}

INSTANTIATE_TEST_SUITE_P(ChronoMulticore,
                         SlidingGravityTest,
                         ::testing::Values(ChSystemSMC::ContactForceModel::Hooke,
                                           ChSystemSMC::ContactForceModel::Hertz,
                                           ChSystemSMC::ContactForceModel::PlainCoulomb));
