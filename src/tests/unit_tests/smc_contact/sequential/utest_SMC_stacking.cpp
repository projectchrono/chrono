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
// This project simulates 5 spheres dropping on top of on another and ending in
// a stacked state. See Asmar et al. (2002) DEM Validation Test 4.
//
// =============================================================================

#include "gtest/gtest.h"

#define SMC_SEQUENTIAL
#include "../utest_SMC.h"

// Test system parameterized by SMC contact force model
class StackingTest : public ::testing::TestWithParam<ChSystemSMC::ContactForceModel> {
  protected:
    StackingTest() {
        auto fmodel = GetParam();

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  // Default 2e5
        float p_ratio = 0.3f;      // Default 0.3f
        float s_frict = 0.3f;      // Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;      // Default 0.6f
        float roll_frict = 0.0f;   // Usually around 1E-3
        float spin_frict = 0.0f;   // Usually around 1E-3
        float cor_in = 0.3f;       // Default 0.4f
        float ad = 0.0f;           // Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;        // Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     // Magnitude of the adhesion in the SPerko adhesion model

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
        time_step = 3.0E-5;
        SetSimParameters(sys, ChVector<>(0, -9.81, 0), fmodel);

        sys->SetNumThreads(2);

        // Add the wall to the system
        double wmass = 10.0;
        ChVector<> wsize(8, 1, 8);
        ChVector<> wpos(0, -wsize.y() / 2, 0);

        auto wall = AddWall(-1, sys, mat, wsize, wmass, wpos, ChVector<>(0, 0, 0), true);

        // Add the spheres to the system
        double srad = 1.0;
        double smass = 1.0;

        for (int sid = 0; sid < 5; ++sid) {
            ChVector<> spos(0, 5 + 2.5 * srad * sid, 0);
            auto body = AddSphere(sid, sys, mat, srad, smass, spos, ChVector<>(0, 0, 0));
            bodies.push_back(body);
        }
    }

    ~StackingTest() { delete sys; }

    ChSystemSMC* sys;
    std::vector<std::shared_ptr<ChBody>> bodies;
    double time_step;
};

TEST_P(StackingTest, stacking) {
    double t_end = 10;
    bool stopped = false;
    while (sys->GetChTime() < t_end) {
        sys->DoStepDynamics(time_step);

        if (CalcAverageKE(sys, 1.0E-9)) {
            std::cout << "[stacking] KE falls below threshold after " << sys->GetChTime() << " s\n";
            stopped = true;
            break;
        }
    }

    // Check that the bodies actually stopped
    ASSERT_TRUE(stopped);

    // Check that spheres are in a stack, with (x,z) positions at (0,0) and no rotation.
    for (auto& body : bodies) {
        const ChVector<>& pos = body->GetPos();
        const ChQuaternion<>& rot = body->GetRot();
        std::cout << pos << "    " << rot << "\n";
        ASSERT_NEAR(pos.x(), 0.0, 1e-6);
        ASSERT_NEAR(pos.z(), 0.0, 1e-6);
        ASSERT_NEAR(rot.e0(), 1.0, 1e-6);
    }
}

INSTANTIATE_TEST_SUITE_P(ChronoSequential,
                         StackingTest,
                         ::testing::Values(ChSystemSMC::ContactForceModel::Hooke,
                                           ChSystemSMC::ContactForceModel::Hertz,
                                           ChSystemSMC::ContactForceModel::PlainCoulomb));
