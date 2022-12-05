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
//  Two spheres are initially held together by adhesion, in the absence of gravity
//  Gravity is turned on first with F_gravity < F_adhesion, then with
//  F_gravity > F_adhesion. The spheres should stay in contact during case 1
//  but seperate during case 2.
//
// =============================================================================

#include "gtest/gtest.h"

#define SMC_SEQUENTIAL
#include "../utest_SMC.h"

// Test system parameterized by SMC contact force model
class CohesionTest : public ::testing::TestWithParam<ChSystemSMC::ContactForceModel> {
  protected:
    CohesionTest() {
        auto fmodel = GetParam();

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  // Default 2e5
        float p_ratio = 0.3f;      // Default 0.3f
        float s_frict = 0.3f;      // Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;      // Default 0.6f
        float roll_frict = 0.0f;   // Usually around 1E-3
        float spin_frict = 0.0f;   // Usually around 1E-3
        float cor_in = 0.0f;       // Default 0.4f
        float adDMT = 0.0f;        // Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     // Magnitude of the adhesion in the SPerko adhesion model
        ad = 10.0;                 // Constant cohesion value

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

        // Create an SMC system and set the system parameters
        sys = new ChSystemSMC();
        time_step = 3.0E-5;
        SetSimParameters(sys, ChVector<>(0, 0, 0), fmodel);

        sys->SetNumThreads(2);

        // Add the sphere to the system
        srad = 0.5;
        double smass = 1.0;
        ChVector<> spos(0, srad + 1e-2, 0);
        ChVector<> init_v(0, -0.1, 0);

        body1 = AddSphere(0, sys, mat, srad, smass, spos, init_v);
        body2 = AddSphere(1, sys, mat, srad, smass, spos * -1, init_v * -1);

        // Let the block settle of the plate before giving it a push
        double t_end = 1;
        while (sys->GetChTime() < t_end) {
            sys->DoStepDynamics(time_step);

            if (CalcAverageKE(sys, 1.0E-9)) {
                std::cout << "[settling] KE falls below threshold after " << sys->GetChTime() << " s\n";
                break;
            }
        }
    }

    ~CohesionTest() { delete sys; }

    ChSystemSMC* sys;
    std::shared_ptr<ChBody> body1;
    std::shared_ptr<ChBody> body2;
    double srad;
    float ad;
    double time_step;
};

TEST_P(CohesionTest, stick) {
    // Fix body1
    body1->SetBodyFixed(true);

    // Set gravitational acceleration below cohesion value
    sys->Set_G_acc(ChVector<>(0, -(ad - 2), 0));

    double t_end = sys->GetChTime() + 0.5;
    while (sys->GetChTime() < t_end) {
        sys->DoStepDynamics(time_step);
    }

    // Compare distance between sphere centers. Test passes if bodies did not detach
    double distance = std::abs(body2->GetPos().y() - body1->GetPos().y());
    std::cout << "distance: " << distance << std::endl;
    ASSERT_LT(distance, 2 * srad);
}

TEST_P(CohesionTest, detach) {
    // Fix body1
    body1->SetBodyFixed(true);

    // Set gravitational acceleration at (or above) cohesion value
    sys->Set_G_acc(ChVector<>(0, -(ad + 0.1), 0));

    double time_sim = sys->GetChTime() + 0.5;
    while (sys->GetChTime() < time_sim) {
        sys->DoStepDynamics(time_step);
    }

    // Compare distance between sphere centers. Test passes if bodies detach
    double distance = std::abs(body2->GetPos().y() - body1->GetPos().y());
    std::cout << "distance: " << distance << std::endl;
    ASSERT_GT(distance, 2 * srad);
}

INSTANTIATE_TEST_SUITE_P(ChronoSequential,
                         CohesionTest,
                         ::testing::Values(ChSystemSMC::ContactForceModel::Hooke,
                                           ChSystemSMC::ContactForceModel::Hertz,
                                           ChSystemSMC::ContactForceModel::PlainCoulomb));
