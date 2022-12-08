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
// This project simulates the collision of a sphere against another sphere and
// compares the measured coefficient of restitution against the input
// coefficient of restitution
//
// =============================================================================

#include "gtest/gtest.h"

#define SMC_SEQUENTIAL
#include "../utest_SMC.h"

// Test system parameterized by SMC contact force model and input COR
class CorNormalTest : public ::testing::TestWithParam<std::tuple<ChSystemSMC::ContactForceModel, float>> {
  protected:
    CorNormalTest() {
        auto fmodel = std::get<0>(GetParam());

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;                // Default 2e5
        float p_ratio = 0.3f;                    // Default 0.3f
        float s_frict = 0.3f;                    // Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;                    // Default 0.6f
        float roll_frict = 0.0f;                 // Usually around 1E-3
        float spin_frict = 0.0f;                 // Usually around 1E-3
        float cor_in = std::get<1>(GetParam());  // Input COR
        float ad = 0.0f;                         // Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;                      // Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;                   // Magnitude of the adhesion in the SPerko adhesion model

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
        SetSimParameters(sys, ChVector<>(0, 0, 0), fmodel);

        sys->SetNumThreads(2);

        // Add the sphere to the system
        double srad = 0.5;
        double smass = 1.0;
        ChVector<> spos(0, srad * 1.25, 0);
        ChVector<> init_v(0, -1, 0);

        body1 = AddSphere(0, sys, mat, srad, smass, spos, init_v);
        body2 = AddSphere(1, sys, mat, srad, smass, spos * -1, init_v * -1);

        // Calculate motion parameters prior to collision
        rel_vm_in = (body2->GetPos_dt() - body1->GetPos_dt()).Length();
    }

    ~CorNormalTest() { delete sys; }

    ChSystemSMC* sys;
    std::shared_ptr<ChBody> body1;
    std::shared_ptr<ChBody> body2;
    double time_step;
    double rel_vm_in;
};

TEST_P(CorNormalTest, impact) {
    double t_end = 0.5;
    while (sys->GetChTime() < t_end) {
        sys->DoStepDynamics(time_step);
    }

    // Calculate output COR and compare against input COR. Test passes if difference below 1e-3
    double rel_vm_out = (body2->GetPos_dt() - body1->GetPos_dt()).Length();
    double cor_out = rel_vm_out / rel_vm_in;
    double cor_in = std::get<1>(GetParam());
    std::cout << ForceModel_name(std::get<0>(GetParam())) << "  "  //
              << cor_in << "  " << cor_out << "  "                 //
              << std::abs(cor_out - cor_in) << "\n";
    ASSERT_NEAR(cor_out, cor_in, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(ChronoSequential,
                         CorNormalTest,
                         ::testing::Combine(::testing::Values(ChSystemSMC::ContactForceModel::Hooke,
                                                              ChSystemSMC::ContactForceModel::Hertz,
                                                              ChSystemSMC::ContactForceModel::PlainCoulomb),
                                            ::testing::Values(0.0, 0.25, 0.5, 0.75, 1.0)));
