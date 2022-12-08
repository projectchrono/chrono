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
//  This project simulates a ball rolling across a plate. See Ai et al. (2011)
//  Validation Test 1. The purpose of this test is to validate the
//  implementation of rolling friction in ChIterativeSolverMulticoreSMC.
//
// =============================================================================

#include "gtest/gtest.h"

#define SMC_SEQUENTIAL
#include "../utest_SMC.h"

// Test system parameterized by SMC contact force model
class RollingGravityTest : public ::testing::TestWithParam<ChSystemSMC::ContactForceModel> {
  protected:
    RollingGravityTest() {
        auto fmodel = GetParam();

        // Create a shared material to be used by the all bodies
        float y_modulus = 2.0e5f;  /// Default 2e5
        float p_ratio = 0.3f;      /// Default 0.3f
        float s_frict = 0.3f;      /// Usually in 0.1 range, rarely above. Default 0.6f
        float k_frict = 0.3f;      /// Default 0.6f
        float roll_frict = 0.2f;   /// Usually around 1E-3
        float spin_frict = 0.0f;   /// Usually around 1E-3
        float cor_in = 0.0f;       /// Default 0.4f
        float ad = 0.0f;           /// Magnitude of the adhesion in the Constant adhesion model
        float adDMT = 0.0f;        /// Magnitude of the adhesion in the DMT adhesion model
        float adSPerko = 0.0f;     /// Magnitude of the adhesion in the SPerko adhesion model

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
        time_step = 3.0E-5;
        SetSimParameters(sys, ChVector<>(0, -9.81, 0), fmodel);

        sys->SetNumThreads(2);

        // Add the wall to the system
        int id = -1;
        double wmass = 10.0;
        ChVector<> wsize(4, 1, 4);
        ChVector<> wpos(0, -wsize.y() / 2, 0);

        auto wall = AddWall(id, sys, mat, wsize, wmass, wpos, ChVector<>(0, 0, 0), true);

        // Add the sphere to the system
        double srad = 0.5;
        double smass = 1.0;
        ChVector<> spos(0, srad + 1e-2, 0);
        ChVector<> init_v(0, -0.1, 0);

        body = AddSphere(++id, sys, mat, srad, smass, spos, init_v);

        // Let the sphere settle on the plate before giving it a push
        double t_end = 1;
        while (sys->GetChTime() < t_end) {
            sys->DoStepDynamics(time_step);

            if (CalcKE(sys, 1.0E-9)) {
                std::cout << "[settling] KE falls below threshold after " << sys->GetChTime() << " s\n";
                break;
            }
        }
    }

    ~RollingGravityTest() { delete sys; }

    ChSystemSMC* sys;
    std::shared_ptr<ChBody> body;
    double time_step;
};

TEST_P(RollingGravityTest, rolling) {
    // Give the sphere a push in the horizontal direction
    ChVector<> init_v(1, 0, 0);
    body->SetPos_dt(init_v);

    double t_start = sys->GetChTime();
    double t_end = t_start + 1;
    while (sys->GetChTime() < t_end) {
        sys->DoStepDynamics(time_step);

        if (CalcKE(sys, 1.0E-9)) {
            std::cout << "[rolling] KE falls below threshold after " << sys->GetChTime() - t_start << " s\n";
            break;
        }
    }

    // Check results. The sphere's rotational velocity should be < 1e-3.
    double wvel = body->GetWvel_par().Length();
    std::cout << ForceModel_name(GetParam()) << "  " << wvel << "\n";
    ASSERT_LT(wvel, 1e-3);
}

INSTANTIATE_TEST_SUITE_P(ChronoSequential,
                         RollingGravityTest,
                         ::testing::Values(ChSystemSMC::ContactForceModel::Hooke,
                                           ChSystemSMC::ContactForceModel::Hertz,
                                           ChSystemSMC::ContactForceModel::PlainCoulomb));
