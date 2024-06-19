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
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Chrono::Multicore unit test for body falling under gravity
// =============================================================================

#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "unit_testing.h"

using namespace chrono;

TEST(ChronoMulticore, gravity) {
    ChVector3d gravity = ChVector3d(0, -9.80665, 0);
    ChSystemMulticoreNSC msystem;
    msystem.SetGravitationalAcceleration(gravity);
    msystem.SetNumThreads(1);

    auto ball = chrono_types::make_shared<ChBody>();
    ChVector3d pos = ChVector3d(0, 0, 0);
    ChVector3d vel = ChVector3d(2, 2, 0);
    ball->SetMass(1);
    ball->SetPos(pos);
    ball->SetPosDt(vel);
    msystem.AddBody(ball);

    for (int i = 0; i < 1000; i++) {
        msystem.DoStepDynamics(1e-5);
    }

    auto time = msystem.GetChTime();
    auto pos_final = ball->GetPos();
    auto pos_ref = pos + vel * time + 0.5 * gravity * time * time;
    // std::cout << "time = " << msystem.GetChTime() << std::endl;
    // std::cout << "diff = " << (pos_final - pos_ref).Length() << std::endl;

    ASSERT_LT((pos_final - pos_ref).Length(), 1e-6);
}
