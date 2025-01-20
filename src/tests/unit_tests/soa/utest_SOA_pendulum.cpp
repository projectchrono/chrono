// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Tests for a 3D simple pendulum1 modeled using SOA relative coordinates
//
// =============================================================================

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChRevoluteBody.h"

#include "../ut_utils.h"

using std::cout;
using std::endl;
using namespace chrono;
using namespace chrono::soa;

TEST(SOA_pendulum, kinematics) {
    ChSoaAssembly soa;

    double mass1 = 1;
    double L1 = 1;
    ChMatrix33d inertia1(ChVector3d(0.01, mass1 * L1 * L1 / 12, mass1 * L1 * L1 / 12));

    double mass2 = 1;
    double L2 = 1;
    ChMatrix33d inertia2(ChVector3d(0.01, mass2 * L2 * L2 / 12, mass2 * L2 * L2 / 12));

    // Douuble pendulum with reference frames at COMs
    {
        ChMassProps pendulum1_mprops(mass1, VNULL, inertia1);
        auto pendulum1 =
            chrono_types::make_shared<ChRevoluteBody>(soa.getGroundBody(), pendulum1_mprops,                 //
                                                      ChFramed(VNULL, Q_ROTATE_Z_TO_Y),                      //
                                                      ChFramed(ChVector3d(-L1 / 2, 0, 0), Q_ROTATE_Z_TO_Y),  //
                                                      "chain1_pendulum1");
        pendulum1->setRelPos(CH_PI_4);
        pendulum1->setRelVel(0.5);
        soa.AddBody(pendulum1);

        ChMassProps pendulum2_mprops(mass2, VNULL, inertia2);
        auto pendulum2 = chrono_types::make_shared<ChRevoluteBody>(pendulum1, pendulum2_mprops,                 //
                                                                   ChFramed(ChVector3d(+L1 / 2, 0, 0), QUNIT),  //
                                                                   ChFramed(ChVector3d(-L2 / 2, 0, 0), QUNIT),  //
                                                                   "chain1_pendulum2");
        pendulum2->setRelPos(CH_PI_2);
        pendulum2->setRelVel(0.1);
        soa.AddBody(pendulum2);
    }

    // Double pendulum with reference frames at inboard joints
    {
        ChMassProps pendulum1_mprops(mass1, ChVector3d(L1 / 2, 0, 0), inertia1);
        auto pendulum1 = chrono_types::make_shared<ChRevoluteBody>(soa.getGroundBody(), pendulum1_mprops,  //
                                                                   ChFramed(VNULL, Q_ROTATE_Z_TO_Y),       //
                                                                   ChFramed(VNULL, Q_ROTATE_Z_TO_Y),       //
                                                                   "chain2_pendulum1");
        pendulum1->setRelPos(CH_PI_4);
        pendulum1->setRelVel(0.5);
        soa.AddBody(pendulum1);

        ChMassProps pendulum2_mprops(mass2, ChVector3d(L2 / 2, 0, 0), inertia2);
        auto pendulum2 = chrono_types::make_shared<ChRevoluteBody>(pendulum1, pendulum2_mprops,             //
                                                                   ChFramed(ChVector3d(+L1, 0, 0), QUNIT),  //
                                                                   ChFramed(VNULL, QUNIT),                  //
                                                                   "chain2_pendulum2");
        pendulum2->setRelPos(CH_PI_2);
        pendulum2->setRelVel(0.1);
        soa.AddBody(pendulum2);
    }

    // Initialize assembly (perform position- and velocity-level traversal)
    soa.Initialize();

    // Traverse bodies in assembly and print their absolute position, orientation, and velocities
    for (const auto& b : soa.getBodies()) {
        cout << b->getName() << endl;
        cout << "          p: " << b->getAbsLoc() << " | q: " << b->getAbsQuat() << endl;
        cout << "          v: " << b->getAbsLinVel() << " | o: " << b->getAbsAngVel() << endl;
        cout << "    COM   p: " << b->getAbsCOMLoc() << " | v: " << b->getAbsCOMVel() << endl;
    }

    auto c1_p1 = soa.findBody("chain1_pendulum1");
    auto c1_p2 = soa.findBody("chain1_pendulum2");
    auto c2_p1 = soa.findBody("chain2_pendulum1");
    auto c2_p2 = soa.findBody("chain2_pendulum2");

    Assert_near(c1_p1->getAbsCOMLoc(), c2_p1->getAbsCOMLoc());
    Assert_near(c1_p1->getAbsCOMVel(), c2_p1->getAbsCOMVel());

    Assert_near(c1_p2->getAbsCOMLoc(), c2_p2->getAbsCOMLoc());
    Assert_near(c1_p2->getAbsCOMVel(), c2_p2->getAbsCOMVel());
}
