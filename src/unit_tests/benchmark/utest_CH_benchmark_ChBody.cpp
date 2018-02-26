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

#include <iostream>

#include "../ChTestConfig.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;
using namespace std;

#define TIME(X, Y)                                \
    timer.start();                                \
    for (auto body : body_list) {                 \
        X;                                        \
    }                                             \
    timer.stop();                                 \
    cout << Y << timer() << endl;

#define TIMEBODY(X, Y) TIME(body->X, Y)

int main() {
    ChTimer<double> timer, full;
    const int num_bodies = 1000000;
    const double current_time = 1;
    const double time_step = .1;
    ChSystemNSC dynamics_system;

    for (int i = 0; i < num_bodies; i++) {
        auto body = std::make_shared<ChBody>();
        body->SetPos(ChVector<>(rand() % 1000 / 1000.0, rand() % 1000 / 1000.0, rand() % 1000 / 1000.0));
        dynamics_system.AddBody(body);
    }

    auto& body_list = dynamics_system.Get_bodylist();

    full.start();

    TIMEBODY(UpdateTime(current_time), "UpdateTime ");
    TIMEBODY(UpdateForces(current_time), "UpdateForces ");
    TIMEBODY(UpdateMarkers(current_time), "UpdateMarkers ");

    TIMEBODY(ClampSpeed(), "ClampSpeed ");
    TIMEBODY(ComputeGyro(), "ComputeGyro ");

    TIMEBODY(VariablesFbReset(), "VariablesFbReset ");
    TIMEBODY(VariablesFbLoadForces(time_step), "VariablesFbLoadForces ");
    TIMEBODY(VariablesQbLoadSpeed(), "VariablesQbLoadSpeed ");
    TIMEBODY(VariablesQbIncrementPosition(time_step), "VariablesQbIncrementPosition ");
    TIMEBODY(VariablesQbSetSpeed(time_step), "VariablesQbSetSpeed ");
    // TIMEBODY(VariablesBody().GetBodyInvInertia(), "GetBodyInvInertia ");

    full.stop();
    cout << "Total: " << full() << endl;
    timer.start();
    for (auto body : body_list) {
        body->UpdateTime(current_time);
        body->UpdateForces(current_time);
        body->UpdateMarkers(current_time);
        body->ClampSpeed();
        body->ComputeGyro();
        body->VariablesFbReset();
        body->VariablesFbLoadForces(time_step);
        body->VariablesQbLoadSpeed();
        body->VariablesQbIncrementPosition(time_step);
        body->VariablesQbSetSpeed(time_step);
    }
    timer.stop();
    cout << "SIngle Loop " << timer() << endl;

    return 0;
}
