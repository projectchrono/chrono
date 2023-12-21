// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Mangoni, Dario Fusai
// =============================================================================
//
// Test for the unilateral distance constraint
//
// =============================================================================


#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono/solver/ChSolverPSOR.h"


// Use the namespaces of Chrono
using namespace chrono;


int main() {
    // System setup ============================================================
    ChSystemNSC system;

    double gravity = 10.0; //without sign
    ChVector<> body_pos0(0.1, 0.9, 0.0);
    double max_dist = 1;


    system.Set_G_acc(ChVector<>(0, -gravity, 0));



    // Bodies ------------------------------------------------------------------
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetBodyFixed(true);
    system.Add(floor);


    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1);
    body->SetPos(body_pos0);
    system.Add(body);

    // Links -------------------------------------------------------------------
    auto unilink = chrono_types::make_shared<ChLinkDistance>();
    unilink->Initialize(body, floor, false, body->GetPos(), floor->GetPos(), false, max_dist, ChLinkDistance::Mode::UNILATERAL_MAXDISTANCE);
    //unilink->Initialize(floor, body, false, floor->GetPos(), body->GetPos(), false, max_dist, ChLinkDistance::Mode::UNILATERAL_MAXDISTANCE);
    system.Add(unilink);


    // Simulation ==============================================================
    double timestep = 0.01;
    double simulation_time_after_contact = 2.0;

    // VI solver required
    auto solver = chrono_types::make_shared<ChSolverPSOR>();
    system.SetSolver(solver);
    system.SetSolverMaxIterations(500);

    system.DoFullAssembly();
    //GetLog() << "body_acc: " << body->GetPos_dtdt() << "\n";


    // analytic solution (assuming constraint between body and the origin)
    double contact_height = -sqrt(max_dist*max_dist - body_pos0.x()*body_pos0.x());
    double fall_height = body_pos0.y() - contact_height;
    double expected_activation_time = sqrt(2*fall_height/gravity);



    // CHECK 1: check if the body is free falling until hitting the constraint, no side motion
    // a failure in this case can be due to wrong reaction forces or to an artificial clamping in IntLoadConstraint_C
    bool straight_fall = true;
    while (std::abs(unilink->Get_react_force().x()) < 1e-8){
        ChVector<> body_acc = body->GetPos_dtdt();
        //GetLog() << "body_acc: " << body_acc << "\n";
        if (std::abs(body_acc.x() - 0.0) > 1e-6 || std::abs(body_acc.y() + gravity) > 1e-6 || std::abs(body_acc.z() - 0.0) > 1e-6)
            straight_fall = false;
        system.DoStepDynamics(timestep);
    }
    GetLog() << "Object on free and straight fall? " << (straight_fall ? "YES: PASSED" : "NO: FAILED") << "\n";

    
    // CHECK 2: check if the contact with the constraint happened at the expected time (partially covered by the vertical acceleration check)
    GetLog() << "Expected hit at: " << expected_activation_time << "; Happened between: " << system.GetChTime() - timestep << " and: " << system.GetChTime() << " |";
    
    bool proper_hit_time = expected_activation_time<=system.GetChTime() && expected_activation_time>=system.GetChTime()- timestep;
    GetLog() << (proper_hit_time ? " PASSED\n" : "FAILED\n");

    // CHECK 3: check if, after activation of the link, the trajectory is within the sphere of radius 'max_dist'
    // the check is actually stricter since it doesn't even allow a rebounce i.e. it is exactly on the surface of the sphere
    bool on_sphere = true;
    for (auto extra_step = 0; extra_step<simulation_time_after_contact/timestep; ++extra_step){
        system.DoStepDynamics(timestep);
        on_sphere = on_sphere && (std::abs(Vdot(body->GetPos(), body->GetPos()) - max_dist*max_dist)<1e-4);
        //GetLog() << "Violation: " << Vdot(body->GetPos(), body->GetPos()) - max_dist*max_dist << "\n";
    }
    GetLog() << "Object on sphere? " << (on_sphere ? "YES: PASSED" : "NO: FAILED") << "\n";

    return !(straight_fall && proper_hit_time && on_sphere);
}