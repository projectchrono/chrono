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
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of the plane-plane joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the system
    // Set gravitational acceleration to zero
    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(3, 2, 0.1, 10, true, false);
    system.AddBody(ground);
    ground->SetBodyFixed(true);

    // Create the sliding body
    // Give an initial angular velocity
    auto body = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 10, true, false);
    system.AddBody(body);
    body->SetBodyFixed(false);
    body->SetPos(ChVector<>(-1.25, -0.75, 0.1));
    body->SetWvel_loc(ChVector<>(0.1, 0.1, 0.1));

    auto body_col = chrono_types::make_shared<ChColorAsset>();
    body_col->SetColor(ChColor(0.6f, 0, 0));
    body->AddAsset(body_col);

    // Create the plane-plane constraint
    // Constrain the sliding body to move and rotate in the x-y plane
    // (i.e. the plane whose normal is the z-axis of the specified coord sys)
    auto plane_plane = chrono_types::make_shared<ChLinkLockPlanePlane>();
    plane_plane->Initialize(ground, body, ChCoordsys<>(ChVector<>(-1.25, -0.75, 0), ChQuaternion<>(1, 0, 0, 0)));
    system.AddLink(plane_plane);

    // Create a linear spring (with default spring & damping coefficients)
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->SetSpringCoefficient(100);
    spring->SetDampingCoefficient(5);
    spring->Initialize(ground, body, true, ChVector<>(0, 0, 2), ChVector<>(0, 0, 0), false, 1.9);
    system.AddLink(spring);

    // Create the Irrlicht application
    ChIrrApp application(&system, L"ChLinkLockPlanePlane", irr::core::dimension2d<irr::u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(3, 0, 3));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        tools::drawSpring(application.GetVideoDriver(), 0.05, spring->GetPoint1Abs(), spring->GetPoint2Abs(),
                               irr::video::SColor(0, 255, 255, 100), 80, 15, true);
        tools::drawAllCOGs(system, application.GetVideoDriver(), 2);
        application.EndScene();
        application.DoStep();
    }

    return 0;
}
