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
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the sys
    // Set gravitational acceleration to zero
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(3, 2, 0.1, 10, true, false);
    sys.AddBody(ground);
    ground->SetFixed(true);

    // Create the sliding body
    // Give an initial angular velocity
    auto body = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 10, true, false);
    sys.AddBody(body);
    body->SetFixed(false);
    body->SetPos(ChVector3d(-1.25, -0.75, 0.1));
    body->SetAngVelLocal(ChVector3d(0.1, 0.1, 0.1));
    body->GetVisualShape(0)->SetColor(ChColor(0.6f, 0, 0));

    // Create the plane-plane constraint
    // Constrain the sliding body to move and rotate in the x-y plane
    // (i.e. the plane whose normal is the z-axis of the specified coord sys)
    auto plane_plane = chrono_types::make_shared<ChLinkLockPlanar>();
    plane_plane->Initialize(ground, body, ChFrame<>(ChVector3d(-1.25, -0.75, 0), ChQuaternion<>(1, 0, 0, 0)));
    sys.AddLink(plane_plane);

    // Create a linear spring (with default spring & damping coefficients)
    auto spring = chrono_types::make_shared<ChLinkTSDA>();
    sys.AddLink(spring);
    spring->SetSpringCoefficient(100);
    spring->SetDampingCoefficient(5);
    spring->Initialize(ground, body, true, ChVector3d(0, 0, 2), ChVector3d(0, 0, 0));
    spring->SetRestLength(1.9);
    auto spring_shape = chrono_types::make_shared<ChVisualShapeSpring>(0.05, 200, 25);
    spring_shape->SetColor(ChColor(0.0f, 0.3f, 0.8f));
    spring->AddVisualShape(spring_shape);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkLockPlanar");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(3, 0, 3));
    vis->AddTypicalLights();
    vis->EnableBodyFrameDrawing(true);

    // Simulation loop
    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
