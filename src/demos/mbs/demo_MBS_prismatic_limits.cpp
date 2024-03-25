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
// Demonstration of using limits on a translational joint.
// The model is built with gravity acting in the negative Y direction.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto rail1 = chrono_types::make_shared<ChVisualShapeBox>(8, 0.1, 0.1);
    ground->AddVisualShape(rail1, ChFrame<>(ChVector3d(0, 0, -1), QUNIT));

    auto rail2 = chrono_types::make_shared<ChVisualShapeBox>(8, 0.1, 0.1);
    ground->AddVisualShape(rail2, ChFrame<>(ChVector3d(0, 0, +1), QUNIT));

    // Create the slider bodies
    // ------------------------

    auto slider1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider1);
    slider1->SetFixed(false);
    slider1->EnableCollision(false);
    slider1->SetMass(1);
    slider1->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    slider1->SetPos(ChVector3d(-4, 0, -1));

    auto cyl1 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.4);
    cyl1->SetColor(ChColor(0.6f, 0, 0));
    slider1->AddVisualShape(cyl1, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    auto slider2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider2);
    slider2->SetFixed(false);
    slider2->EnableCollision(false);
    slider2->SetMass(1);
    slider2->SetInertiaXX(ChVector3d(0.1, 0.1, 01));
    slider2->SetPos(ChVector3d(-4, 0, +1));

    auto cyl2 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.4);
    cyl2->SetColor(ChColor(0, 0, 0.6f));
    slider2->AddVisualShape(cyl2, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    // Create prismatic joints between ground and sliders
    // --------------------------------------------------

    // Add limit (Z min) on this prismatic joint.
    // The limit value relates to the Z component of the relative marker position 2->1
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, slider1, ChFrame<>(ChVector3d(0, 0, -1), QuatFromAngleY(CH_PI_2)));
    prismatic1->LimitZ().SetActive(true);
    prismatic1->LimitZ().SetMin(-6);
    sys.AddLink(prismatic1);

    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(ground, slider2, ChFrame<>(ChVector3d(0, 0, +1), QuatFromAngleY(CH_PI_2)));
    sys.AddLink(prismatic2);

    // Add linear springs to the sliders
    // ---------------------------------

    auto spring1 = chrono_types::make_shared<ChLinkTSDA>();
    spring1->Initialize(ground, slider1, true, ChVector3d(0, 0, -1), ChVector3d(0, 0, 0));
    spring1->SetRestLength(0.0);
    spring1->SetSpringCoefficient(10);
    spring1->SetDampingCoefficient(0);
    sys.AddLink(spring1);
    spring1->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.1, 80, 15));

    auto spring2 = chrono_types::make_shared<ChLinkTSDA>();
    spring2->Initialize(ground, slider2, true, ChVector3d(0, 0, +1), ChVector3d(0, 0, 0));
    spring2->SetRestLength(0.0);
    spring2->SetSpringCoefficient(10);
    spring2->SetDampingCoefficient(0);
    sys.AddLink(spring2);
    spring2->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.1, 80, 15));

    // Create the Irrlicht application
    // -------------------------------

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Limits on LinkLockPrismatic demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-1, 1.5, -6));
    vis->AddTypicalLights();
    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    // ---------------

    double timestep = 0.001;
    ChRealtimeStepTimer realtime_timer;
    bool max_lim_enabled = false;

    while (vis->Run()) {
        // Enable also the Z max limit on the 1st prismatic joint
        if (!max_lim_enabled && slider1->GetPos().x() > 0) {
            prismatic1->LimitZ().SetMax(-3);
            max_lim_enabled = true;
        }

        vis->BeginScene();
        vis->Render();
        tools::drawSegment(vis.get(), ChVector3d(+2, 0, 0), ChVector3d(+2, 0, -2), ChColor(1, 0.5f, 0), true);
        if (max_lim_enabled) {
            tools::drawSegment(vis.get(), ChVector3d(-1, 0, 0), ChVector3d(-1, 0, -2), ChColor(1, 0.5f, 0), true);
        }
        vis->EndScene();

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
