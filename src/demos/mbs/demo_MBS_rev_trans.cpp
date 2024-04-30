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
// Demonstration of the revolute-translational composite joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0.01, -1, 1));

    double L = 0.5;  // distance for the revolute-translational joint

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);
    ground->SetPos(ChVector3d(0, 0, -1));

    auto box = chrono_types::make_shared<ChVisualShapeBox>(20, 0.08, 0.12);
    box->SetColor(ChColor(0, 0, 0.6f));
    ground->AddVisualShape(box, ChFrame<>(ChVector3d(5, 0, 0)));

    // Create a pendulum body
    // ----------------------

    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetFixed(false);
    pend->EnableCollision(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector3d(0.2, 1, 1));

    // Initial position of the pendulum (horizontal, pointing towards positive X).
    pend->SetPos(ChVector3d(1.5, -L, -1));

    // Attach visualization assets.
    auto cyl_p = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2.92);
    cyl_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_p, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    auto cyl_j = chrono_types::make_shared<ChVisualShapeCylinder>(0.04, 0.4);
    cyl_j->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_j, ChFrame<>(ChVector3d(-1.5, 0, 0), QUNIT));

    // Create a revolute-translational joint to connect pendulum to ground.
    auto rev_trans = chrono_types::make_shared<ChLinkRevoluteTranslational>();
    sys.AddLink(rev_trans);

    // Initialize the joint specifying a coordinate sys (expressed in the
    // absolute frame) and a distance. The revolute side is attached to the
    // pendulum and the translational side to the ground.
    rev_trans->Initialize(pend, ground, ChCoordsys<>(ChVector3d(0, -L, -1), QuatFromAngleZ(CH_PI_2)), L);

    // Create the Irrlicht application
    // -------------------------------

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkRevoluteTranslational demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-1.5, 2, 3));
    vis->AddTypicalLights();

    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // Render the connecting body.
        // Recall that the joint reference frame is given in the Body coordinates.
        ChVector3d p1 = rev_trans->GetFrame1Abs().GetPos();
        ChVector3d p2 = rev_trans->GetFrame1Abs().TransformPointLocalToParent(ChVector3d(L, 0, 0));
        tools::drawSegment(vis.get(), p1, p2, ChColor(0, 1, 0), true);

        vis->EndScene();

        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
