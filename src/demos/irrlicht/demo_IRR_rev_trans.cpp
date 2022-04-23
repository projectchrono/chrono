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

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0.01, -1, 1));

    double L = 0.5;  // distance for the revolute-translational joint

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    ground->SetPos(ChVector<>(0, 0, -1));

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(10, 0.04, 0.06);
    box->SetColor(ChColor(0, 0, 0.6f));
    ground->AddVisualShape(box, ChFrame<>(ChVector<>(5, 0, 0)));

    // Create a pendulum body
    // ----------------------

    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetIdentifier(1);
    pend->SetBodyFixed(false);
    pend->SetCollide(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector<>(0.2, 1, 1));

    // Initial position of the pendulum (horizontal, pointing towards positive X).
    pend->SetPos(ChVector<>(1.5, -L, -1));

    // Attach visualization assets.
    auto cyl_p = chrono_types::make_shared<ChCylinderShape>();
    cyl_p->GetCylinderGeometry().p1 = ChVector<>(-1.46, 0, 0);
    cyl_p->GetCylinderGeometry().p2 = ChVector<>(1.46, 0, 0);
    cyl_p->GetCylinderGeometry().rad = 0.2;
    cyl_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_p);

    auto cyl_j = chrono_types::make_shared<ChCylinderShape>();
    cyl_j->GetCylinderGeometry().p1 = ChVector<>(-1.5, 0, 0.2);
    cyl_j->GetCylinderGeometry().p2 = ChVector<>(-1.5, 0, -0.2);
    cyl_j->GetCylinderGeometry().rad = 0.04;
    cyl_j->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_j);

    // Create a revolute-translational joint to connect pendulum to ground.
    auto rev_trans = chrono_types::make_shared<ChLinkRevoluteTranslational>();
    sys.AddLink(rev_trans);

    // Initialize the joint specifying a coordinate sys (expressed in the
    // absolute frame) and a distance. The revolute side is attached to the
    // pendulum and the translational side to the ground.
    rev_trans->Initialize(pend, ground, ChCoordsys<>(ChVector<>(0, -L, -1), Q_from_AngZ(CH_C_PI_2)), L);

    // Create the Irrlicht application
    // -------------------------------

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkRevoluteTranslational demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(-1.5, 2, 3));
    vis->AddTypicalLights();

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();

        // Render the connecting body.
        // Recall that the joint reference frame is given in the Body coordinates.
        ChCoordsys<> joint_csys = ground->GetCoord() >> rev_trans->GetLinkRelativeCoords();
        ChVector<> point1 = joint_csys.pos;
        ChVector<> point2 = joint_csys.TransformPointLocalToParent(ChVector<>(L, 0, 0));
        tools::drawSegment(vis->GetVideoDriver(), point1, point2, video::SColor(255, 0, 20, 0), true);

        // Render a line between the two points of the revolute-translational joint.
        tools::drawSegment(vis->GetVideoDriver(), rev_trans->GetPoint1Abs(), rev_trans->GetPoint2Abs(),
                           video::SColor(255, 120, 120, 120), true);

        vis->EndScene();

        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
