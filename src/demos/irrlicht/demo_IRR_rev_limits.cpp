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
// Demonstration of using rotation limits on a revolute joint. Note that this
// capability is only available for ChLinkLockRevolute.  It is not available
// for ChLinkRevolute (which uses a different formulation).
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0.2);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -0.2);
    cyl->GetCylinderGeometry().rad = 0.04;
    ground->AddVisualShape(cyl);

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
    pend->SetPos(ChVector<>(1.5, 0, 0));

    // Attach visualization assets.
    auto cyl_p = chrono_types::make_shared<ChCylinderShape>();
    cyl_p->GetCylinderGeometry().p1 = ChVector<>(-1.46, 0, 0);
    cyl_p->GetCylinderGeometry().p2 = ChVector<>(1.46, 0, 0);
    cyl_p->GetCylinderGeometry().rad = 0.2;
    cyl_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_p);

    // Create a revolute joint to connect pendulum to ground
    // -----------------------------------------------------

    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    sys.AddLink(rev);

    // Add limits to the Z rotation of the revolute joint
    double min_angle = 0;
    double max_angle = 0.75 * CH_C_PI;
    rev->GetLimit_Rz().SetActive(true);
    rev->GetLimit_Rz().SetMin(min_angle);
    rev->GetLimit_Rz().SetMax(max_angle);

    // Initialize the joint specifying a coordinate sys (expressed in the absolute frame).
    rev->Initialize(ground, pend, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Create the Irrlicht application
    // -------------------------------

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Limits on LinkLockRevolute demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(-2, 1.5, 5));
    vis->AddTypicalLights();
    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    // ---------------

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        ChVector<> p0(0, 0, 0);
        ChVector<> p1(std::cos(min_angle), -std::sin(min_angle), 0);
        ChVector<> p2(std::cos(max_angle), -std::sin(max_angle), 0);
        tools::drawSegment(vis.get(), p0, p0 + 4.0 * p1, ChColor(1, 0.5f, 0), true);
        tools::drawSegment(vis.get(), p0, p0 + 4.0 * p2, ChColor(1, 0.5f, 0), true);
        vis->EndScene();

        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
