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

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.04, 0.4);
    ground->AddVisualShape(cyl);

    // Create a pendulum body
    // ----------------------

    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetFixed(false);
    pend->EnableCollision(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector3d(0.2, 1, 1));

    // Initial position of the pendulum (horizontal, pointing towards positive X).
    pend->SetPos(ChVector3d(1.5, 0, 0));

    // Attach visualization assets.
    auto cyl_p = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2.92);
    cyl_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_p, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    // Create a revolute joint to connect pendulum to ground
    // -----------------------------------------------------

    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    sys.AddLink(rev);

    // Add limits to the Z rotation of the revolute joint
    double min_angle = 0;
    double max_angle = 0.75 * CH_PI;
    rev->LimitRz().SetActive(true);
    rev->LimitRz().SetMin(min_angle);
    rev->LimitRz().SetMax(max_angle);

    // Initialize the joint specifying a coordinate sys (expressed in the absolute frame).
    rev->Initialize(ground, pend, ChFrame<>(ChVector3d(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Create the Irrlicht application
    // -------------------------------

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Limits on LinkLockRevolute demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(-2, 1.5, 5));
    vis->AddTypicalLights();
    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    // ---------------

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        ChVector3d p0(0, 0, 0);
        ChVector3d p1(std::cos(min_angle), -std::sin(min_angle), 0);
        ChVector3d p2(std::cos(max_angle), -std::sin(max_angle), 0);
        tools::drawSegment(vis.get(), p0, p0 + 4.0 * p1, ChColor(1, 0.5f, 0), true);
        tools::drawSegment(vis.get(), p0, p0 + 4.0 * p2, ChColor(1, 0.5f, 0), true);
        vis->EndScene();

        sys.DoStepDynamics(1e-3);
    }

    return 0;
}
