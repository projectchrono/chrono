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
// Joint reaction forces
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -1));

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.04, 0.4);
    ground->AddVisualShape(cyl, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Create a pendulum body, initially horizontal along positive X
    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetFixed(false);
    pend->EnableCollision(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector3d(1, 1, 1));
    // pend->SetPos(ChVector3d(0, 0, -1));
    pend->SetPos(ChVector3d(1, 0, 0));
    pend->SetRot(QuatFromAngleY(-CH_PI_2));

    auto cyl_p = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2);
    cyl_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddVisualShape(cyl_p);

    // Create revolute joint with rotation axis about global Y
    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    sys.AddLink(rev);
    rev->Initialize(ground, pend, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Create the Irrlicht application
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkRevoluteSpherical demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, -3, 0));
    vis->AddTypicalLights();

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // Render the joint frames
        tools::drawCoordsys(vis.get(), rev->GetFrame1Abs().GetCoordsys());
        tools::drawCoordsys(vis.get(), rev->GetFrame2Abs().GetCoordsys());

        // Render reaction force on 1st connected body
        if (false) {
            // Extract reaction force on body1 and express it in absolute frame
            auto frc = rev->GetFrame1Abs().TransformDirectionLocalToParent(rev->GetReaction1().force);

            std::cout << "|F1| = " << frc.Length() << std::endl;

            auto p1 = rev->GetFrame1Abs().GetPos();
            auto p2 = p1 + frc;
            tools::drawSegment(vis.get(), p1, p2, ChColor(0.5f, 0.5f, 0), false);
        }

        // Render reaction force on 2nd connected body
        if (true) {
            // Extract reaction force on body1 and express it in absolute frame
            auto frc = rev->GetFrame2Abs().TransformDirectionLocalToParent(rev->GetReaction2().force);

            std::cout << "|F2| = " << frc.Length() << std::endl;

            auto p1 = rev->GetFrame2Abs().GetPos();
            auto p2 = p1 + frc;
            tools::drawSegment(vis.get(), p1, p2, ChColor(0.5f, 0.5f, 0), false);
        }

        vis->EndScene();

        sys.DoStepDynamics(0.001);
    }

    return 0;
}
