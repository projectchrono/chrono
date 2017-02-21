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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    ChSystem system;
    system.Set_G_acc(ChVector<>(0.01, -1, 1));

    double L = 0.5; // distance for the revolute-translational joint

    // Create the ground body
    // ----------------------

    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    ground->SetPos(ChVector<>(0, 0, -1));

    auto cyl = std::make_shared<ChBoxShape>();
    cyl->GetBoxGeometry().Size = ChVector<>(3, 0.04, 0.06);
    ground->AddAsset(cyl);

    auto col_g = std::make_shared<ChColorAsset>();
    col_g->SetColor(ChColor(0, 0, 0.6f, 0));
    ground->AddAsset(col_g);

    // Create a pendulum body
    // ----------------------

    auto pend = std::make_shared<ChBody>();
    system.AddBody(pend);
    pend->SetIdentifier(1);
    pend->SetBodyFixed(false);
    pend->SetCollide(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector<>(0.2, 1, 1));

    // Initial position of the pendulum (horizontal, pointing towards positive X).
    pend->SetPos(ChVector<>(1.5, -L, -1));

    // Attach visualization assets.
    auto cyl_p = std::make_shared<ChCylinderShape>();
    cyl_p->GetCylinderGeometry().p1 = ChVector<>(-1.46, 0, 0);
    cyl_p->GetCylinderGeometry().p2 = ChVector<>(1.46, 0, 0);
    cyl_p->GetCylinderGeometry().rad = 0.2;
    pend->AddAsset(cyl_p);

    auto cyl_j = std::make_shared<ChCylinderShape>();
    cyl_j->GetCylinderGeometry().p1 = ChVector<>(-1.5, 0, 0.2);
    cyl_j->GetCylinderGeometry().p2 = ChVector<>(-1.5, 0, -0.2);
    cyl_j->GetCylinderGeometry().rad = 0.04;
    pend->AddAsset(cyl_j);

    auto col_p = std::make_shared<ChColorAsset>();
    col_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddAsset(col_p);

    // Create a revolute-translational joint to connect pendulum to ground.
    auto rev_trans = std::make_shared<ChLinkRevoluteTranslational>();
    system.AddLink(rev_trans);

    // Initialize the joint specifying a coordinate system (expressed in the
    // absolute frame) and a distance. The revolute side is attached to the
    // pendulum and the translational side to the ground.
    rev_trans->Initialize(pend, ground, ChCoordsys<>(ChVector<>(0, -L, -1), Q_from_AngZ(CH_C_PI_2)), L);

    // Create the Irrlicht application
    // -------------------------------

    ChIrrApp application(&system, L"ChLinkRevoluteTranslational demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-1.5, 2, 3));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // Render the connecting body.
        // Recall that the joint reference frame is given in the Body coordinates.
        ChCoordsys<> joint_csys = ground->GetCoord() >> rev_trans->GetLinkRelativeCoords();
        ChVector<> point1 = joint_csys.pos;
        ChVector<> point2 = joint_csys.TransformPointLocalToParent(ChVector<>(L, 0, 0));
        ChIrrTools::drawSegment(application.GetVideoDriver(), point1, point2, video::SColor(255, 0, 20, 0), true);

        // Render a line between the two points of the revolute-translational joint.
        ChIrrTools::drawSegment(application.GetVideoDriver(), rev_trans->GetPoint1Abs(), rev_trans->GetPoint2Abs(),
                                video::SColor(255, 120, 120, 120), true);

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
