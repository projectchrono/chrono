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
// Demonstration of the revolute-spherical composite joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(1, -1, 1));

    // Create the ground body
    // ----------------------

    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 1.2);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0.8);
    cyl->GetCylinderGeometry().rad = 0.04;
    ground->AddAsset(cyl);

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
    pend->SetPos(ChVector<>(1.5, 0, 1));

    // Attach visualization assets.
    auto cyl_p = std::make_shared<ChCylinderShape>();
    cyl_p->GetCylinderGeometry().p1 = ChVector<>(-0.96, 0, 0);
    cyl_p->GetCylinderGeometry().p2 = ChVector<>(0.96, 0, 0);
    cyl_p->GetCylinderGeometry().rad = 0.2;
    pend->AddAsset(cyl_p);

    auto sph_p = std::make_shared<ChSphereShape>();
    sph_p->Pos = ChVector<>(-1, 0, 0);
    sph_p->GetSphereGeometry().rad = 0.04;
    pend->AddAsset(sph_p);

    auto col_p = std::make_shared<ChColorAsset>();
    col_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddAsset(col_p);

    // Create a revolute-spherical joint to connect pendulum to ground.
    auto rev_sph = std::make_shared<ChLinkRevoluteSpherical>();
    system.AddLink(rev_sph);

    // Initialize the pendulum specifying a coordinate system (expressed in the
    // absolute frame) and a distance.
    rev_sph->Initialize(ground, pend, ChCoordsys<>(ChVector<>(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)), 0.5);

    // Alternatively, the joint can be initialized by specifying a point and a
    // direction on the first body and a point on the second body.
    // rev_sph->Initialize(ground, pend, false, ChVector<>(0, 0, 1), ChVector<>(0, 0, 1), ChVector<>(0.5, 0, 1));

    // Create the Irrlicht application
    // -------------------------------

    ChIrrApp application(&system, L"ChLinkRevoluteSpherical demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Cache for point trajectory plot
    std::vector<ChVector<> > trajectory;

    // Simulation loop
    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // Render the rev-sph massless link.
        ChIrrTools::drawSegment(application.GetVideoDriver(), rev_sph->GetPoint1Abs(), rev_sph->GetPoint2Abs(),
                                video::SColor(255, 0, 20, 0), true);

        // Render the point trajectory
        ChIrrTools::drawPolyline(application.GetVideoDriver(), trajectory, video::SColor(255, 0, 150, 0), false);

        application.DoStep();

        // Add latest point to trajectory. Only keep a buffer of latest 1500 points.
        trajectory.push_back(rev_sph->GetPoint2Abs());
        if (trajectory.size() > 1500)
            trajectory.erase(trajectory.begin());

        application.EndScene();
    }

    return 0;
}
