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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    ChSystem system;

    // Create the ground body
    // ----------------------

    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0.2);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -0.2);
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
    pend->SetPos(ChVector<>(1.5, 0, 0));

    // Attach visualization assets.
    auto cyl_p = std::make_shared<ChCylinderShape>();
    cyl_p->GetCylinderGeometry().p1 = ChVector<>(-1.46, 0, 0);
    cyl_p->GetCylinderGeometry().p2 = ChVector<>(1.46, 0, 0);
    cyl_p->GetCylinderGeometry().rad = 0.2;
    pend->AddAsset(cyl_p);

    auto col_p = std::make_shared<ChColorAsset>();
    col_p->SetColor(ChColor(0.6f, 0, 0));
    pend->AddAsset(col_p);

    // Create a revolute joint to connect pendulum to ground
    // -----------------------------------------------------

    auto rev = std::make_shared<ChLinkLockRevolute>();
    system.AddLink(rev);

    // Add limits to the Z rotation of the revolute joint
    double min_angle = 0;
    double max_angle = 0.75 * CH_C_PI;
    rev->GetLimit_Rz()->Set_active(true);
    rev->GetLimit_Rz()->Set_min(min_angle);
    rev->GetLimit_Rz()->Set_max(max_angle);

    // Initialize the joint specifying a coordinate system (expressed in the absolute frame).
    rev->Initialize(ground, pend, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Create the Irrlicht application
    // -------------------------------

    ChIrrApp application(&system, L"Limits on LinkLockRevolute demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-2, 1.5, 5));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    // ---------------

    application.SetTimestep(0.001);

    system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ChVector<> p0(0, 0, 0);
        ChVector<> p1(std::cos(min_angle), -std::sin(min_angle), 0);
        ChVector<> p2(std::cos(max_angle), -std::sin(max_angle), 0);
        ChIrrTools::drawSegment(application.GetVideoDriver(), p0, p0 + 4.0 * p1, video::SColor(255, 255, 150, 0), true);
        ChIrrTools::drawSegment(application.GetVideoDriver(), p0, p0 + 4.0 * p2, video::SColor(255, 255, 150, 0), true);
        ChIrrTools::drawAllLinkframes(system, application.GetVideoDriver(), 1.0);
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
