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
#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;

    // Create the ground body
    // ----------------------

    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto rail1 = std::make_shared<ChBoxShape>();
    rail1->GetBoxGeometry().SetLengths(ChVector<>(8, 0.1, 0.1));
    rail1->GetBoxGeometry().Pos = ChVector<>(0, 0, -1);
    ground->AddAsset(rail1);

    auto rail2 = std::make_shared<ChBoxShape>();
    rail2->GetBoxGeometry().SetLengths(ChVector<>(8, 0.1, 0.1));
    rail2->GetBoxGeometry().Pos = ChVector<>(0, 0, +1);
    ground->AddAsset(rail2);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.6f, 0.6f, 0.6f));
    ground->AddAsset(col);

    // Create the slider bodies
    // ------------------------

    auto slider1 = std::make_shared<ChBody>();
    system.AddBody(slider1);
    slider1->SetIdentifier(1);
    slider1->SetBodyFixed(false);
    slider1->SetCollide(false);
    slider1->SetMass(1);
    slider1->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
    slider1->SetPos(ChVector<>(-4, 0, -1));

    auto cyl1 = std::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().p1 = ChVector<>(-0.2, 0, 0);
    cyl1->GetCylinderGeometry().p2 = ChVector<>(+0.2, 0, 0);
    cyl1->GetCylinderGeometry().rad = 0.2;
    slider1->AddAsset(cyl1);

    auto col1 = std::make_shared<ChColorAsset>();
    col1->SetColor(ChColor(0.6f, 0, 0));
    slider1->AddAsset(col1);

    auto slider2 = std::make_shared<ChBody>();
    system.AddBody(slider2);
    slider2->SetIdentifier(1);
    slider2->SetBodyFixed(false);
    slider2->SetCollide(false);
    slider2->SetMass(1);
    slider2->SetInertiaXX(ChVector<>(0.1, 0.1, 01));
    slider2->SetPos(ChVector<>(-4, 0, +1));

    auto cyl2 = std::make_shared<ChCylinderShape>();
    cyl2->GetCylinderGeometry().p1 = ChVector<>(-0.2, 0, 0);
    cyl2->GetCylinderGeometry().p2 = ChVector<>(+0.2, 0, 0);
    cyl2->GetCylinderGeometry().rad = 0.2;
    slider2->AddAsset(cyl2);

    auto col2 = std::make_shared<ChColorAsset>();
    col2->SetColor(ChColor(0, 0, 0.6f));
    slider2->AddAsset(col2);

    // Create prismatic joints between ground and sliders
    // --------------------------------------------------

    // Add limit (Z min) on this prismatic joint.
    // The limit value relates to the Z component of the relative marker position 2->1
    auto prismatic1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, slider1, ChCoordsys<>(ChVector<>(0, 0, -1), Q_from_AngY(CH_C_PI_2)));
    prismatic1->GetLimit_Z().SetActive(true);
    prismatic1->GetLimit_Z().SetMin(-6);
    system.AddLink(prismatic1);

    auto prismatic2 = std::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(ground, slider2, ChCoordsys<>(ChVector<>(0, 0, +1), Q_from_AngY(CH_C_PI_2)));
    system.AddLink(prismatic2);

    // Add linear springs to the sliders
    // ---------------------------------

    auto spring1 = std::make_shared<ChLinkSpring>();
    spring1->Initialize(ground, slider1, true, ChVector<>(0, 0, -1), ChVector<>(0, 0, 0), false, 0);
    spring1->Set_SpringK(10);
    spring1->Set_SpringR(0);
    system.AddLink(spring1);
    spring1->AddAsset(std::make_shared<ChPointPointSpring>(0.1, 80, 15));

    auto spring2 = std::make_shared<ChLinkSpring>();
    spring2->Initialize(ground, slider2, true, ChVector<>(0, 0, +1), ChVector<>(0, 0, 0), false, 0);
    spring2->Set_SpringK(10);
    spring2->Set_SpringR(0);
    system.AddLink(spring2);
    spring2->AddAsset(std::make_shared<ChPointPointSpring>(0.1, 80, 15));

    // Create the Irrlicht application
    // -------------------------------

    ChIrrApp application(&system, L"Limits on LinkLockPrismatic demo", irr::core::dimension2d<irr::u32>(800, 600),
                         false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(-1, 1.5, -6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    // ---------------

    application.SetTimestep(0.001);
    bool max_lim_enabled = false;

    while (application.GetDevice()->run()) {
        // Enable also the Z max limit on the 1st prismatic joint
        if (!max_lim_enabled && slider1->GetPos().x() > 0) {
            prismatic1->GetLimit_Z().SetMax(-3);
            max_lim_enabled = true;
        }

        application.BeginScene();
        application.DrawAll();
        ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(+2, 0, 0), ChVector<>(+2, 0, -2),
                                irr::video::SColor(255, 255, 150, 0), true);
        if (max_lim_enabled) {
            ChIrrTools::drawSegment(application.GetVideoDriver(), ChVector<>(-1, 0, 0), ChVector<>(-1, 0, -2),
                                    irr::video::SColor(255, 255, 150, 0), true);
        }
        ChIrrTools::drawAllLinkframes(system, application.GetVideoDriver(), 1.0);
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
