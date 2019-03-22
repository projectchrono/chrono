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
// Demonstration of actuating a translational joint with a ChLinkForce.
// The model is built with gravity acting in the negative Y direction.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground body
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
    auto prismatic1 = std::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(slider1, ground, ChCoordsys<>(ChVector<>(0, 0, -1), Q_from_AngY(CH_C_PI_2)));
    system.AddLink(prismatic1);

    auto prismatic2 = std::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(slider2, ground, ChCoordsys<>(ChVector<>(0, 0, +1), Q_from_AngY(CH_C_PI_2)));
    system.AddLink(prismatic2);

    // Sine function
    double freq = 1;
    double ampl = 4;
    double omg = 2 * CH_C_PI * freq;
    auto mod = std::make_shared<ChFunction_Sine>(0.0, freq, ampl);

    // Actuate first slider using a link force
    prismatic1->GetForce_Z().SetActive(true);
    prismatic1->GetForce_Z().SetF(1);
    prismatic1->GetForce_Z().SetModulationF(mod);

    // Actuate second slider using a body force
    auto frc2 = std::make_shared<ChForce>();
    frc2->SetF_x(mod);
    slider2->AddForce(frc2);

    // Create the Irrlicht application
    ChIrrApp application(&system, L"Actuated prismatic joint", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(-1, 1.5, -6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    application.SetTimestep(1e-3);

    // Simulation loop
    double x0 = slider1->GetPos().x();
    while (application.GetDevice()->run()) {
        double time = system.GetChTime();

        // Output slider x position/velocity and analytical solution
        ////double x = slider1->GetPos().x();
        ////double x_d = slider1->GetPos_dt().x();
        ////double xa = x0 + (ampl / omg) * (time - std::sin(omg * time) / omg);
        ////double xa_d = (ampl / omg) * (1 - std::cos(omg * time));
        ////std::cout << time << "   " << x << " " << x_d << "   " << xa << " " << xa_d << std::endl;

        application.BeginScene();
        application.DrawAll();
        ChIrrTools::drawAllLinkframes(system, application.GetVideoDriver(), 1.0);
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
