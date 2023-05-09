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
#include "chrono/assets/ChPointPointShape.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    auto rail1 = chrono_types::make_shared<ChBoxShape>(8, 0.1, 0.1);
    ground->AddVisualShape(rail1, ChFrame<>(ChVector<>(0, 0, -1), QUNIT));

    auto rail2 = chrono_types::make_shared<ChBoxShape>(8, 0.1, 0.1);
    ground->AddVisualShape(rail2, ChFrame<>(ChVector<>(0, 0, +1), QUNIT));

    // Create the slider bodies
    auto slider1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider1);
    slider1->SetIdentifier(1);
    slider1->SetBodyFixed(false);
    slider1->SetCollide(false);
    slider1->SetMass(1);
    slider1->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));
    slider1->SetPos(ChVector<>(-4, 0, -1));

    auto cyl1 = chrono_types::make_shared<ChCylinderShape>(0.2, 0.4);
    cyl1->SetColor(ChColor(0.6f, 0, 0));
    slider1->AddVisualShape(cyl1, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));

    auto slider2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider2);
    slider2->SetIdentifier(1);
    slider2->SetBodyFixed(false);
    slider2->SetCollide(false);
    slider2->SetMass(1);
    slider2->SetInertiaXX(ChVector<>(0.1, 0.1, 01));
    slider2->SetPos(ChVector<>(-4, 0, +1));

    auto cyl2 = chrono_types::make_shared<ChCylinderShape>(0.2, 0.4);
    cyl2->SetColor(ChColor(0, 0, 0.6f));
    slider2->AddVisualShape(cyl2, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));

    // Create prismatic joints between ground and sliders
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(slider1, ground, ChCoordsys<>(ChVector<>(0, 0, -1), Q_from_AngY(CH_C_PI_2)));
    sys.AddLink(prismatic1);

    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(slider2, ground, ChCoordsys<>(ChVector<>(0, 0, +1), Q_from_AngY(CH_C_PI_2)));
    sys.AddLink(prismatic2);

    // Sine function
    double freq = 1;
    double ampl = 4;
    ////double omg = 2 * CH_C_PI * freq;
    auto mod = chrono_types::make_shared<ChFunction_Sine>(0.0, freq, ampl);

    // Actuate first slider using a link force
    prismatic1->GetForce_Z().SetActive(true);
    prismatic1->GetForce_Z().SetF(1);
    prismatic1->GetForce_Z().SetModulationF(mod);

    // Actuate second slider using a body force
    auto frc2 = chrono_types::make_shared<ChForce>();
    frc2->SetF_x(mod);
    slider2->AddForce(frc2);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Actuated prismatic joint");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(-1, 1.5, -6));
    vis->AddTypicalLights();
    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    double timestep = 1e-3;
    ChRealtimeStepTimer realtime_timer;
    ////double x0 = slider1->GetPos().x();

    while (vis->Run()) {
        ////double time = sys.GetChTime();

        // Output slider x position/velocity and analytical solution
        ////double x = slider1->GetPos().x();
        ////double x_d = slider1->GetPos_dt().x();
        ////double xa = x0 + (ampl / omg) * (time - std::sin(omg * time) / omg);
        ////double xa_d = (ampl / omg) * (1 - std::cos(omg * time));
        ////std::cout << time << "   " << x << " " << x_d << "   " << xa << " " << xa_d << std::endl;

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
