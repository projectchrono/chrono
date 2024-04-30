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
#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create the ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto rail1 = chrono_types::make_shared<ChVisualShapeBox>(8, 0.1, 0.1);
    ground->AddVisualShape(rail1, ChFrame<>(ChVector3d(0, 0, -1), QUNIT));

    auto rail2 = chrono_types::make_shared<ChVisualShapeBox>(8, 0.1, 0.1);
    ground->AddVisualShape(rail2, ChFrame<>(ChVector3d(0, 0, +1), QUNIT));

    // Create the slider bodies
    auto slider1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider1);
    slider1->SetFixed(false);
    slider1->EnableCollision(false);
    slider1->SetMass(1);
    slider1->SetInertiaXX(ChVector3d(0.1, 0.1, 0.1));
    slider1->SetPos(ChVector3d(-4, 0, -1));

    auto cyl1 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.4);
    cyl1->SetColor(ChColor(0.6f, 0, 0));
    slider1->AddVisualShape(cyl1, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    auto slider2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(slider2);
    slider2->SetFixed(false);
    slider2->EnableCollision(false);
    slider2->SetMass(1);
    slider2->SetInertiaXX(ChVector3d(0.1, 0.1, 01));
    slider2->SetPos(ChVector3d(-4, 0, +1));

    auto cyl2 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.4);
    cyl2->SetColor(ChColor(0, 0, 0.6f));
    slider2->AddVisualShape(cyl2, ChFrame<>(VNULL, QuatFromAngleY(CH_PI_2)));

    // Create prismatic joints between ground and sliders
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(slider1, ground, ChFrame<>(ChVector3d(0, 0, -1), QuatFromAngleY(CH_PI_2)));
    sys.AddLink(prismatic1);

    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(slider2, ground, ChFrame<>(ChVector3d(0, 0, +1), QuatFromAngleY(CH_PI_2)));
    sys.AddLink(prismatic2);

    // Sine function
    double freq = 1;
    double ampl = 4;
    ////double omg = 2 * CH_PI * freq;
    auto mod = chrono_types::make_shared<ChFunctionSine>(ampl, freq);

    // Actuate first slider using a link force
    prismatic1->ForceZ().SetActive(true);
    prismatic1->ForceZ().SetActuatorForceTorque(1);
    prismatic1->ForceZ().SetActuatorModulation(mod);

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
    vis->AddCamera(ChVector3d(-1, 1.5, -6));
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
        ////double x_d = slider1->GetPosDt().x();
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
