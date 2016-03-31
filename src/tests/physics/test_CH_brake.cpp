//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <cmath>

#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// ====================================================================================

int main(int argc, char* argv[]) {
    // ---------------------
    // Simulation parameters
    // ---------------------

    double gravity = -10;     // gravitational acceleration
    double time_step = 1e-3;  // integration step size

    // -------------------------------
    // Parameters for the wheel
    // -------------------------------
    ChVector<> init_pos(0, 0, 0);
    // ChQuaternion<> init_rot(1, 0, 0, 0);
    // ChQuaternion<> init_rot(0.866025, 0, 0.5, 0);
    ChQuaternion<> init_rot(0.7071068, 0, 0.7071068, 0);
    // ChQuaternion<> init_rot(0.25882, 0, 0.965926, 0);
    // ChQuaternion<> init_rot(0, 0, 1, 0);

    // -----------------
    // Create the system
    // -----------------

    ChSystem system;

    system.Set_G_acc(ChVector<>(0, gravity, 0));


    // Create ground
    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(0);
    ground->SetCollide(false);
    ground->SetBodyFixed(true);

    // Create wheel
    auto wheel = std::make_shared<ChBody>();
    system.AddBody(wheel);
    wheel->SetIdentifier(1);
    wheel->SetMass(100);
    wheel->SetPos(init_pos);
    wheel->SetRot(init_rot);
    wheel->SetWvel_loc(ChVector<>(0, 0, 100));
    wheel->SetCollide(false);
    wheel->SetBodyFixed(false);

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = 0.2;
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, -0.05);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0,  0.05);
    wheel->AddAsset(cyl);

    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    wheel->AddAsset(texture);

    // Revolute joint
    auto joint = std::make_shared<ChLinkLockRevolute>();
    system.AddLink(joint);
    joint->Initialize(ground, wheel, ChCoordsys<>(init_pos, init_rot));

    // Brake
    auto brake = std::make_shared<ChLinkBrake>();
    system.AddLink(brake);

    // Equivalent ways of initializing the brake link
    ////brake->Initialize(ground, wheel, ChCoordsys<>(init_pos, init_rot));
    ////brake->Initialize(ground, wheel, wheel->GetCoord() * joint->GetMarker2()->GetCoord());
    brake->Initialize(ground, wheel, true, joint->GetMarker1()->GetCoord(), joint->GetMarker2()->GetCoord());

    // Create the Irrlicht visualization
    ChIrrApp application(&system, L"Brake test", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1, 0.5, -1));
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Set solver
    ChLcpIterativeMINRES* minres_solver = new ChLcpIterativeMINRES;
    minres_solver->SetDiagonalPreconditioning(true);
    system.ChangeLcpSolverSpeed(minres_solver);
    system.SetIterLCPmaxItersSpeed(100);
    system.SetTolForce(1e-6);

    // ---------------
    // Simulation loop
    // ---------------
    application.SetTimestep(time_step);

    irr::gui::IGUIFont* font = application.GetIGUIEnvironment()->getBuiltInFont();
    irr::core::rect<s32> text_box1(600, 100, 700, 120);
    irr::core::rect<s32> text_box2(600, 130, 700, 150);
    irr::core::rect<s32> text_box3(600, 160, 700, 180);
    irr::video::SColor text_col(255, 20, 20, 20);
    char msg[100];

    double modulation = 0;
    double max_torque = 200;
    bool monitor = true;

    while (application.GetDevice()->run()) {
        double time = system.GetChTime();

        application.BeginScene();
        application.DrawAll();
        ChIrrTools::drawAllCOGs(system, application.GetVideoDriver(), 1);
        sprintf(msg, "Time:    %.2f", time);
        font->draw(msg, text_box1, text_col);
        sprintf(msg, "Omega:   %.2f", wheel->GetWvel_loc().z);
        font->draw(msg, text_box2, text_col);
        sprintf(msg, "Braking: %.2f", modulation);
        font->draw(msg, text_box3, text_col);
        application.DoStep();
        application.EndScene();

        if (time > 2) {
            if (time < 3) modulation = time - 2;
            else if (time < 4) modulation = 4 - time;
            else modulation = 0;
        }
        brake->Set_brake_torque(modulation * max_torque);

        if (monitor && std::abs(wheel->GetWvel_loc().z) < 0.1) {
            GetLog() << "Wheel stopped at t = " << time << "\n";
            monitor = false;
        }
    }

    return 0;
}
