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
// Simple example demonstrating the use of ChLinkRotSpringCB.
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

// =============================================================================

double spring_coef = 40;
double damping_coef = 2;
double rest_angle = CH_C_PI / 6;

// =============================================================================

// Functor class implementing the torque for a ChLinkRotSpringCB link.
class MySpringTorque : public ChLinkRotSpringCB::TorqueFunctor {
    virtual double operator()(double time,             // current time
                              double angle,            // relative angle of rotation
                              double vel,              // relative angular speed
                              ChLinkRotSpringCB* link  // back-pointer to associated link
                              ) override {
        double torque = -spring_coef * (angle - rest_angle) - damping_coef * vel;
        return torque;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    //ChQuaternion<> rev_rot = QUNIT;
    ChQuaternion<> rev_rot = Q_from_AngX(CH_C_PI / 6.0);
    //ChQuaternion<> rev_rot = Q_from_AngX(CH_C_PI / 2.0);

    ChVector<> rev_dir = rev_rot.GetZaxis();

    ChVector<> rev_pos(+1, 0, 0);

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    // Visualization for revolute joint
    auto cyl_rev = chrono_types::make_shared<ChCylinderShape>();
    cyl_rev->GetCylinderGeometry().p1 = rev_pos + 0.2 * rev_dir;
    cyl_rev->GetCylinderGeometry().p2 = rev_pos - 0.2 * rev_dir;
    cyl_rev->GetCylinderGeometry().rad = 0.1;
    ground->AddAsset(cyl_rev);

    // Offset from joint to body COM
    ChVector<> offset(1.5, 0, 0);

    // Consistent initial velocities
    double omega = 5.0;
    ChVector<> ang_vel = omega * rev_dir;
    ChVector<> lin_vel = Vcross(ang_vel, offset);

    // Create pendulum body
    auto body = chrono_types::make_shared<ChBody>();
    system.AddBody(body);
    body->SetPos(rev_pos + offset);
    body->SetPos_dt(lin_vel);
    body->SetWvel_par(ang_vel);
    body->SetIdentifier(1);
    body->SetBodyFixed(false);
    body->SetCollide(false);
    body->SetMass(1);
    body->SetInertiaXX(ChVector<>(1, 1, 1));

    // Attach visualization assets
    auto sph = chrono_types::make_shared<ChSphereShape>();
    sph->GetSphereGeometry().rad = 0.3;
    body->AddAsset(sph);
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(-1.5, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl->GetCylinderGeometry().rad = 0.1;
    body->AddAsset(cyl);
    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    body->AddAsset(col);

    // Create revolute joint between body and ground
    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    rev->Initialize(body, ground, ChCoordsys<>(rev_pos, rev_rot));
    system.AddLink(rev);

    // Create the rotational spring between body and ground
    auto torque = chrono_types::make_shared<MySpringTorque>();
    auto spring = chrono_types::make_shared<ChLinkRotSpringCB>();
    spring->Initialize(body, ground, ChCoordsys<>(rev_pos, rev_rot));
    spring->RegisterTorqueFunctor(torque);
    system.AddLink(spring);

    // Create the Irrlicht application
    ChIrrApp application(&system, L"ChLinkRotSpringCB demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(3, 1, 3));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    int frame = 0;

    application.SetTimestep(0.001);

    GetLog().SetNumFormat("%10.3f");

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        ChIrrTools::drawAllCOGs(system, application.GetVideoDriver(), 1.0);
        ChIrrTools::drawAllLinkframes(system, application.GetVideoDriver(), 1.5);
        application.EndScene();
        application.DoStep();

        if (frame % 50 == 0) {
            GetLog() << system.GetChTime() << "\n";
            GetLog() << "Body position" << body->GetPos() << "\n";
            GetLog() << "Body lin. vel." << body->GetPos_dt() << "\n";
            GetLog() << "Body absolute ang. vel." << body->GetWvel_par() << "\n";
            GetLog() << "Body local ang. vel." << body->GetWvel_loc() << "\n";
            GetLog() << "Rot. spring-damper  " << spring->GetRotSpringAngle() << "  " << spring->GetRotSpringSpeed()
                     << "  " << spring->GetRotSpringTorque() << "\n";
            GetLog() << "---------------\n\n";
        }

        frame++;
    }

    return 0;
}
