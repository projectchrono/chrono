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
// Simple example demonstrating the use of ChLinkRSDA.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================

double spring_coef = 40;
double damping_coef = 2;
double rest_angle = CH_C_PI / 6;

// =============================================================================

// Functor class implementing the torque for a ChLinkRSDA link.
class MySpringTorque : public ChLinkRSDA::TorqueFunctor {
    virtual double evaluate(double time,            // current time
                            double angle,           // relative angle of rotation
                            double vel,             // relative angular speed
                            const ChLinkRSDA& link  // associated link
                            ) override {
        double torque = -spring_coef * (angle - rest_angle) - damping_coef * vel;
        return torque;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, 0));

    // ChQuaternion<> rev_rot = QUNIT;
    ChQuaternion<> rev_rot = Q_from_AngX(CH_C_PI / 6.0);
    // ChQuaternion<> rev_rot = Q_from_AngX(CH_C_PI / 2.0);

    ChVector<> rev_dir = rev_rot.GetZaxis();

    ChVector<> rev_pos(+1, 0, 0);

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    // Visualization for revolute joint
    auto cyl_rev = chrono_types::make_shared<ChCylinderShape>();
    cyl_rev->GetCylinderGeometry().p1 = rev_pos + 0.2 * rev_dir;
    cyl_rev->GetCylinderGeometry().p2 = rev_pos - 0.2 * rev_dir;
    cyl_rev->GetCylinderGeometry().rad = 0.1;
    ground->AddVisualShape(cyl_rev);

    // Offset from joint to body COM
    ChVector<> offset(1.5, 0, 0);

    // Consistent initial velocities
    double omega = 5.0;
    ChVector<> ang_vel = omega * rev_dir;
    ChVector<> lin_vel = Vcross(ang_vel, offset);

    // Create pendulum body
    auto body = chrono_types::make_shared<ChBody>();
    sys.AddBody(body);
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
    sph->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    body->AddVisualShape(sph);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(-1.5, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl->GetCylinderGeometry().rad = 0.1;
    cyl->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    body->AddVisualShape(cyl);

    // Create revolute joint between body and ground
    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    rev->Initialize(body, ground, ChCoordsys<>(rev_pos, rev_rot));
    sys.AddLink(rev);

    // Create the rotational spring between body and ground
    auto torque = chrono_types::make_shared<MySpringTorque>();
    auto spring = chrono_types::make_shared<ChLinkRSDA>();
    spring->Initialize(body, ground, ChCoordsys<>(rev_pos, rev_rot));
    spring->AddVisualShape(chrono_types::make_shared<ChRotSpringShape>(0.5, 100));
    spring->RegisterTorqueFunctor(torque);
    sys.AddLink(spring);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkRSDA demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(3, 1, 3));
    vis->AddTypicalLights();
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    int frame = 0;

    GetLog().SetNumFormat("%10.3f");

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(1e-3);

        if (frame % 50 == 0) {
            GetLog() << sys.GetChTime() << "\n";
            GetLog() << "Body position" << body->GetPos() << "\n";
            GetLog() << "Body lin. vel." << body->GetPos_dt() << "\n";
            GetLog() << "Body absolute ang. vel." << body->GetWvel_par() << "\n";
            GetLog() << "Body local ang. vel." << body->GetWvel_loc() << "\n";
            GetLog() << "Rot. spring-damper  " << spring->GetAngle() << "  " << spring->GetVelocity() << "  "
                     << spring->GetTorque() << "\n";
            GetLog() << "---------------\n\n";
        }

        frame++;
    }

    return 0;
}
