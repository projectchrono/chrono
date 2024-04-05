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

// Functor class implementing the torque for a ChLinkRSDA link.
class MySpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    MySpringTorque(double k, double c) : m_k(k), m_c(c) {}
    virtual double evaluate(double time,            // current time
                            double rest_angle,      // undeformed angle
                            double angle,           // relative angle of rotation
                            double vel,             // relative angular speed
                            const ChLinkRSDA& link  // associated link
                            ) override {
        double torque = -m_c * (angle - rest_angle) - m_k * vel;
        return torque;
    }
    double m_k;
    double m_c;
};

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // ChQuaternion<> rev_rot = QUNIT;
    ChQuaternion<> rev_rot = QuatFromAngleX(CH_PI / 6.0);
    // ChQuaternion<> rev_rot = QuatFromAngleX(CH_PI / 2.0);

    ChVector3d rev_dir = rev_rot.GetAxisZ();

    ChVector3d rev_pos(+1, 0, 0);

    // Create ground body
    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    // Visualization for revolute joint
    ChLineSegment seg(rev_pos + 0.2 * rev_dir, rev_pos - 0.2 * rev_dir);
    auto cyl_rev = chrono_types::make_shared<ChVisualShapeCylinder>(0.1, seg.GetLength());
    ground->AddVisualShape(cyl_rev, seg.GetFrame());

    // Offset from joint to body COM
    ChVector3d offset(1.5, 0, 0);

    // Consistent initial velocities
    double omega = 5.0;
    ChVector3d ang_vel = omega * rev_dir;
    ChVector3d lin_vel = Vcross(ang_vel, offset);

    // Create pendulum body
    auto body = chrono_types::make_shared<ChBody>();
    sys.AddBody(body);
    body->SetPos(rev_pos + offset);
    body->SetPosDt(lin_vel);
    body->SetAngVelParent(ang_vel);
    body->SetFixed(false);
    body->EnableCollision(false);
    body->SetMass(1);
    body->SetInertiaXX(ChVector3d(1, 1, 1));

    // Attach visualization assets
    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.3);
    sph->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    body->AddVisualShape(sph);

    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.1, 1.5);
    cyl->SetColor(ChColor(0.7f, 0.8f, 0.8f));
    body->AddVisualShape(cyl, ChFrame<>(ChVector3d(-0.75, 0, 0), QuatFromAngleY(CH_PI_2)));

    // Create revolute joint between body and ground
    auto rev = chrono_types::make_shared<ChLinkLockRevolute>();
    rev->Initialize(body, ground, ChFrame<>(rev_pos, rev_rot));
    sys.AddLink(rev);

    // Create the rotational spring between body and ground
    double spring_coef = 40;
    double damping_coef = 2;
    double rest_angle = CH_PI / 6;

    auto torque_functor = chrono_types::make_shared<MySpringTorque>(spring_coef, damping_coef);
    auto spring = chrono_types::make_shared<ChLinkRSDA>();
    spring->SetRestAngle(rest_angle);
    spring->Initialize(body, ground, ChFrame<>(rev_pos, rev_rot));
    spring->AddVisualShape(chrono_types::make_shared<ChVisualShapeRotSpring>(0.5, 100));
    spring->RegisterTorqueFunctor(torque_functor);
    sys.AddLink(spring);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkRSDA demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(3, 1, 3));
    vis->AddTypicalLights();
    vis->EnableBodyFrameDrawing(true);
    vis->EnableLinkFrameDrawing(true);

    // Simulation loop
    int frame = 0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(1e-3);

        if (frame % 50 == 0) {
            std::cout << sys.GetChTime() << "\n";
            std::cout << "Body position" << body->GetPos() << "\n";
            std::cout << "Body lin. vel." << body->GetPosDt() << "\n";
            std::cout << "Body absolute ang. vel." << body->GetAngVelParent() << "\n";
            std::cout << "Body local ang. vel." << body->GetAngVelLocal() << "\n";
            std::cout << "Rot. spring-damper  " << spring->GetAngle() << "  " << spring->GetVelocity() << "  "
                      << spring->GetTorque() << "\n";
            std::cout << "---------------\n\n";
        }

        frame++;
    }

    return 0;
}
