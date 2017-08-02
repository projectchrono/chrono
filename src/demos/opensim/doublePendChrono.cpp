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
// To demonstrate the use of ChBodyAuxRef, this simple example constructs two
// identical pendulums, one modeled as a ChBody, the other as a ChBodyAuxRef.
// The system is modeled in a (right-hand) frame with Y up.
//
// The two pendulums have length 2 and are pinned to ground through revolute
// joints with the rotation axis along the Z axis. The absolute locations of
// the revolute joints are at (0, 0, 1) and (0, 0, -1), respectively.
//
// The ChBody pendulum is defined with respect to a centroidal frame (as assumed
// by ChBody) located at the geometric center of the pendulum, with the X axis
// along the length of the pendulum.
// The ChBodyAuxRef pendulum is defined with respect to a local frame, parallel
// to its centroidal frame but located at the pin location.  In other words, the
// center of mass of the pendulum is at (1, 0, 0), relative to the body frame.
//
// The two pendulums move under the action of gravity, acting along the negative
// global Y direction.
//
// The ChBody pendulum is colored red. The ChBodyAuxRef pendulum is blue.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Create the ground body with two visualization cylinders
    // -------------------------------------------------------

    auto ground = std::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    {
        auto cyl_1 = std::make_shared<ChCylinderShape>();
        cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, 0, -.2);
        cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, 0, .2);
        cyl_1->GetCylinderGeometry().rad = 0.2;
        ground->AddAsset(cyl_1);
    }

    // Create a pendulum modeled using ChBody
    // --------------------------------------

    auto pend_1 = std::make_shared<ChBody>();
    system.AddBody(pend_1);
    pend_1->SetIdentifier(1);
    pend_1->SetBodyFixed(false);
    pend_1->SetCollide(false);
    pend_1->SetMass(1);
    pend_1->SetInertiaXX(ChVector<>(1, 1, 1));

    auto pend_2 = std::make_shared<ChBody>();
    system.AddBody(pend_2);
    pend_2->SetIdentifier(1);
    pend_2->SetBodyFixed(false);
    pend_2->SetCollide(false);
    pend_2->SetMass(1);
    pend_2->SetInertiaXX(ChVector<>(1, 1, 1));

    // Attach a visualization asset. Note that the cylinder is defined with
    // respect to the centroidal reference frame (which is the body reference
    // frame for a ChBody).
    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, .5, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_1->GetCylinderGeometry().rad = 0.1;
    pend_1->AddAsset(cyl_1);

    auto col_1 = std::make_shared<ChColorAsset>();
    col_1->SetColor(ChColor(0.6f, 0, 0));
    pend_1->AddAsset(col_1);

    auto joint_cyl = std::make_shared<ChCylinderShape>();
    joint_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, -.2);
    joint_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, .2);
    joint_cyl->GetCylinderGeometry().rad = 0.2;
    pend_1->AddAsset(joint_cyl);

    // Attach a visualization asset. Note that the cylinder is defined with
    // respect to the centroidal reference frame (which is the body reference
    // frame for a ChBody).
    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, .5, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
    cyl_2->GetCylinderGeometry().rad = 0.1;
    pend_2->AddAsset(cyl_1);

    auto col_2 = std::make_shared<ChColorAsset>();
    col_2->SetColor(ChColor(0, 0, .6f));
    pend_2->AddAsset(col_2);

    auto joint_cyl_2 = std::make_shared<ChCylinderShape>();
    joint_cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, 0, -.2);
    joint_cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, 0, .2);
    joint_cyl_2->GetCylinderGeometry().rad = 0.2;
    pend_2->AddAsset(joint_cyl_2);

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X). In this case, we set the absolute position of its center of
    // mass.
    pend_1->SetPos(ChVector<>(-0.353553, -0.353553, 0));
    pend_1->GetRot().Q_from_AngZ(-CH_C_PI / 4);
    pend_2->SetPos(ChVector<>(-0.836516, -0.224144, 0));
    pend_2->GetRot().Q_from_AngZ(-(CH_C_PI / 3) - (CH_C_PI / 4));

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_1 = std::make_shared<ChLinkLockRevolute>();
    rev_1->Initialize(ground, pend_1, true, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)),
                      ChCoordsys<>(ChVector<>(0, .5, 0), ChQuaternion<>(1, 0, 0, 0)));
    system.AddLink(rev_1);

    auto rev_2 = std::make_shared<ChLinkLockRevolute>();
    rev_2->Initialize(pend_1, pend_2, true, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)),
                      ChCoordsys<>(ChVector<>(0, .5, 0), ChQuaternion<>(1, 0, 0, 0)));
    system.AddLink(rev_2);

    // Create the Irrlicht application
    // -------------------------------

    ChIrrApp application(&system, L"ChBodyAuxRef demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    application.SetTimestep(0.001);

    bool log_info = true;

    while (application.GetDevice()->run()) {
        application.BeginScene();

        // auto bodies = system.Get_bodylist();
        // auto links = system.Get_linklist();
        // for (int i = 0; i < bodies->size(); ++i) {
        //     auto b = bodies->at(i);
        //     std::cout << b->GetName() << " is at " << b->GetPos().x() << "," << b->GetPos().y() << ","
        //               << b->GetPos().z() << " mass is " << b->GetMass() << std::endl;
        //     std::cout << b->GetRot().e0() << "," << b->GetRot().e1() << "," << b->GetRot().e2() << ","
        //               << b->GetRot().e3() << std::endl;
        // }
        // for (int i = 0; i < links->size(); ++i) {
        //     auto b = links->at(i);
        //     std::cout << b->GetName() << std::endl;
        // }

        application.DrawAll();

        application.DoStep();

        if (log_info && system.GetChTime() > 1.0) {
            // Note that GetPos() and similar functions will return information for
            // the centroidal frame for both pendulums:
            ChVector<> pos_1 = pend_1->GetPos();
            GetLog() << "t = " << system.GetChTime() << "\n";
            GetLog() << "      " << pos_1.x() << "  " << pos_1.y() << "\n";

            // OK, what about velocities? Here again, GetPos_dt() returns the linear
            // velocity of the body COG (expressed in the global frame) for both
            // pendulums:
            ChVector<> lin_vel_1 = pend_1->GetPos_dt();
            GetLog() << "      " << lin_vel_1.x() << "  " << lin_vel_1.y() << "\n";

            log_info = false;
        }

        application.EndScene();
    }

    return 0;
}
