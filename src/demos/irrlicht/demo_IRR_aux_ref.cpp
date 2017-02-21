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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    ChSystem system;
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
        cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, 0, 1.2);
        cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0.8);
        cyl_1->GetCylinderGeometry().rad = 0.2;
        ground->AddAsset(cyl_1);

        auto cyl_2 = std::make_shared<ChCylinderShape>();
        cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, 0, -1.2);
        cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, 0, -0.8);
        cyl_2->GetCylinderGeometry().rad = 0.2;
        ground->AddAsset(cyl_2);
    }

    // Create a pendulum modeled using ChBody
    // --------------------------------------

    auto pend_1 = std::make_shared<ChBody>();
    system.AddBody(pend_1);
    pend_1->SetIdentifier(1);
    pend_1->SetBodyFixed(false);
    pend_1->SetCollide(false);
    pend_1->SetMass(1);
    pend_1->SetInertiaXX(ChVector<>(0.2, 1, 1));

    // Attach a visualization asset. Note that the cylinder is defined with
    // respect to the centroidal reference frame (which is the body reference
    // frame for a ChBody).
    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = ChVector<>(-1, 0, 0);
    cyl_1->GetCylinderGeometry().p2 = ChVector<>(1, 0, 0);
    cyl_1->GetCylinderGeometry().rad = 0.2;
    pend_1->AddAsset(cyl_1);
    auto col_1 = std::make_shared<ChColorAsset>();
    col_1->SetColor(ChColor(0.6f, 0, 0));
    pend_1->AddAsset(col_1);

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X). In this case, we set the absolute position of its center of
    // mass.
    pend_1->SetPos(ChVector<>(1, 0, 1));

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_1 = std::make_shared<ChLinkLockRevolute>();
    rev_1->Initialize(ground, pend_1, ChCoordsys<>(ChVector<>(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    system.AddLink(rev_1);

    // Create a pendulum modeled using ChBodyAuxRef
    // --------------------------------------------

    auto pend_2 = std::make_shared<ChBodyAuxRef>();
    system.Add(pend_2);
    pend_2->SetIdentifier(2);
    pend_2->SetBodyFixed(false);
    pend_2->SetCollide(false);
    pend_2->SetMass(1);
    pend_2->SetInertiaXX(ChVector<>(0.2, 1, 1));
    // NOTE: the inertia tensor must still be expressed in the centroidal frame!

    // Attach a visualization asset. Note that now the cylinder is defined with
    // respect to the body reference frame.
    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    cyl_2->GetCylinderGeometry().p2 = ChVector<>(2, 0, 0);
    cyl_2->GetCylinderGeometry().rad = 0.2;
    pend_2->AddAsset(cyl_2);
    auto col_2 = std::make_shared<ChColorAsset>();
    col_2->SetColor(ChColor(0, 0, 0.6f));
    pend_2->AddAsset(col_2);

    // In this case, we must specify the centroidal frame, relative to the body
    // reference frame.
    pend_2->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(1, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X).  Here, we want to specify the position of the body reference
    // frame (relative to the absolute frame). Recall that the body reference
    // frame is located at the pin.
    pend_2->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(0, 0, -1)));

    // Note: Beware of using the method SetPos() to specify the initial position
    // (as we did for the first pendulum)!  SetPos() specifies the position of the
    // centroidal frame.  So one could use it (instead of SetFrame_REF_to_abs) but
    // using:
    //   pend_2->SetPos(ChVector<>(1, 0, -1));
    // However, this defeats the goal of specifying the body through the desired
    // body reference frame.
    // Alternatively, one could use SetPos() and pass it the position of the body
    // reference frame; i.e.
    //   pend_2->SetPos(ChVector<>(0, 0, -1));
    // provided we do this BEFORE the call to SetFrame_COG_to_REF().
    //
    // This also applies to SetRot().

    // Create a revolute joint to connect pendulum to ground. We specify the link
    // coordinate frame in the absolute frame.
    auto rev_2 = std::make_shared<ChLinkLockRevolute>();
    rev_2->Initialize(ground, pend_2, ChCoordsys<>(ChVector<>(0, 0, -1), ChQuaternion<>(1, 0, 0, 0)));
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

        application.DrawAll();

        application.DoStep();

        if (log_info && system.GetChTime() > 1.0) {
            // Note that GetPos() and similar functions will return information for
            // the centroidal frame for both pendulums:
            ChVector<> pos_1 = pend_1->GetPos();
            ChVector<> pos_2 = pend_2->GetPos();
            GetLog() << "t = " << system.GetChTime() << "\n";
            GetLog() << "      " << pos_1.x() << "  " << pos_1.y() << "\n";
            GetLog() << "      " << pos_2.x() << "  " << pos_2.y() << "\n";

            // But it's quite likely that, for the second pendulum, waht we want is
            // the position of the body reference frame.  This is available with:
            ChFrame<> frame_2 = pend_2->GetFrame_REF_to_abs();
            pos_2 = frame_2.GetPos();
            GetLog() << "      " << pos_2.x() << "  " << pos_2.y() << "\n";

            // OK, what about velocities? Here again, GetPos_dt() returns the linear
            // velocity of the body COG (expressed in the global frame) for both
            // pendulums:
            ChVector<> lin_vel_1 = pend_1->GetPos_dt();
            ChVector<> lin_vel_2 = pend_2->GetPos_dt();
            GetLog() << "      " << lin_vel_1.x() << "  " << lin_vel_1.y() << "\n";
            GetLog() << "      " << lin_vel_2.x() << "  " << lin_vel_2.y() << "\n";

            // To obtain the absolute linear velocity of the body reference frame,
            // we use again GetPos_dt(), but this time for the reference frame,
            // using GetFrame_REF_to_abs() similarly for what we did for positions:
            lin_vel_2 = pend_2->GetFrame_REF_to_abs().GetPos_dt();
            GetLog() << "      " << lin_vel_2.x() << "  " << lin_vel_2.y() << "\n";

            log_info = false;
        }

        application.EndScene();
    }

    return 0;
}
