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
// To demonstrate the use of ChBodyAuxRef, this simple example constructs two
// identical pendulums, one modeled as a ChBody, the other as a ChBodyAuxRef.
// The sys is modeled in a (right-hand) frame with Y up.
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

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));

    // Create the ground body with two visualization cylinders
    // -------------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    // Create a pendulum modeled using ChBody
    // --------------------------------------

    auto pend_1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend_1);
    pend_1->SetFixed(false);
    pend_1->EnableCollision(false);
    pend_1->SetMass(1);
    pend_1->SetInertiaXX(ChVector3d(0.2, 1, 1));

    // Attach a visualization asset. Note that the cylinder is defined with
    // respect to the centroidal reference frame (which is the body reference
    // frame for a ChBody).
    {
        auto cyl_1 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2);
        cyl_1->SetColor(ChColor(0.6f, 0, 0));
        pend_1->AddVisualShape(cyl_1, ChFrame<>(ChVector3d(), QuatFromAngleY(CH_PI_2)));
    }

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X). In this case, we set the absolute position of its center of
    // mass.
    pend_1->SetPos(ChVector3d(1, 0, 1));

    // Create a revolute joint to connect pendulum to ground. 
    // Specify the link coordinate frame in the absolute frame.
    // Attach link visualization.
    auto rev_1 = chrono_types::make_shared<ChLinkLockRevolute>();
    sys.AddLink(rev_1);
    rev_1->Initialize(ground, pend_1, ChFrame<>(ChVector3d(0, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    {
        auto cyl_1 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.42);
        rev_1->AddVisualShape(cyl_1);
    }

    // Create a pendulum modeled using ChBodyAuxRef
    // --------------------------------------------

    auto pend_2 = chrono_types::make_shared<ChBodyAuxRef>();
    sys.Add(pend_2);
    pend_2->SetFixed(false);
    pend_2->EnableCollision(false);
    pend_2->SetMass(1);
    pend_2->SetInertiaXX(ChVector3d(0.2, 1, 1));
    // NOTE: the inertia tensor must still be expressed in the centroidal frame!

    // Attach a visualization asset. Note that now the cylinder is defined with
    // respect to the body reference frame.
    {
        auto cyl_2 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2);
        cyl_2->SetColor(ChColor(0, 0, 0.6f));
        pend_2->AddVisualShape(cyl_2, ChFrame<>(ChVector3d(1, 0, 0), QuatFromAngleY(CH_PI_2)));
    }

    // In this case, we must specify the centroidal frame, relative to the body
    // reference frame.
    pend_2->SetFrameCOMToRef(ChFrame<>(ChVector3d(1, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    // Specify the initial position of the pendulum (horizontal, pointing towards
    // positive X).  Here, we want to specify the position of the body reference
    // frame (relative to the absolute frame). Recall that the body reference
    // frame is located at the pin.
    pend_2->SetFrameRefToAbs(ChFrame<>(ChVector3d(0, 0, -1)));

    // Note: Beware of using the method SetPos() to specify the initial position
    // (as we did for the first pendulum)!  SetPos() specifies the position of the
    // centroidal frame.  So one could use it (instead of SetFrameRefToAbs) but
    // using:
    //   pend_2->SetPos(ChVector3d(1, 0, -1));
    // However, this defeats the goal of specifying the body through the desired
    // body reference frame.
    // Alternatively, one could use SetPos() and pass it the position of the body
    // reference frame; i.e.
    //   pend_2->SetPos(ChVector3d(0, 0, -1));
    // provided we do this BEFORE the call to SetFrameCOMToRef().
    //
    // This also applies to SetRot().

    // Create a revolute joint to connect pendulum to ground. 
    // Specify the link coordinate frame in the absolute frame.
    // Attach link visualization.
    auto rev_2 = chrono_types::make_shared<ChLinkLockRevolute>();
    sys.AddLink(rev_2);
    rev_2->Initialize(ground, pend_2, ChFrame<>(ChVector3d(0, 0, -1), ChQuaternion<>(1, 0, 0, 0)));
    {
        auto cyl_2 = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 0.42);
        rev_2->AddVisualShape(cyl_2);
    }

    // Create the Irrlicht application
    // -------------------------------

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("ChBodyAuxRef demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 3, 6));
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("ChBodyAuxRef demo");
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->EnableSkyBox();
            vis_vsg->AddCamera(ChVector3d(0, 3, 6));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    bool log_info = true;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(1e-3);

        if (log_info && sys.GetChTime() > 1.0) {
            // Note that GetPos() and similar functions will return information for
            // the centroidal frame for both pendulums:
            ChVector3d pos_1 = pend_1->GetPos();
            ChVector3d pos_2 = pend_2->GetPos();
            std::cout << "t = " << sys.GetChTime() << "\n";
            std::cout << "      " << pos_1.x() << "  " << pos_1.y() << "\n";
            std::cout << "      " << pos_2.x() << "  " << pos_2.y() << "\n";

            // But it's quite likely that, for the second pendulum, what we want is
            // the position of the body reference frame.  This is available with:
            ChFrame<> frame_2 = pend_2->GetFrameRefToAbs();
            pos_2 = frame_2.GetPos();
            std::cout << "      " << pos_2.x() << "  " << pos_2.y() << "\n";

            // OK, what about velocities? Here again, GetPosDt() returns the linear
            // velocity of the body COG (expressed in the global frame) for both
            // pendulums:
            ChVector3d lin_vel_1 = pend_1->GetPosDt();
            ChVector3d lin_vel_2 = pend_2->GetPosDt();
            std::cout << "      " << lin_vel_1.x() << "  " << lin_vel_1.y() << "\n";
            std::cout << "      " << lin_vel_2.x() << "  " << lin_vel_2.y() << "\n";

            // To obtain the absolute linear velocity of the body reference frame,
            // we use again GetPosDt(), but this time for the reference frame,
            // using GetFrameRefToAbs() similarly for what we did for positions:
            lin_vel_2 = pend_2->GetFrameRefToAbs().GetPosDt();
            std::cout << "      " << lin_vel_2.x() << "  " << lin_vel_2.y() << "\n";

            log_info = false;
        }
    }

    return 0;
}
