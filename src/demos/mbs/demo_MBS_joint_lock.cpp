// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Demonstration of the locking and releasing a joint (link-lock formulation)
//
// =============================================================================

#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"

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

// -----------------------------------------------------------------------------

enum class JointType { REVOLUTE, SPHERICAL, PRISMATIC };

JointType joint_type = JointType::SPHERICAL;
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, +1, -1));

    // Create the ground body
    // ----------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    // Create a pendulum body
    // ----------------------

    auto pend = chrono_types::make_shared<ChBody>();
    sys.AddBody(pend);
    pend->SetFixed(false);
    pend->EnableCollision(false);
    pend->SetMass(1);
    pend->SetInertiaXX(ChVector3d(0.2, 1, 1));

    // Initial position of the pendulum (horizontal, pointing towards positive X).
    pend->SetPos(ChVector3d(1, 0, 0));

    // Attach visualization assets.
    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2.0);
    cyl->SetTexture(GetChronoDataFile("textures/checker1.png"), 1, 10);
    pend->AddVisualShape(cyl, ChFramed(VNULL, Q_ROTATE_Z_TO_X));

    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.2);
    sph->SetColor(ChColor(1.0f, 1.0f, 0.8f));
    pend->AddVisualShape(sph, ChFramed(ChVector3d(-1, 0, 0), QUNIT));

    // Create and initialize the joint
    // -------------------------------

    std::shared_ptr<ChLinkLock> joint;
    switch (joint_type) {
        case JointType::REVOLUTE:
            joint = chrono_types::make_shared<ChLinkLockRevolute>();
            joint->Initialize(ground, pend, ChFramed(VNULL, Q_ROTATE_Z_TO_Y));
            break;
        case JointType::SPHERICAL:
            joint = chrono_types::make_shared<ChLinkLockSpherical>();
            joint->Initialize(ground, pend, ChFramed(VNULL, QUNIT));
            break;
        case JointType::PRISMATIC:
            joint = chrono_types::make_shared<ChLinkLockPrismatic>();
            joint->Initialize(ground, pend, ChFramed(VNULL, Q_ROTATE_Z_TO_Y));
            break;
    }

    sys.AddLink(joint);

    // Create the run-time visualization system
    // ----------------------------------------

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
            vis_irr->SetWindowTitle("Locking revolute joint");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, -4, 0));
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
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetWindowTitle("Locking revolute joint");
            vis_vsg->SetBackgroundColor(ChColor(18.0f / 255, 26.0f / 255, 32.0f / 255));
            vis_vsg->AddCamera(ChVector3d(0, -4, 0));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->Initialize();
            vis_vsg->ToggleAbsFrameVisibility();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    // ---------------

    ChRealtimeStepTimer rt_timer;
    double step = 1e-2;
    bool locked = false;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Lock/unlock the joint every 2 seconds
        double t = sys.GetChTime();
        if (std::abs(std::remainder(t, 2.0)) < step / 2 && t > step) {
            locked = !locked;
            joint->Lock(locked);
        }

        sys.DoStepDynamics(step);
        rt_timer.Spin(step);
    }

    return 0;
}
