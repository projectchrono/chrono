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
// Authors: Wing To Ku (FAU Erlangen), Radu Serban
// =============================================================================
//
// Spinning tops demo (use SI units)
// 
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChTexture.h"

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

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Top material density (kg/m3)
double density = 7000;

// Top angular velocities (rad/s)
double omega1 = 50;
double omega2 = 50;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);
    double mesh_thickness = 5e-3;

    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_mat->SetRestitution(0.4f);
    contact_mat->SetStaticFriction(0.0f);

    // Available meshes: spinning_top_1.obj, spinning_top_2.obj, spinning_top_3.obj, spinning_top_4.obj
    std::string top1_mesh = GetChronoDataFile("models/spinning_top/spinning_top_1.obj");
    std::string top2_mesh = GetChronoDataFile("models/spinning_top/spinning_top_2.obj");

    // NOTES:
    // - Each body mesh is defined relative to a mesh with Y up.
    // - Desired angular velocity is about the top axis (Y axis of the body reference frame).
    // - Attention! SetAngVelLocal sets the angular velocity as expressed in the body centroidal frame.
    // - ChBodyEasyMesh constructor calculates the centroidal frame relative to the body reference frame and this may
    //   involve an unknown rotation; as such, we cannot use SetAngVelLocal.
    // Instead, do the following:
    // - Rotate the body reference frame by pi/2 about X axis to align Y axis of body reference frame with world
    //   vertical.
    // - Transform desired angular velocity into global frame and use SetAngVelParent; this will calculate the necessary
    //   angular velocity expressed in the centroidal frame (which is what Chrono uses for simulation).

    auto top1 = chrono_types::make_shared<ChBodyEasyMesh>(top1_mesh,                                 //
                                                          density, true,                             //
                                                          true, true, contact_mat, mesh_thickness);  //
    ChFramed X_G1 = ChFramed(ChVector3d(3, 0, 0), QuatFromAngleX(CH_PI_2));
    top1->SetFrameRefToAbs(X_G1);
    top1->SetAngVelParent(X_G1 * ChVector3d(0, omega1, 0));
    top1->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/metal.jpg"));
    sys.Add(top1);

    auto top2 = chrono_types::make_shared<ChBodyEasyMesh>(top2_mesh,                                 //
                                                          density, true,                             //
                                                          true, true, contact_mat, mesh_thickness);  //
    ChFramed X_G2 = ChFramed(ChVector3d(-3, 0, 0), QuatFromAngleX(CH_PI_2));
    top2->SetFrameRefToAbs(X_G2);
    top2->SetAngVelParent(X_G2 * ChVector3d(0, omega2, 0));
    top2->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bronze.png"));
    sys.Add(top2);

    auto platform = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/spinning_top/platform.obj"),  //
                                                              1, false,                                               //
                                                              true, true, contact_mat, mesh_thickness);               //
    platform->SetFixed(true);
    platform->SetPos(ChVector3d(0, 0, -3));
    platform->GetVisualShape(0)->SetColor(ChColor(182 / 255.0f, 142 / 255.0f, 101 / 255.0f));
    sys.Add(platform);

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
            vis_irr->SetWindowTitle("Spinning tops");
            vis_irr->SetWindowSize(1280, 800);
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector3d(0, -5, 3), ChVector3d(0, 0, 0));

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("Spinning tops");
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->AddCamera(ChVector3d(0, -15, 3), ChVector3d(0, 0, 0));
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis_vsg->ToggleRefFrameVisibility();
            vis_vsg->SetRefFrameScale(2.0);
            vis_vsg->SetContactNormalsVisibility(true);

            vis = vis_vsg;
#endif
            break;
        }
    }

    double timestep = 5e-4;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
