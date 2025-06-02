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
// Simple demo for contact between a ball and a plate, using NSC or SMC contact
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
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

ChContactMethod contact_method = ChContactMethod::SMC;
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;
ChCollisionSystem::Type coll_type = ChCollisionSystem::Type::BULLET;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation parameters
    double gravity = -9.81;
    double time_step = contact_method == ChContactMethod::NSC ? 1e-3 : 1e-5;
    double render_fps = 100;

    // Parameters for the falling ball
    double radius = 1;
    double mass = 1000;
    ChVector3d pos(0, 2, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector3d init_vel(0, 0, 0);

    // Parameters for the containing bin
    double width = 2;
    double length = 2;
    double thickness = 0.1;

    // Create the system
    auto sys = ChSystem::Create(contact_method);

    sys->SetGravitationalAcceleration(ChVector3d(0, gravity, 0));
    sys->SetCollisionSystemType(coll_type);

    // Change the default collision effective radius of curvature (SMC only)
    ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);

    // Create a material (will be used by both objects)
    ChContactMaterialData mat_data;
    mat_data.mu = 0.4f;
    mat_data.cr = 0.1f;
    auto material = mat_data.CreateMaterial(contact_method);

    // Create the falling ball
    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
    ball->SetPos(pos);
    ball->SetRot(rot);
    ball->SetPosDt(init_vel);
    // ball->SetAngVelParent(ChVector3d(0,0,3));
    ball->SetFixed(false);

    auto sphere_coll = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
    ball->AddCollisionShape(sphere_coll, ChFrame<>());
    ball->EnableCollision(true);

    auto sphere_vis = chrono_types::make_shared<ChVisualShapeSphere>(radius);
    sphere_vis->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sphere_vis->SetOpacity(1.0f);
    ball->AddVisualShape(sphere_vis);

    sys->AddBody(ball);

    // Create container
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetMass(1);
    bin->SetPos(ChVector3d(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetFixed(true);

    auto box_coll = chrono_types::make_shared<ChCollisionShapeBox>(material, width * 2, thickness * 2, length * 2);
    bin->AddCollisionShape(box_coll, ChFrame<>());
    bin->EnableCollision(true);

    auto box_vis = chrono_types::make_shared<ChVisualShapeBox>(width * 2, thickness * 2, length * 2);
    box_vis->SetColor(ChColor(0.8f, 0.2f, 0.2f));
    box_vis->SetOpacity(0.8f);
    bin->AddVisualShape(box_vis);

    sys->AddBody(bin);

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
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Ball drop demonstration");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector3d(0, 3, -6));
            vis_irr->AttachSystem(sys.get());
            vis_irr->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector3d(0, 0.11, 0), QuatFromAngleX(CH_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys.get());
            vis_vsg->SetWindowTitle("Ball drop demonstration");
            vis_vsg->AddCamera(ChVector3d(0, 3, -6));
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector3d(0, 0.11, 0), QuatFromAngleX(CH_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    ChRealtimeStepTimer rt_timer;
    double time = 0.0;
    int render_frame = 0;

    while (vis->Run()) {
        if (time > render_frame / render_fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }

        sys->DoStepDynamics(time_step);
        rt_timer.Spin(time_step);
        time += time_step;
    }

    return 0;
}
