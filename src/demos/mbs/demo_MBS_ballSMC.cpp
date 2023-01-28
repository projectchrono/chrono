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
// Demo code about collisions and contacts using the penalty method (SMC)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation parameters
    double gravity = -9.81;
    double time_step = 0.00001;
    double out_step = 2000 * time_step;

    // Parameters for the falling ball
    int ballId = 100;
    double radius = 1;
    double mass = 1000;
    ChVector<> pos(0, 2, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);

    // Parameters for the containing bin
    int binId = 200;
    double width = 2;
    double length = 2;
    ////double height = 1;
    double thickness = 0.1;

    // Collision system type
    auto collision_type = collision::ChCollisionSystemType::BULLET;

    // Create the system
    ChSystemSMC sys;

    sys.Set_G_acc(ChVector<>(0, gravity, 0));
    sys.SetCollisionSystemType(collision_type);

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    sys.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);

    // Change the default collision effective radius of curvature
    collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);

    // Create a material (will be used by both objects)
    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetRestitution(0.1f);
    material->SetFriction(0.4f);
    material->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling ball
    auto ball = chrono_types::make_shared<ChBody>(collision_type);

    ball->SetIdentifier(ballId);
    ball->SetMass(mass);
    ball->SetPos(pos);
    ball->SetRot(rot);
    ball->SetPos_dt(init_vel);
    // ball->SetWvel_par(ChVector<>(0,0,3));
    ball->SetBodyFixed(false);

    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(material, radius);
    ball->GetCollisionModel()->BuildModel();

    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));

    auto sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = radius;
    sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sphere->SetOpacity(1.0f);

    auto ball_vis = chrono_types::make_shared<ChVisualModel>();
    ball_vis->AddShape(sphere);
    ball->AddVisualModel(ball_vis);

    sys.AddBody(ball);

    // Create container
    auto bin = chrono_types::make_shared<ChBody>(collision_type);

    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    bin->GetCollisionModel()->ClearModel();
    bin->GetCollisionModel()->AddBox(material, width, thickness, length);
    bin->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(width, thickness, length);
    box->SetColor(ChColor(0.8f, 0.2f, 0.2f));
    box->SetOpacity(0.8f);

    auto bin_vis = chrono_types::make_shared<ChVisualModel>();
    bin_vis->AddShape(box, ChFrame<>());
    bin->AddVisualModel(bin_vis);

    sys.AddBody(bin);

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("SMC demonstration");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddTypicalLights();
            vis_irr->AddCamera(ChVector<>(0, 3, -6));
            vis_irr->AttachSystem(&sys);

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("SMC demonstration");
            vis_vsg->AddCamera(ChVector<>(0, 3, -6));
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetWireFrameMode(false);
            vis_vsg->AddGrid(0.2, 0.2, 20, 20, ChCoordsys<>(ChVector<>(0, 0.11, 0), Q_from_AngX(CH_C_PI_2)),
                             ChColor(0.1f, 0.1f, 0.1f));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // The soft-real-time cycle
    double time = 0.0;
    double out_time = 0.0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->RenderGrid(ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)), 20, 0.2);
        vis->RenderFrame(ChFrame<>(ball->GetCoord()), 1.2 * radius);
        vis->EndScene();

        while (time < out_time) {
            sys.DoStepDynamics(time_step);
            time += time_step;
        }
        out_time += out_step;
    }

    return 0;
}
