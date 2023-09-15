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
//  Demo code about collisions and contacts using a penalty (SMC) method
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

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

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

void AddFallingItems(ChSystemSMC& sys) {
    // Shared contact material for falling objects
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    for (int ix = -2; ix < 3; ix++) {
        for (int iz = -2; iz < 3; iz++) {
            // Spheres
            {
                double mass = 1;
                double radius = 1.1;
                auto body = chrono_types::make_shared<ChBody>(collision_type);
                body->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 2) * ChVector<>(1, 1, 1));
                body->SetMass(mass);
                body->SetPos(ChVector<>(4.0 * ix + 0.1, 4.0, 4.0 * iz));

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddSphere(mat, radius);
                body->GetCollisionModel()->BuildModel();
                body->SetCollide(true);

                auto sphere = chrono_types::make_shared<ChSphereShape>(radius);
                sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
                body->AddVisualShape(sphere);

                sys.AddBody(body);
            }

            // Boxes
            {
                double mass = 1;
                ChVector<> size(1.5, 1.5, 1.5);
                auto body = chrono_types::make_shared<ChBody>(collision_type);

                body->SetMass(mass);
                body->SetPos(ChVector<>(4.0 * ix, 6.0, 4.0 * iz));

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddBox(mat, size.x(), size.y(), size.z());
                body->GetCollisionModel()->BuildModel();
                body->SetCollide(true);

                auto box = chrono_types::make_shared<ChBoxShape>(size);
                box->SetTexture(GetChronoDataFile("textures/cubetexture_pinkwhite.png"));
                body->AddVisualShape(box);

                sys.AddBody(body);
            }
        }
    }
}

void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChMaterialSurface> mat,
                      std::shared_ptr<ChVisualMaterial> vis_mat,
                      const ChVector<>& size,
                      const ChVector<>& pos,
                      bool visible = true) {
    body->GetCollisionModel()->AddBox(mat, size.x(), size.y(), size.z(), pos);

    if (visible) {
        auto box = chrono_types::make_shared<ChBoxShape>(size);
        box->SetMaterial(0, vis_mat);
        body->AddVisualShape(box, ChFrame<>(pos, QUNIT));
    }
}

std::shared_ptr<ChBody> AddContainer(ChSystemSMC& sys) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>(collision_type);

    fixedBody->SetMass(1.0);
    fixedBody->SetBodyFixed(true);
    fixedBody->SetPos(ChVector<>());
    fixedBody->SetCollide(true);

    // Contact material for container
    auto fixed_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    auto fixed_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    fixed_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));

    fixedBody->GetCollisionModel()->ClearModel();
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(20, 1, 20), ChVector<>(0, -5, 0));
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(1, 10, 20.99), ChVector<>(-10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(1, 10, 20.99), ChVector<>(10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(20.99, 10, 1), ChVector<>(0, 0, -10), false);
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(20.99, 10, 1), ChVector<>(0, 0, 10));
    fixedBody->GetCollisionModel()->BuildModel();

    sys.AddBody(fixedBody);

    // The rotating mixer body
    auto rotatingBody = chrono_types::make_shared<ChBody>(collision_type);

    rotatingBody->SetMass(10.0);
    rotatingBody->SetInertiaXX(ChVector<>(50, 50, 50));
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->SetCollide(true);

    // Contact material for mixer body
    auto rot_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    ChVector<> size(10, 5.5, 1.0);

    rotatingBody->GetCollisionModel()->ClearModel();
    rotatingBody->GetCollisionModel()->AddBox(rot_mat, size.x(), size.y(), size.z());
    rotatingBody->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>(size);
    box->SetTexture(GetChronoDataFile("textures/blue.png"));
    rotatingBody->AddVisualShape(box);

    sys.AddBody(rotatingBody);

    // A motor between the two
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

    my_motor->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto mfun = chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2.0);  // speed w=90°/s
    my_motor->SetSpeedFunction(mfun);

    sys.AddLink(my_motor);

    return rotatingBody;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation and rendering time-step
    double time_step = 1e-4;
    double out_step = 1.0 / 20;

    // Create the physical system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(collision_type);

    // Add fixed and moving bodies
    auto mixer = AddContainer(sys);
    AddFallingItems(sys);

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
            vis_irr->SetWindowTitle("SMC collision demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(0, 18, -20));
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
            vis_vsg->SetWindowTitle("SMC callbacks");
            vis_vsg->AddCamera(ChVector<>(0, 18, -20));
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->SetUseSkyBox(true);  // use built-in path
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetWireFrameMode(false);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    double out_time = 0;

    while (vis->Run()) {
        sys.DoStepDynamics(time_step);

        double time = sys.GetChTime();
        if (time >= out_time) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            ////auto frc = mixer->GetAppliedForce();
            ////auto trq = mixer->GetAppliedTorque();
            ////std::cout << time << "  force: " << frc << "  torque: " << trq << std::endl;

            out_time += out_step;
        }
    }

    return 0;
}
