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

#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

void AddFallingItems(ChSystemSMC& sys) {
    // Shared contact material for falling objects
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    for (int ix = -2; ix < 3; ix++) {
        for (int iz = -2; iz < 3; iz++) {
            // Spheres
            {
                double mass = 1;
                double radius = 1.1;
                auto body = chrono_types::make_shared<ChBody>();
                body->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 2) * ChVector<>(1, 1, 1));
                body->SetMass(mass);
                body->SetPos(ChVector<>(4.0 * ix + 0.1, 4.0, 4.0 * iz));

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddSphere(mat, radius);
                body->GetCollisionModel()->BuildModel();
                body->SetCollide(true);

                auto sphere = chrono_types::make_shared<ChSphereShape>();
                sphere->GetSphereGeometry().rad = radius;
                body->AddAsset(sphere);

                auto texture = chrono_types::make_shared<ChTexture>();
                texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
                body->AddAsset(texture);

                sys.AddBody(body);
            }

            // Boxes
            {
                double mass = 1;
                ChVector<> hsize(0.75, 0.75, 0.75);
                auto body = chrono_types::make_shared<ChBody>();

                body->SetMass(mass);
                body->SetPos(ChVector<>(4.0 * ix, 6.0, 4.0 * iz));

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z());
                body->GetCollisionModel()->BuildModel();
                body->SetCollide(true);

                auto box = chrono_types::make_shared<ChBoxShape>();
                box->GetBoxGeometry().Size = hsize;
                body->AddAsset(box);

                auto texture = chrono_types::make_shared<ChTexture>();
                texture->SetTextureFilename(GetChronoDataFile("pinkwhite.png"));
                body->AddAsset(texture);

                sys.AddBody(body);
            }
        }
    }
}

void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChMaterialSurface> mat,
                      const ChVector<>& size,
                      const ChVector<>& pos,
                      bool visible = true) {
    ChVector<> hsize = 0.5 * size;

    body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z(), pos);

    if (visible) {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Pos = pos;
        box->GetBoxGeometry().Size = hsize;
        body->AddAsset(box);
    }
}

std::shared_ptr<ChBody> AddContainer(ChSystemSMC& sys) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>();

    fixedBody->SetMass(1.0);
    fixedBody->SetBodyFixed(true);
    fixedBody->SetPos(ChVector<>());
    fixedBody->SetCollide(true);

    // Contact material for container
    auto fixed_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    fixedBody->GetCollisionModel()->ClearModel();
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(20, 1, 20), ChVector<>(0, -5, 0));
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(1, 10, 20.99), ChVector<>(-10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(1, 10, 20.99), ChVector<>(10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(20.99, 10, 1), ChVector<>(0, 0, -10), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(20.99, 10, 1), ChVector<>(0, 0, 10));
    fixedBody->GetCollisionModel()->BuildModel();

    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    fixedBody->AddAsset(texture);

    sys.AddBody(fixedBody);

    // The rotating mixer body
    auto rotatingBody = chrono_types::make_shared<ChBody>();

    rotatingBody->SetMass(10.0);
    rotatingBody->SetInertiaXX(ChVector<>(50, 50, 50));
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->SetCollide(true);

    // Contact material for mixer body
    auto rot_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    ChVector<> hsize(5, 2.75, 0.5);

    rotatingBody->GetCollisionModel()->ClearModel();
    rotatingBody->GetCollisionModel()->AddBox(rot_mat, hsize.x(), hsize.y(), hsize.z());
    rotatingBody->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = hsize;
    rotatingBody->AddAsset(box);

    rotatingBody->AddAsset(texture);

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

    // Create the Irrlicht visualization
    ChIrrApp application(&sys, L"SMC collision demo", core::dimension2d<u32>(800, 600), false, true);

    // Add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 18, -20));

    // Add fixed and moving bodies
    auto mixer = AddContainer(sys);
    AddFallingItems(sys);

    // Complete asset specification: convert all assets to Irrlicht
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    double time = 0;
    double out_time = 0;

    while (application.GetDevice()->run()) {
        sys.DoStepDynamics(time_step);

        double time = sys.GetChTime();
        if (time >= out_time) {
            application.BeginScene();
            application.DrawAll();
            application.EndScene();

            ////auto frc = mixer->GetAppliedForce();
            ////auto trq = mixer->GetAppliedTorque();
            ////std::cout << time << "  force: " << frc << "  torque: " << trq << std::endl;

            out_time += out_step;
        }
    }

    return 0;
}
