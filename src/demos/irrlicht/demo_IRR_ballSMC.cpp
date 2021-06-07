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

void AddWall(std::shared_ptr<ChBody> body, const ChVector<>& dim, const ChVector<>& loc, std::shared_ptr<ChMaterialSurface> mat) {
    body->GetCollisionModel()->AddBox(mat, dim.x(), dim.y(), dim.z(), loc);

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = dim;
    box->GetBoxGeometry().Pos = loc;
    box->SetColor(ChColor(1, 0, 0));
    box->SetFading(0.6f);
    body->AddAsset(box);
}

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

    // Create the system
    ChSystemSMC msystem;

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    msystem.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    msystem.SetAdhesionForceModel(ChSystemSMC::AdhesionForceModel::Constant);

    msystem.Set_G_acc(ChVector<>(0, gravity, 0));

    // Change the default collision effective radius of curvature 
    collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);

    // Create the Irrlicht visualization
    ChIrrApp application(&msystem, L"SMC demo", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, -6));

    // This means that contactforces will be shown in Irrlicht application
    application.SetSymbolscale(1e-4);
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_FORCES);

    // Create a material (will be used by both objects)
    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetRestitution(0.1f);
    material->SetFriction(0.4f);
    material->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling ball
    auto ball = chrono_types::make_shared<ChBody>();

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
    ball->AddAsset(sphere);

    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddAsset(mtexture);

    msystem.AddBody(ball);

    // Create container
    auto bin = chrono_types::make_shared<ChBody>();

    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);

    bin->GetCollisionModel()->ClearModel();
    AddWall(bin, ChVector<>(width, thickness, length), ChVector<>(0, 0, 0), material);
    ////AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(-width + thickness, height, 0), material);
    ////AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(width - thickness, height, 0), material);
    ////AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, -length + thickness), material);
    ////AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, length - thickness), material);
    bin->GetCollisionModel()->BuildModel();

    msystem.AddBody(bin);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    // The soft-real-time cycle
    double time = 0.0;
    double out_time = 0.0;

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        tools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);

        while (time < out_time) {
            msystem.DoStepDynamics(time_step);
            time += time_step;
        }
        out_time += out_step;

        application.EndScene();
    }

    return 0;
}
