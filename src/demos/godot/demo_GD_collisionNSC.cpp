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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
//     - collisions and contacts
//     - using Irrlicht to display objects.
//
// =============================================================================
#include "chrono/core/ChVector.h"
// extern chrono::ChVector<> VNULL = {0, 0, 0};
namespace chrono {
const ChVector<double> VNULL(0., 0., 0.);
}

#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_godot/ChGodotApp.h"

#ifdef WINDOWS_ENABLED
extern HINSTANCE godot_hinstance =
    NULL;  // this is horrible, but Asher does not understand the external symbol error caused by not having this
#endif
// Use the namespaces of Chrono
// using namespace chrono;
// using namespace chrono::godot;

void create_some_falling_items(chrono::ChSystemNSC& mphysicalSystem, int num_bodies) {
    for (int bi = 0; bi < num_bodies; bi++) {
        auto msphereBody = std::make_shared<chrono::ChBodyEasySphere>(1.1,    // radius size
                                                                      1000,   // density
                                                                      true,   // collide enable?
                                                                      true);  // visualization?
        msphereBody->SetPos(
            chrono::ChVector<>(-5 + chrono::ChRandom() * 10, 4 + bi * 0.05, -5 + chrono::ChRandom() * 10));
        msphereBody->GetMaterialSurfaceNSC()->SetFriction(0.2f);

        mphysicalSystem.Add(msphereBody);

        // optional, attach a texture for better visualization
        auto mtexture = std::make_shared<chrono::ChTexture>();
        mtexture->SetTextureFilename(chrono::GetChronoDataFile("bluwhite.png"));
        msphereBody->AddAsset(mtexture);

        auto mboxBody = std::make_shared<chrono::ChBodyEasyBox>(1.5, 1.5, 1.5,  // x,y,z size
                                                                100,            // density
                                                                true,           // collide enable?
                                                                true);          // visualization?
        mboxBody->SetPos(chrono::ChVector<>(-5 + chrono::ChRandom() * 10, 4 + bi * 0.05, -5 + chrono::ChRandom() * 10));

        mphysicalSystem.Add(mboxBody);

        // optional, attach a texture for better visualization
        auto mtexturebox = std::make_shared<chrono::ChTexture>();
        mtexturebox->SetTextureFilename(chrono::GetChronoDataFile("cubetexture_bluwhite.png"));
        mboxBody->AddAsset(mtexturebox);

        auto mcylBody = std::make_shared<chrono::ChBodyEasyCylinder>(0.75, 0.5,  // radius, height
                                                                     100,        // density
                                                                     true,       // collide enable?
                                                                     true);      // visualization?
        mcylBody->SetPos(chrono::ChVector<>(-5 + chrono::ChRandom() * 10, 4 + bi * 0.05, -5 + chrono::ChRandom() * 10));

        mphysicalSystem.Add(mcylBody);

        // optional, attach a texture for better visualization
        auto mtexturecyl = std::make_shared<chrono::ChTexture>();
        mtexturecyl->SetTextureFilename(chrono::GetChronoDataFile("pinkwhite.png"));
        mcylBody->AddAsset(mtexturecyl);
    }

    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    auto floorBody = std::make_shared<chrono::ChBodyEasyBox>(20, 1, 20, 1000, true, true);
    floorBody->SetPos(chrono::ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

    auto wallBody1 = std::make_shared<chrono::ChBodyEasyBox>(1, 10, 20.99, 1000, true, true);
    wallBody1->SetPos(chrono::ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody1);

    auto wallBody2 = std::make_shared<chrono::ChBodyEasyBox>(1, 10, 20.99, 1000, true, true);
    wallBody2->SetPos(chrono::ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody2);

    auto wallBody3 = std::make_shared<chrono::ChBodyEasyBox>(20.99, 10, 1, 1000, true, true);
    wallBody3->SetPos(chrono::ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody3);

    auto wallBody4 = std::make_shared<chrono::ChBodyEasyBox>(20.99, 10, 1, 1000, true, true);
    wallBody4->SetPos(chrono::ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody4);

    // optional, attach  textures for better visualization
    auto mtexturewall = std::make_shared<chrono::ChTexture>();
    mtexturewall->SetTextureFilename(chrono::GetChronoDataFile("concrete.jpg"));
    wallBody1->AddAsset(mtexturewall);  // note: most assets can be shared
    wallBody2->AddAsset(mtexturewall);
    wallBody3->AddAsset(mtexturewall);
    wallBody4->AddAsset(mtexturewall);
    floorBody->AddAsset(mtexturewall);

    // Add the rotating mixer
    auto rotatingBody = std::make_shared<chrono::ChBodyEasyBox>(10, 5, 1,  // x,y,z size
                                                                4000,      // density
                                                                true,      // collide enable?
                                                                true);     // visualization?
    rotatingBody->SetPos(chrono::ChVector<>(0, -1.6, 0));
    rotatingBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);

    mphysicalSystem.Add(rotatingBody);

    // .. a motor between mixer and truss

    auto my_motor = std::make_shared<chrono::ChLinkMotorRotationSpeed>();
    my_motor->Initialize(
        rotatingBody, floorBody,
        chrono::ChFrame<>(chrono::ChVector<>(0, 0, 0), chrono::Q_from_AngAxis(chrono::CH_C_PI_2, {1, 0, 0})));
    auto mfun = std::make_shared<chrono::ChFunction_Const>(chrono::CH_C_PI / 2.0);  // speed w=90�/s
    my_motor->SetSpeedFunction(mfun);
    mphysicalSystem.AddLink(my_motor);
}

int main(int argc, char* argv[]) {
    chrono::GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    chrono::ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc({0, -9.81, 0});

    //===CREATE FALLING OBJECTS AND BOX====
    create_some_falling_items(mphysicalSystem, 10);

    // Modify some setting of the physical system for the simulation, if you want
    // mphysicalSystem.SetSolverType(ChSolver::Type::SOR);
    // mphysicalSystem.SetMaxItersSolverSpeed(100);
    // mphysicalSystem.SetMaxItersSolverStab(5);

    std::cout << "Created ChSystem and objects\n";

    //===Create ChGodot Application===//
    chrono::gd::ChGodotApp app(&mphysicalSystem, 1280, 720, true);

    std::cout << "Created ChGodot object\n";

    //===ChGodot: add sky===//
    app.AddEnvironment();

    //===ChGodot: add lights===//
    app.AddDirectionalLight(chrono::Q_from_AngX(-1.57));
    //    app.AddPointLight({0, 0, 2});

    //===ChGodot: add debug camera===//
    app.AddInteractiveCamera({10, 30, 0}, {0, 0, 0}, {0, 1, 0}, 45);

    //===ChGodot: set fps===//
    app.SetFPS(60);

    //===ChGodot: add HUD===//
    app.SetDisplayHUD(true);

    //===ChGodot: update assets===//
    app.UpdateAssets();

    //===ChGodot: print whats in the scene tree===//
    app.PrintGodotTree();

    // std::cout << "===Godot Tree After Setup===\n";
    // app.PrintGodotTree();

    float time_step = 0.001;
    float m_time = 0;

    //    while(mphysicalSystem.GetChTime() < 100.0){
    while (!app.ShouldClose()) {  // && time < 100
        m_time = mphysicalSystem.GetChTime();
        std::cout << "Time: " << m_time << std::endl;

        app.DoStep(time_step);

        // mphysicalSystem.DoStepDynamics(time_step);
    }

    return 0;
}
