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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_irrlicht/ChIrrApp.h"

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

void create_some_falling_items(ChSystemNSC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver) {
    // Shared contact materials for falling objects
    auto sph_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    sph_mat->SetFriction(0.2f);
    auto box_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto cyl_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create falling rigid bodies (spheres and boxes etc.)
    for (int bi = 0; bi < 29; bi++) {
        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(1.1,       // radius size
                                                                       1000,      // density
                                                                       true,      // visualization?
                                                                       true,      // collision?
                                                                       sph_mat);  // contact material
        msphereBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        mphysicalSystem.Add(msphereBody);

        auto mtexture = chrono_types::make_shared<ChTexture>();
        mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
        msphereBody->AddAsset(mtexture);

        auto mboxBody = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5,  // x,y,z size
                                                                 100,            // density
                                                                 true,           // visualization?
                                                                 true,           // collision?
                                                                 box_mat);       // contact material
        mboxBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));

        mphysicalSystem.Add(mboxBody);

        auto mtexturebox = chrono_types::make_shared<ChTexture>();
        mtexturebox->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));
        mboxBody->AddAsset(mtexturebox);

        auto mcylBody = chrono_types::make_shared<ChBodyEasyCylinder>(0.75, 0.5,  // radius, height
                                                                      100,        // density
                                                                      true,       // visualization?
                                                                      true,       // collision?
                                                                      cyl_mat);   // contact material
        mcylBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));

        mphysicalSystem.Add(mcylBody);

        // optional, attach a texture for better visualization
        auto mtexturecyl = chrono_types::make_shared<ChTexture>();
        mtexturecyl->SetTextureFilename(GetChronoDataFile("pinkwhite.png"));
        mcylBody->AddAsset(mtexturecyl);
    }

    // Contact material for container
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create the five walls of the rectangular container, using fixed rigid bodies of 'box' type
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true, ground_mat);
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);
    mphysicalSystem.Add(floorBody);

    auto wallBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, true, true, ground_mat);
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);
    mphysicalSystem.Add(wallBody1);

    auto wallBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 10, 20.99, 1000, true, true, ground_mat);
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);
    mphysicalSystem.Add(wallBody2);

    auto wallBody3 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, true, true, ground_mat);
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);
    mphysicalSystem.Add(wallBody3);

    auto wallBody4 = chrono_types::make_shared<ChBodyEasyBox>(20.99, 10, 1, 1000, true, true, ground_mat);
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);
    mphysicalSystem.Add(wallBody4);

    // optional, attach  textures for better visualization
    auto mtexturewall = chrono_types::make_shared<ChTexture>();
    mtexturewall->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    wallBody1->AddAsset(mtexturewall);  // note: most assets can be shared
    wallBody2->AddAsset(mtexturewall);
    wallBody3->AddAsset(mtexturewall);
    wallBody4->AddAsset(mtexturewall);
    floorBody->AddAsset(mtexturewall);

    // Add the rotating mixer
    auto mixer_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mixer_mat->SetFriction(0.4f);

    auto rotatingBody = chrono_types::make_shared<ChBodyEasyBox>(10, 5, 1,  // x,y,z size
                                                        4000,            // density
                                                                 true,        // visualization?
                                                                 true,        // collision?
                                                                 mixer_mat);  // contact material
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    mphysicalSystem.Add(rotatingBody);

    // .. a motor between mixer and truss

    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(rotatingBody, floorBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto mfun = chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2.0);  // speed w=90°/s
    my_motor->SetSpeedFunction(mfun);
    mphysicalSystem.AddLink(my_motor);

    /*

    /// NOTE: Instead of creating five separate 'box' bodies to make
    /// the walls of the container, you could have used a single body
    /// made of five box shapes, which build a single collision description,
    /// as in the alternative approach:

        // create a plain ChBody (no colliding shape nor visualization mesh is used yet)
    auto mrigidBody = chrono_types::make_shared<ChBody>();

        // set as fixed body, and turn collision ON, otherwise no collide by default
    mrigidBody->SetBodyFixed(true);
    mrigidBody->SetCollide(true);

        // Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
    mrigidBody->GetCollisionModel()->ClearModel();
        // Describe the (invisible) colliding shape by adding five boxes (the walls and floor)
    mrigidBody->GetCollisionModel()->AddBox(ground_mat, 20,1,20, ChVector<>(  0,-10,  0));
    mrigidBody->GetCollisionModel()->AddBox(ground_mat, 1,40,20, ChVector<>(-11,  0,  0));
    mrigidBody->GetCollisionModel()->AddBox(ground_mat, 1,40,20, ChVector<>( 11,  0,  0));
    mrigidBody->GetCollisionModel()->AddBox(ground_mat, 20,40,1, ChVector<>(  0,  0,-11));
    mrigidBody->GetCollisionModel()->AddBox(ground_mat, 20,40,1, ChVector<>(  0,  0, 11));
        // Complete the description of collision shape.
    mrigidBody->GetCollisionModel()->BuildModel();

        // Attach some visualization shapes if needed:
    auto vshape = chrono_types::make_shared<ChBoxShape>();
    vshape->GetBoxGeometry().SetLengths( ChVector<> (20,1,20) );
    vshape->GetBoxGeometry().Pos = ChVector<> (0,-5,0);
    this->AddAsset( vshape );
    // etc. for other 4 box shapes..
    */
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    // Create all the rigid bodies.

    create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(20);

    // mphysicalSystem.SetUseSleeping(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //
   
    application.SetTimestep(0.02);

    while (application.GetDevice()->run()) {

        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();
        
        application.EndScene();
    }

    return 0;
}
