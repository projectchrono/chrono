//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about
//
//     - collisions and contacts
//     - use Irrlicht to display objects.
//
//       (This is just a possible method of integration
//       of Chrono::Engine + Irrlicht: many others
//       are possible.)
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver) {
    for (int bi = 0; bi < 29; bi++) {
        // Create a bunch of ChronoENGINE rigid bodies (spheres and
        // boxes etc.) which will fall..

        ChSharedPtr<ChBodyEasySphere> msphereBody(new ChBodyEasySphere(1.1,     // radius size
                                                                       1000,    // density
                                                                       true,    // collide enable?
                                                                       true));  // visualization?
        msphereBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        msphereBody->GetMaterialSurface()->SetFriction(0.2f);

        mphysicalSystem.Add(msphereBody);

        // optional, attach a texture for better visualization
        ChSharedPtr<ChTexture> mtexture(new ChTexture());
        mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
        msphereBody->AddAsset(mtexture);

        ChSharedPtr<ChBodyEasyBox> mboxBody(new ChBodyEasyBox(1.5, 1.5, 1.5,  // x,y,z size
                                                              100,            // density
                                                              true,           // collide enable?
                                                              true));         // visualization?
        mboxBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));

        mphysicalSystem.Add(mboxBody);

        // optional, attach a texture for better visualization
        ChSharedPtr<ChTexture> mtexturebox(new ChTexture());
        mtexturebox->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));
        mboxBody->AddAsset(mtexturebox);

        ChSharedPtr<ChBodyEasyCylinder> mcylBody(new ChBodyEasyCylinder(0.75, 0.5,  // radius, height
                                                                        100,        // density
                                                                        true,       // collide enable?
                                                                        true));     // visualization?
        mcylBody->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));

        mphysicalSystem.Add(mcylBody);

        // optional, attach a texture for better visualization
        ChSharedPtr<ChTexture> mtexturecyl(new ChTexture());
        mtexturecyl->SetTextureFilename(GetChronoDataFile("pinkwhite.png"));
        mcylBody->AddAsset(mtexturecyl);
    }

    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    ChSharedPtr<ChBodyEasyBox> floorBody(new ChBodyEasyBox(20, 1, 20, 1000, true, true));
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

    ChSharedPtr<ChBodyEasyBox> wallBody1(new ChBodyEasyBox(1, 10, 20.99, 1000, true, true));
    wallBody1->SetPos(ChVector<>(-10, 0, 0));
    wallBody1->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody1);

    ChSharedPtr<ChBodyEasyBox> wallBody2(new ChBodyEasyBox(1, 10, 20.99, 1000, true, true));
    wallBody2->SetPos(ChVector<>(10, 0, 0));
    wallBody2->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody2);

    ChSharedPtr<ChBodyEasyBox> wallBody3(new ChBodyEasyBox(20.99, 10, 1, 1000, true, true));
    wallBody3->SetPos(ChVector<>(0, 0, -10));
    wallBody3->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody3);

    ChSharedPtr<ChBodyEasyBox> wallBody4(new ChBodyEasyBox(20.99, 10, 1, 1000, true, true));
    wallBody4->SetPos(ChVector<>(0, 0, 10));
    wallBody4->SetBodyFixed(true);

    mphysicalSystem.Add(wallBody4);

    // optional, attach  textures for better visualization
    ChSharedPtr<ChTexture> mtexturewall(new ChTexture());
    mtexturewall->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    wallBody1->AddAsset(mtexturewall);  // note: most assets can be shared
    wallBody2->AddAsset(mtexturewall);
    wallBody3->AddAsset(mtexturewall);
    wallBody4->AddAsset(mtexturewall);
    floorBody->AddAsset(mtexturewall);

    // Add the rotating mixer
    ChSharedPtr<ChBodyEasyBox> rotatingBody(new ChBodyEasyBox(10, 5, 1,  // x,y,z size
                                                              4000,      // density
                                                              true,      // collide enable?
                                                              true));    // visualization?
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->GetMaterialSurface()->SetFriction(0.4f);

    mphysicalSystem.Add(rotatingBody);

    // .. an engine between mixer and truss
    ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);
    my_motor->Initialize(rotatingBody, floorBody, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (ChSharedPtr<ChFunction_Const> mfun = my_motor->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
        mfun->Set_yconst(CH_C_PI / 2.0);  // speed w=90�/s
    mphysicalSystem.AddLink(my_motor);

    /*

    /// NOTE: Instead of creating five separate 'box' bodies to make
    /// the walls of the bowl, you could have used a single body
    /// made of five box shapes, which build a single collision description,
    /// as in the alternative approach:

        // create a plain ChBody (no colliding shape nor visualization mesh is used yet)
    ChSharedPtr<ChBodyEasyBox> mrigidBody(new ChBody);

        // set the ChBodySceneNode as fixed body, and turn collision ON, otherwise no collide by default
    mrigidBody->SetBodyFixed(true);
    mrigidBody->SetCollide(true);

        // Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
    mrigidBody->GetCollisionModel()->ClearModel();
        // Describe the (invisible) colliding shape by adding five boxes (the walls and floor)
    mrigidBody->GetCollisionModel()->AddBox(20,1,20, ChVector<>(  0,-10,  0));
    mrigidBody->GetCollisionModel()->AddBox(1,40,20, ChVector<>(-11,  0,  0));
    mrigidBody->GetCollisionModel()->AddBox(1,40,20, ChVector<>( 11,  0,  0));
    mrigidBody->GetCollisionModel()->AddBox(20,40,1, ChVector<>(  0,  0,-11));
    mrigidBody->GetCollisionModel()->AddBox(20,40,1, ChVector<>(  0,  0, 11));
        // Complete the description of collision shape.
    mrigidBody->GetCollisionModel()->BuildModel();

        // Attach some visualization shapes if needed:
    ChSharedPtr<ChBoxShape> vshape (new ChBoxShape() );
    vshape->GetBoxGeometry().SetLengths( ChVector<> (20,1,20) );
    vshape->GetBoxGeometry().Pos = ChVector<> (0,-5,0);
    this->AddAsset( vshape );
    // etc. for other 4 box shapes..
    */
}

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

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
    mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    mphysicalSystem.SetIterLCPmaxItersSpeed(20);
    mphysicalSystem.SetIterLCPmaxItersStab(5);

    // mphysicalSystem.SetUseSleeping(true);

    application.SetStepManage(true);
    application.SetTimestep(0.02);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
