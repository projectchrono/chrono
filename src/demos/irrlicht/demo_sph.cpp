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
//     - SPH smooth particle hydrodynamics
//
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChProximityContainerSPH.h"
#include "chrono/physics/ChMatterSPH.h"

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

void create_some_falling_items(ChIrrAppInterface& mapp) {
    // box data
    double xsize = 0.5;
    double zsize = 0.5;
    double height = 0.5;
    double thick = 0.1;

    ChSystem* mphysicalSystem = mapp.GetSystem();
    ISceneManager* msceneManager = mapp.GetSceneManager();
    IVideoDriver* driver = mapp.GetVideoDriver();

    // Create the SPH fluid
    ChSharedPtr<ChMatterSPH> myfluid(new ChMatterSPH);

    // Use the FillBox easy way to create the set of SPH particles
    myfluid->FillBox(ChVector<>(xsize - 0.2, height, zsize),                       // size of box
                     xsize / 16.0,                                                 // resolution step
                     1000,                                                         // initial density
                     ChCoordsys<>(ChVector<>(0.1, height * 0.5 + 0.0, 0), QUNIT),  // position & rotation of box
                     true,  // do a centered cubic lattice initial arrangement
                     1.5,   // set the kernel radius (as multiples of step)
                     0.3);  // the randomness to avoid too regular initial lattice

    // Set some material properties of the SPH fluid
    myfluid->GetMaterial().Set_viscosity(0.5);
    myfluid->GetMaterial().Set_pressure_stiffness(300);

    // Add the SPH fluid matter to the system
    myfluid->SetCollide(true);
    mphysicalSystem->Add(myfluid);

    GetLog() << "Added " << myfluid->GetNnodes() << " SPH particles \n\n";

    // Create some spheres that will fall

    ChBodySceneNode* mrigidBody;

    for (int bi = 0; bi < 0; bi++) {
        mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
            mphysicalSystem, msceneManager, 10.0, ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10),
            1.1);

        mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0f);
        mrigidBody->addShadowVolumeSceneNode();

        video::ITexture* sphereMap = driver->getTexture(GetChronoDataFile("bluwhite.png").c_str());
        mrigidBody->setMaterialTexture(0, sphereMap);
    }

    // Create the five walls of the rectangular container, using
    // fixed rigid bodies of 'box' type:

    mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        mphysicalSystem, msceneManager, 100.0, ChVector<>(0, -thick * 0.5, 0), ChQuaternion<>(1, 0, 0, 0),
        ChVector<>(xsize + 2 * thick, thick, zsize + 2 * thick));
    mrigidBody->GetBody()->SetBodyFixed(true);
    mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0f);

    video::ITexture* cubeMap = driver->getTexture(GetChronoDataFile("blu.png").c_str());
    mrigidBody->setMaterialTexture(0, cubeMap);

    mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        mphysicalSystem, msceneManager, 100.0, ChVector<>(-xsize * 0.5 - thick * 0.5, height * 0.5, 0),
        ChQuaternion<>(1, 0, 0, 0), ChVector<>(thick, height, zsize + 2 * thick));
    mrigidBody->GetBody()->SetBodyFixed(true);
    mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0f);
    mrigidBody->setMaterialTexture(0, cubeMap);

    mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        mphysicalSystem, msceneManager, 100.0, ChVector<>(xsize * 0.5 + thick * 0.5, height * 0.5, 0),
        ChQuaternion<>(1, 0, 0, 0), ChVector<>(thick, height, zsize + 2 * thick));
    mrigidBody->GetBody()->SetBodyFixed(true);
    mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0f);
    mrigidBody->setMaterialTexture(0, cubeMap);

    mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        mphysicalSystem, msceneManager, 100.0, ChVector<>(0, height * 0.5, -zsize * 0.5 - thick * 0.5),
        ChQuaternion<>(1, 0, 0, 0), ChVector<>(xsize + 2 * thick, height, thick));
    mrigidBody->GetBody()->SetBodyFixed(true);
    mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0f);
    mrigidBody->setMaterialTexture(0, cubeMap);

    double opening = 0.2;  //=0.2
    mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
        mphysicalSystem, msceneManager, 100.0, ChVector<>(opening, height * 0.5, zsize * 0.5 + thick * 0.5),
        ChQuaternion<>(1, 0, 0, 0), ChVector<>(xsize + 2 * thick, height, thick));
    mrigidBody->GetBody()->SetBodyFixed(true);
    mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.0f);
    mrigidBody->setMaterialTexture(0, cubeMap);

    // another floor

    mrigidBody =
        (ChBodySceneNode*)addChBodySceneNode_easyBox(mphysicalSystem, msceneManager, 100.0, ChVector<>(0, -0.5, 0),
                                                     ChQuaternion<>(1, 0, 0, 0), ChVector<>(2, 0.1, 2));
    mrigidBody->GetBody()->SetBodyFixed(true);
    mrigidBody->GetBody()->GetMaterialSurface()->SetFriction(0.2f);

    // Create floating balls
    for (int ib = 0; ib < 12; ib++) {
        ChSharedPtr<ChBodyEasySphere> mrigidBall(new ChBodyEasySphere(0.02 + ChRandom() * 0.02,  // radius
                                                                      100,                       // density
                                                                      true,                      // collide enable?
                                                                      true));                    // visualization?
        mrigidBall->SetPos(ChVector<>(ChRandom() * 0.3 - 0.15, 0.2, ChRandom() * 0.3 - 0.15));
        mrigidBall->GetMaterialSurface()->SetFriction(0.0f);
        mphysicalSystem->Add(mrigidBall);

        // optional, attach a texture for better visualization
        ChSharedPtr<ChTexture> mtextureball(new ChTexture());
        mtextureball->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
        mrigidBall->AddAsset(mtextureball);
    }
}

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"SPH fluid", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 1, -1));

    // Create all the rigid bodies.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.003);

    create_some_falling_items(application);

    // Use this function for adding a ChIrrNodeAsset to all items
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' into Irrlicht meshes the assets
    // into Irrlicht-visualizable meshes
    application.AssetUpdateAll();

    // IMPORTANT!
    // This takes care of the interaction between the particles of the SPH material
    ChSharedPtr<ChProximityContainerSPH> my_sph_proximity(new ChProximityContainerSPH);
    mphysicalSystem.Add(my_sph_proximity);


    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetIterLCPmaxItersSpeed(6);  // lower the LCP iters, no needed here

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetStepManage(true);
    application.SetTimestep(0.0025);

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        ChSystem::IteratorOtherPhysicsItems myiter = mphysicalSystem.IterBeginOtherPhysicsItems();
        while (myiter != mphysicalSystem.IterEndOtherPhysicsItems()) {
            if (ChMatterSPH* myfluid = dynamic_cast<ChMatterSPH*>((*myiter).get_ptr())) {
                for (unsigned int ip = 0; ip < myfluid->GetNnodes(); ip++) {
                    // ChNodeSPH* mnode = (ChNodeSPH*)(myfluid->GetNode(ip).get_ptr());
                    ChSharedPtr<ChNodeSPH> mnode = myfluid->GetNode(ip).DynamicCastTo<ChNodeSPH>();

                    ChVector<> mv = mnode->GetPos();
                    float rad = (float)mnode->GetKernelRadius();
                    core::vector3df mpos((irr::f32)mv.x, (irr::f32)mv.y, (irr::f32)mv.z);
                    core::position2d<s32> spos =
                        application.GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(
                            mpos);
                    application.GetVideoDriver()->draw2DRectangle(
                        video::SColor(100, 200, 200, 230),
                        core::rect<s32>(spos.X - 2, spos.Y - 2, spos.X + 2, spos.Y + 2));
                    /*
                    application.GetVideoDriver()->setTransform(video::ETS_WORLD, core::matrix4());
                    application.GetVideoDriver()->draw3DBox( core::aabbox3d<f32>(
                                    (irr::f32)mv.x-rad ,(irr::f32)mv.y-rad , (irr::f32)mv.z-rad    ,
                                    (irr::f32)mv.x+rad ,(irr::f32)mv.y+rad , (irr::f32)mv.z+rad )   ,
                                    video::SColor(300,200,200,230) );
                    */

                    /*
                    double strain_scale =1;
                    ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_X*mnode->p_strain.XX()* strain_scale), video::SColor(255,255,0,0),false);
                    ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Y*mnode->p_strain.YY()* strain_scale), video::SColor(255,0,255,0),false);
                    ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Z*mnode->p_strain.ZZ()* strain_scale), video::SColor(255,0,0,255),false);
                    */

                    /*
                    double stress_scale =0.008;
                    ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_X*mnode->e_stress.XX()* stress_scale), video::SColor(100,255,0,0),false);
                    ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Y*mnode->e_stress.YY()* stress_scale), video::SColor(100,0,255,0),false);
                    ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Z*mnode->e_stress.ZZ()* stress_scale), video::SColor(100,0,0,255),false);
                    */

                    // ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    // mnode->GetPos()+(mnode->UserForce * 0.1), video::SColor(100,0,0,0),false);
                }
            }
            ++myiter;
        }

        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
