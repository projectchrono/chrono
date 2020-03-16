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

#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono/fea/ChMatterMeshless.h"
#include "chrono/fea/ChProximityContainerMeshless.h"

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

// Use the namespace of Chrono

using namespace chrono;
using namespace fea;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Meshless deformable material", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 3, -3));

    // CREATE A FLOOR

    ChSharedPtr<ChBodyEasyBox> mfloorBody( new ChBodyEasyBox(20,1,20,1000,true,true));
    my_system.Add(mfloorBody);
    mfloorBody->SetBodyFixed(true);
    mfloorBody->SetPos(ChVector<>(0, -5, 0));

    ChSharedPtr<ChTexture> mtexture( new ChTexture(GetChronoDataFile("concrete.jpg").c_str()));
    mfloorBody->AddAsset(mtexture);

   
    // CREATE THE ELASTOPLASTIC MESHLESS CONTINUUM

    // Create elastoplastic matter
    ChSharedPtr<ChMatterMeshless> mymatter(new ChMatterMeshless);

    // Use the FillBox easy way to create the set of nodes in the meshless matter
    mymatter->FillBox(ChVector<>(4, 2, 4),                          // size of box
                      4.0 / 10.0,                                   // resolution step
                      1000,                                         // initial density
                      ChCoordsys<>(ChVector<>(0, -3.9, 0), QUNIT),  // position & rotation of box
                      true,                                         // do a centered cubic lattice initial arrangement
                      2.1);                                         // set the kernel radius (as multiples of step)

    GetLog() << "Added " << mymatter->GetNnodes() << " nodes \n";

    // Set some material properties of the meshless matter

    ChSharedPtr<ChContinuumDruckerPrager> mmaterial(new ChContinuumDruckerPrager);
    mymatter->ReplaceMaterial(mmaterial);
    mmaterial->Set_v(0.35);
    mmaterial->Set_E(30000.0);
    mmaterial->Set_elastic_yeld(0);
    mmaterial->Set_alpha(30 * CH_C_DEG_TO_RAD);
    // mmaterial->Set_from_MohrCoulomb(30 * CH_C_DEG_TO_RAD, 1000);
    mmaterial->Set_dilatancy(30 * CH_C_DEG_TO_RAD);
    mmaterial->Set_flow_rate(50000000.0);
    mmaterial->Set_hardening_limit(mmaterial->Get_elastic_yeld());
    mmaterial->Set_hardening_speed(100000000);

    /*
        ChSharedPtr<ChContinuumPlasticVonMises> mmaterial(new ChContinuumPlasticVonMises);
        mymatter->ReplaceMaterial(mmaterial);
        mmaterial->Set_v(0.38);
        mmaterial->Set_E(60000.0);
        mmaterial->Set_elastic_yeld(0.06);
        mmaterial->Set_flow_rate(300);
    */

    mymatter->SetViscosity(5000);

    // Add the matter to the physical system
    mymatter->SetCollide(true);
    mphysicalSystem.Add(mymatter);

    // Join some nodes of meshless matter to a ChBody
    /*
    ChSharedPtr<ChIndexedNodes> mnodes = mymatter;
    for (int ij = 0; ij < 120; ij++)
    {
        ChSharedPtr<ChLinkPointFrame> myjointnodebody(new ChLinkPointFrame);
        myjointnodebody->Initialize(mnodes,
                                    ij,
                                    mfloorBody->GetBody());
        mphysicalSystem.Add(myjointnodebody);
    }
    */

    // IMPORTANT!
    // This takes care of the interaction between the particles of the meshless material
    ChSharedPtr<ChProximityContainerMeshless> my_sph_proximity(new ChProximityContainerMeshless);
    mphysicalSystem.Add(my_sph_proximity);


    // CREATE A SPHERE PRESSING THE MATERIAL:

    ChSharedPtr<ChBodyEasySphere> msphere( new ChBodyEasySphere(2, 7000, true,true));
    my_system.Add(msphere);
    msphere->SetPos(ChVector<>(0, -0.5, 0));

    

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();


    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverMaxIterations(25);  // lower the LCP iters, no needed here

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    static int printed_prox = 0;
    application.SetTimestep(0.002);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        ChSystem::IteratorOtherPhysicsItems myiter = mphysicalSystem.IterBeginOtherPhysicsItems();

        while (myiter != mphysicalSystem.IterEndOtherPhysicsItems()) {
            if ((*myiter).IsType<ChMatterMeshless>()) {
                ChSharedPtr<ChMatterMeshless> mymatter((*myiter).DynamicCastTo<ChMatterMeshless>());

                for (unsigned int ip = 0; ip < mymatter->GetNnodes(); ip++) {
                    ChSharedPtr<ChNodeMeshless> mnode(mymatter->GetNode(ip).DynamicCastTo<ChNodeMeshless>());
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
                    double strain_scale =10;
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

                    double stress_scale = 0.00001;
                    ChIrrTools::drawSegment(
                        application.GetVideoDriver(), mnode->GetPos(),
                        mnode->GetPos() + (VECT_X * mnode->e_stress.GetInvariant_I1() * stress_scale),
                        video::SColor(100, 255, 0, 0), false);

                    // GetLog() << "Mass i="<< ip << "   m=" << mnode->GetMass() << "\n";
                    // ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    // mnode->GetPos()+(mnode->UserForce * 0.1), video::SColor(100,0,0,0),false);
                }
            }
            ++myiter;
        }

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
