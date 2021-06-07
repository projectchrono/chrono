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
// SPH smooth particle hydrodynamics demo
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChProximityContainerSPH.h"
#include "chrono/physics/ChMatterSPH.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;

void create_some_falling_items(ChSystemNSC& system) {
    // box data
    double xsize = 0.5;
    double zsize = 0.5;
    double height = 0.5;
    double thick = 0.1;

    // Create the SPH fluid
    auto myfluid = chrono_types::make_shared<ChMatterSPH>();

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
    system.Add(myfluid);

    GetLog() << "Added " << myfluid->GetNnodes() << " SPH particles \n\n";

    // Create the five walls of the rectangular container.
    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/blue.png"));

    auto wall_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    wall_mat->SetFriction(0.0f);

    auto wall1 = chrono_types::make_shared<ChBodyEasyBox>(xsize + 2 * thick, thick, zsize + 2 * thick, 1.0, true, true,
                                                          wall_mat);
    wall1->SetPos(ChVector<>(0, -thick * 0.5, 0));
    wall1->SetBodyFixed(true);
    wall1->SetMass(100);
    wall1->AddAsset(texture);
    system.Add(wall1);

    auto wall2 = chrono_types::make_shared<ChBodyEasyBox>(thick, height, zsize + 2 * thick, 1.0, true, true, wall_mat);
    wall2->SetPos(ChVector<>(-xsize * 0.5 - thick * 0.5, height * 0.5, 0));
    wall2->SetBodyFixed(true);
    wall2->SetMass(100);
    wall2->AddAsset(texture);
    system.Add(wall2);

    auto wall3 = chrono_types::make_shared<ChBodyEasyBox>(thick, height, zsize + 2 * thick, 1.0, true, true, wall_mat);
    wall3->SetPos(ChVector<>(xsize * 0.5 + thick * 0.5, height * 0.5, 0));
    wall3->SetBodyFixed(true);
    wall3->SetMass(100);
    wall3->AddAsset(texture);
    system.Add(wall3);

    auto wall4 = chrono_types::make_shared<ChBodyEasyBox>(xsize + 2 * thick, height, thick, 1.0, true, true, wall_mat);
    wall4->SetPos(ChVector<>(0, height * 0.5, -zsize * 0.5 - thick * 0.5));
    wall4->SetBodyFixed(true);
    wall4->SetMass(100);
    wall4->AddAsset(texture);
    system.Add(wall4);

    double opening = 0.2;
    auto wall5 = chrono_types::make_shared<ChBodyEasyBox>(xsize + 2 * thick, height, thick, 1.0, true, true, wall_mat);
    wall5->SetPos(ChVector<>(opening, height * 0.5, zsize * 0.5 + thick * 0.5));
    wall5->SetBodyFixed(true);
    wall5->SetMass(100);
    wall5->AddAsset(texture);
    system.Add(wall5);

    // Create the floor.
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 2, 1.0, true, true, wall_mat);
    floor->SetPos(ChVector<>(0, -0.5, 0));
    floor->SetBodyFixed(true);
    floor->SetMass(100);
    system.Add(floor);

    // Create floating balls.
    auto ball_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ball_mat->SetFriction(0.0f);

    auto ball_texture = chrono_types::make_shared<ChTexture>();
    ball_texture->SetTextureFilename(GetChronoDataFile("textures/bluewhite.png"));

    for (int ib = 0; ib < 12; ib++) {
        auto ball = chrono_types::make_shared<ChBodyEasySphere>(0.02 + ChRandom() * 0.02, 100, true, true, ball_mat);
        ball->SetPos(ChVector<>(ChRandom() * 0.3 - 0.15, 0.2, ChRandom() * 0.3 - 0.15));
        ball->AddAsset(ball_texture);
        system.Add(ball);
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"SPH fluid", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 1, -1));

    // Create all the rigid bodies.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.003);

    create_some_falling_items(mphysicalSystem);

    // Use this function for adding a ChIrrNodeAsset to all items
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' into Irrlicht meshes the assets
    // into Irrlicht-visualizable meshes
    application.AssetUpdateAll();

    // IMPORTANT!
    // This takes care of the interaction between the particles of the SPH material
    auto my_sph_proximity = chrono_types::make_shared<ChProximityContainerSPH>();
    mphysicalSystem.Add(my_sph_proximity);

    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverMaxIterations(6);  // lower the solver iters, no needed here

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetTimestep(0.0025);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, video::SColor(255, 140, 161, 192));

        application.DrawAll();

        for (auto ph : mphysicalSystem.Get_otherphysicslist()) {
            if (ChMatterSPH* myfluid = dynamic_cast<ChMatterSPH*>(ph.get())) {
                for (unsigned int ip = 0; ip < myfluid->GetNnodes(); ip++) {
                    auto mnode = std::dynamic_pointer_cast<ChNodeSPH>(myfluid->GetNode(ip));

                    ChVector<> mv = mnode->GetPos();
                    ////float rad = (float)mnode->GetKernelRadius();
                    core::vector3df mpos((irr::f32)mv.x(), (irr::f32)mv.y(), (irr::f32)mv.z());
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
                    tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_X*mnode->p_strain.XX()* strain_scale), video::SColor(255,255,0,0),false);
                    tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Y*mnode->p_strain.YY()* strain_scale), video::SColor(255,0,255,0),false);
                    tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Z*mnode->p_strain.ZZ()* strain_scale), video::SColor(255,0,0,255),false);
                    */

                    /*
                    double stress_scale =0.008;
                    tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_X*mnode->e_stress.XX()* stress_scale), video::SColor(100,255,0,0),false);
                    tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Y*mnode->e_stress.YY()* stress_scale), video::SColor(100,0,255,0),false);
                    tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    mnode->GetPos()+(VECT_Z*mnode->e_stress.ZZ()* stress_scale), video::SColor(100,0,0,255),false);
                    */

                    // tools::drawSegment(application.GetVideoDriver(), mnode->GetPos(),
                    // mnode->GetPos()+(mnode->UserForce * 0.1), video::SColor(100,0,0,0),false);
                }
            }
        }

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
