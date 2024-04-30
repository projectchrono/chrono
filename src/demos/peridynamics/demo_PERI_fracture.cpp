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
// Demo code explaining how to create a deformable solid using peridynamics. 
// Peridynamics allow an efficient handling of fracturing and cracks, working as
// a meshless framework: differently from conventional FEA it does not need 
// that the solid is preprocessed as a tetrahedral mesh. In the peridynamics 
// context, the solid is made with a node cloud, where nodes are connected
// each other (if withing a small distance called 'horizon') by microscopic bounds. 
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChLinkNodeFrame.h"

#include "chrono_peridynamics/ChMatterPeriSprings.h"
#include "chrono_peridynamics/ChMatterPeriBulkElastic.h"
#include "chrono_peridynamics/ChProximityContainerPeridynamics.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "chrono_postprocess/ChBlender.h"


// Use the namespaces of Chrono

using namespace chrono;
using namespace peridynamics;
using namespace postprocess;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Set small collision envelopes for objects that will be created from now on
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
    mphysicalSystem.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);


    // CREATE A FLOOR
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.4f);

    auto mfloorBody = chrono_types::make_shared<ChBodyEasyBox>(20,1,20,1000,true,true,mat);
    mphysicalSystem.Add(mfloorBody);
    mfloorBody->SetFixed(true);
    mfloorBody->SetPos(ChVector3d(0, -7.5, 0));

    mfloorBody->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.2f));


    // CREATE A SPHERE PRESSING THE MATERIAL:
    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.7, 23000, true,true, mat);
    mphysicalSystem.Add(msphere);
    msphere->SetPos(ChVector3d(0, -0.5, 0));
    msphere->SetPosDt(ChVector3d(0, -16.5, 0));

   

    // CREATE THE PERIDYNAMIC CONTINUUM


    // Create peridynamics matter - a very simple one
    /*
    auto mymattersprings = chrono_types::make_shared<ChMatterPeriSpringsBreakable>();
    mymattersprings->k = 260000; // stiffness of bounds
    mymattersprings->r = 300; // damping of bounds
    mymattersprings->max_stretch = 0.08;
    */
    auto mymattersprings = chrono_types::make_shared<ChMatterPeriBulkElastic>();
    mymattersprings->k_bulk = 26000; // bulk stiffness 
    mymattersprings->r = 10; // bulk damping s
    mymattersprings->max_stretch = 0.08;

    // IMPORTANT!
    // This takes care of the interaction between the particles of the Peridynamics material
    auto my_peri_proximity = chrono_types::make_shared<ChProximityContainerPeri>();
    mphysicalSystem.Add(my_peri_proximity);

    my_peri_proximity->AddMatter(mymattersprings);

    // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
    my_peri_proximity->FillBox(
        mymattersprings,
        ChVector3d(3, 1.5, 3),                        // size of box
        4.0 / 20.0,                                   // resolution step
        1000,                                         // initial density
        ChCoordsys<>(ChVector3d(0, -3.4, 0), QUNIT),  // position & rotation of box
        false,                                        // do a centered cubic lattice initial arrangement
        1.6,                                          // set the horizon radius (as multiples of step) 
        0.4);                                         // set the collision radius (as multiples of step) for interface particles

    // Just for testing, fix some nodes
    for (const auto& node : my_peri_proximity->GetNodes()) {
        if (node->GetPos().z() < -2.70 || node->GetPos().z() > 2.50 || node->GetPos().x() < -2.70 || node->GetPos().x() > 2.50)
            node->SetFixed(true);
    }

    // to automate visualization
    /*
    auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriSpringsBreakable>(mymattersprings);
    my_peri_proximity->AddVisualShape(mglyphs_nodes);

    auto mglyphs_bounds = chrono_types::make_shared<ChVisualPeriSpringsBreakableBounds>(mymattersprings);
    mglyphs_bounds->draw_unbroken = true;
    mglyphs_bounds->draw_broken = true;
    my_peri_proximity->AddVisualShape(mglyphs_bounds);
    */
    auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriBulkElastic>(mymattersprings);
    my_peri_proximity->AddVisualShape(mglyphs_nodes);
    mglyphs_nodes->SetGlyphsSize(0.04);

    //auto mglyphs_bounds = chrono_types::make_shared<ChVisualPeriBulkElasticBounds>(mymattersprings);
    //mglyphs_bounds->draw_unbroken = true;
    //mglyphs_bounds->draw_broken = true;
    //my_peri_proximity->AddVisualShape(mglyphs_bounds);

    // -----Blender postprocess

    // Create an exporter to Blender
    ChBlender blender_exporter = ChBlender(&mphysicalSystem);

    // Set the path where it will save all files, a directory will be created if not existing
    blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI");

    // Export all existing visual shapes to POV-Ray
    blender_exporter.AddAll();

    blender_exporter.SetCamera(ChVector3d(3, 4, -5), ChVector3d(0, 0.5, 0), 50);  // pos, aim, angle

    blender_exporter.ExportScript();

    // --------------------
    

    // Create the Irrlicht visualization system
    auto vsys = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vsys->AttachSystem(&mphysicalSystem);
    vsys->SetWindowSize(1024, 768);
    vsys->SetWindowTitle("Peridynamics test");
    vsys->Initialize();
    vsys->AddLogo();
    vsys->AddSkyBox();
    vsys->AddCamera(ChVector3d(-6, 0.3, 2.3), ChVector3d(0, -4, 0));
    vsys->AddLight(ChVector3d(30, 30, 60), 120, ChColor(0.6f, 0.6f, 0.6f));
    vsys->AddLight(ChVector3d(40, 60, 30), 120, ChColor(0.6f, 0.6f, 0.6f));


    // Modify some setting of the physical system for the simulation, if you want
    if (mphysicalSystem.GetSolver()->IsIterative()) {
        mphysicalSystem.GetSolver()->AsIterative()->SetMaxIterations(25);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    double timestep = 0.00010;

    while (vsys->Run()) {
        vsys->BeginScene();
        vsys->Render();
        
        if(true)
        for  (auto& myiter : mphysicalSystem.GetOtherPhysicsItems()) {

            // OLD --
            /*
            if (auto myimatter = std::dynamic_pointer_cast<ChMatterPeridynamics>(myiter)) {
                
                for (unsigned int ip = 0; ip < myimatter->GetNnodes(); ip++) {
                    auto mnode = std::dynamic_pointer_cast<ChNodePeridynamics>(myimatter->GetNode(ip));

                    ChVector<> mv = mnode->GetPos();
                    float rad = (float)mnode->GetHorizonRadius();
                    core::vector3df mpos((irr::f32)mv.x(), (irr::f32)mv.y(), (irr::f32)mv.z());

                    // draw points
                    if (true) {
                        core::position2d<s32> spos = vsys->GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
                        vsys->GetVideoDriver()->draw2DRectangle(
                            (mnode->is_boundary ? video::SColor(200, 0, 0, 230) :  video::SColor(100, 200, 200, 230)) ,
                            core::rect<s32>(spos.X - 2, spos.Y - 2, spos.X + 2, spos.Y + 2));
                    }

                    // draw aabb of nodes 
                    if (false) {
                        double msize = rad;
                        vsys->GetVideoDriver()->setTransform(video::ETS_WORLD, core::matrix4());
                        vsys->GetVideoDriver()->draw3DBox(core::aabbox3d<f32>(
                            (irr::f32)mv.x() - msize, (irr::f32)mv.y() - msize, (irr::f32)mv.z() - msize,
                            (irr::f32)mv.x() + msize, (irr::f32)mv.y() + msize, (irr::f32)mv.z() + msize),
                            video::SColor(300, 200, 200, 230));
                    }
                    
                    // draw strain diagonal
                    if (false) {
                        double strain_scale = 100000;
                        irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                            mnode->GetPos() + (VECT_X * mnode->p_strain.XX() * strain_scale), ChColor(1, 0, 0), false);
                        irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                            mnode->GetPos() + (VECT_Y * mnode->p_strain.YY() * strain_scale), ChColor(0, 1, 0), false);
                        irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                            mnode->GetPos() + (VECT_Z * mnode->p_strain.ZZ() * strain_scale), ChColor(0, 0, 1), false);
                    }

                    // draw stress diagonal
                    if (false) {
                        double stress_scale =0.00001;
                        irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                        mnode->GetPos()+(VECT_X*mnode->e_stress.XX()* stress_scale), ChColor(1, 0, 0),false);
                        irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                        mnode->GetPos()+(VECT_Y*mnode->e_stress.YY()* stress_scale), ChColor(0, 1, 0),false);
                        irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                        mnode->GetPos()+(VECT_Z*mnode->e_stress.ZZ()* stress_scale), ChColor(0, 0, 1),false);
                    }

                    // draw strain invariants
                    if (false) {
                        double stress_scale = 0.00001;
                        irrlicht::tools::drawSegment(
                            vsys.get(), mnode->GetPos(),
                            mnode->GetPos() + (VECT_X * mnode->e_stress.GetEquivalentVonMises() * stress_scale),
                            ChColor(1, 1, 1), false);
                    }

                    // draw strain principal axes
                    if (true) {
                            double stress_scale = 0.00001;
                            double s1, s2, s3;
                            ChVector<> v1, v2, v3;
                            mnode->e_stress.ComputeEigenvectors(s1, s2, s3, v1, v2, v3);
                            irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                            mnode->GetPos()+(v1*s1* stress_scale), ChColor(1, 0, 0),false);
                            irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                            mnode->GetPos()+(v2*s2* stress_scale), ChColor(0, 1, 0),false);
                            irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                            mnode->GetPos()+(v3*s3* stress_scale), ChColor(0, 0, 1),false);
                    }
                    // GetLog() << "Mass i="<< ip << "   m=" << mnode->GetMass() << "\n";
                    // irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                    // mnode->GetPos()+(mnode->UserForce * 0.1), ChColor(0, 0, 0),false);
                }
            }*/

            // NEW----

            if (auto myperi = std::dynamic_pointer_cast<ChProximityContainerPeri>(myiter)) {

                // show nodes
                for (const auto& mnode : myperi->GetNodes()) {

                    ChVector3d mv = mnode->GetPos();
                    float rad = (float)mnode->GetHorizonRadius();
                    core::vector3df mpos((irr::f32)mv.x(), (irr::f32)mv.y(), (irr::f32)mv.z());

                    // draw points
                    if (false) {
                        core::position2d<s32> spos = vsys->GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
                        irr::video::SMaterial mattransp;
                        mattransp.ZBuffer = false;
                        mattransp.Lighting = false;
                        vsys->GetVideoDriver()->setMaterial(mattransp);
                        vsys->GetVideoDriver()->draw2DRectangle(
                            //(mnode->is_boundary ? video::SColor(200, 0, 0, 230) :  video::SColor(100, 200, 200, 230)) ,
                            (mnode->is_colliding ? video::SColor(200, 0, 0, 230) : video::SColor(100, 200, 200, 230)),
                            core::rect<s32>(spos.X - 2, spos.Y - 2, spos.X + 2, spos.Y + 2));
                    }

                    // draw horizon of nodes 
                    if (false) {
                        double msize = rad;
                        vsys->GetVideoDriver()->setTransform(video::ETS_WORLD, core::matrix4());
                        vsys->GetVideoDriver()->draw3DBox(core::aabbox3d<f32>(
                            (irr::f32)mv.x() - msize, (irr::f32)mv.y() - msize, (irr::f32)mv.z() - msize,
                            (irr::f32)mv.x() + msize, (irr::f32)mv.y() + msize, (irr::f32)mv.z() + msize),
                            video::SColor(300, 200, 200, 230));
                    }
                }

                // show materials
                for (const auto& mmaterial : myperi->GetMaterials()) {

                    if (auto mysmatter = std::dynamic_pointer_cast<ChMatterPeriSprings>(mmaterial)) {
                        // draw the material as bonds
                        for (const auto& mbounddata : mysmatter->GetMapOfBounds()) {
                            auto mnodeA = mbounddata.second.nodeA;
                            auto mnodeB = mbounddata.second.nodeB;

                            if (false)
                                irrlicht::tools::drawSegment(vsys.get(), mnodeA->GetPos(), mnodeB->GetPos(), ChColor(0, 0, 0), true);
                        }
                    }
                    if (auto mysmatter = std::dynamic_pointer_cast<ChMatterPeriSpringsBreakable>(mmaterial)) {
                        // draw the material as bonds
                        for (const auto& mbounddata : mysmatter->GetMapOfBounds()) {
                            auto mnodeA = mbounddata.second.nodeA;
                            auto mnodeB = mbounddata.second.nodeB;

                            if (true) {
                                if (mbounddata.second.broken)
                                    irrlicht::tools::drawSegment(vsys.get(), mnodeA->GetPos(), mnodeB->GetPos(), ChColor(1, 0, 0), true);
                                //else
                                 //   irrlicht::tools::drawSegment(vsys.get(), mnodeA->GetPos(), mnodeB->GetPos(), ChColor(0, 0.5, 1), true);
                            }
                        }
                    }

                }
            }

        } // end loop on physics items for visualization tricks
        
        vsys->EndScene();
        
        mphysicalSystem.DoStepDynamics(timestep);
        
        if (mphysicalSystem.GetNumSteps() % 5 == 0)
            blender_exporter.ExportData();
    }


    return 0;
}
