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

#include "chrono/physics/ChMatterPeridynamics.h"
#include "chrono/physics/ChProximityContainerPeridynamics.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChLinkPointFrame.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

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
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Set small collision envelopes for objects that will be created from now on
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
    mphysicalSystem.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);


    // CREATE A FLOOR
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.4f);
    mat->SetCompliance(0.0);
    mat->SetComplianceT(0.0);
    mat->SetDampingF(0.2f);

    auto mfloorBody = chrono_types::make_shared<ChBodyEasyBox>(20,1,20,1000,true,true,mat);
    mphysicalSystem.Add(mfloorBody);
    mfloorBody->SetBodyFixed(true);
    mfloorBody->SetPos(ChVector<>(0, -5, 0));

    mfloorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg").c_str());


    // CREATE A SPHERE PRESSING THE MATERIAL:
    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(2, 1000, true,true, mat);
    mphysicalSystem.Add(msphere);
    msphere->SetPos(ChVector<>(0, -0.5, 0));

   

    // CREATE THE PERIDYNAMIC CONTINUUM

    if (false) {
        // Create elastoplastic matter
        auto mymatter = chrono_types::make_shared<ChMatterPeridynamics>();

        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        mymatter->FillBox(ChVector<>(4, 2, 4),                          // size of box
            4.0 / 25.0,                                   // resolution step
            1000,                                         // initial density
            ChCoordsys<>(ChVector<>(0, -3.9, 0), QUNIT),  // position & rotation of box
            false,                                         // do a centered cubic lattice initial arrangement
            2.1);                                         // set the horizon radius (as multiples of step)

        GetLog() << "Added " << mymatter->GetNnodes() << " nodes \n";

        // Set some material properties of the Peridynamics matter

        auto mmaterial = chrono_types::make_shared<ChContinuumDruckerPrager>();
        mymatter->ReplaceMaterial(mmaterial);
        mmaterial->Set_v(0.35);
        mmaterial->Set_E(50000.0);
        mmaterial->Set_elastic_yeld(0);
        mmaterial->Set_alpha(30 * CH_C_DEG_TO_RAD);
        // mmaterial->Set_from_MohrCoulomb(30 * CH_C_DEG_TO_RAD, 1000);
        mmaterial->Set_dilatancy(30 * CH_C_DEG_TO_RAD);
        mmaterial->Set_flow_rate(50000000.0);
        mmaterial->Set_hardening_limit(mmaterial->Get_elastic_yeld());
        mmaterial->Set_hardening_speed(100000000);

        /*
            auto mmaterial = chrono_types::make_shared<ChContinuumPlasticVonMises>();
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

        // Join some nodes of Peridynamics matter to a ChBody
        /*
        auto mnodes = mymatter;
        for (int ij = 0; ij < 120; ij++)
        {
            auto myjointnodebody = chrono_types::make_shared<ChLinkPointFrame>();
            myjointnodebody->Initialize(mnodes,
                                        ij,
                                        mfloorBody);
            mphysicalSystem.Add(myjointnodebody);
        }
        */

        // IMPORTANT!
        // This takes care of the interaction between the particles of the Peridynamics material
        auto my_sph_proximity = chrono_types::make_shared<ChProximityContainerPeridynamics>();
        mphysicalSystem.Add(my_sph_proximity);





        // This will setup the elastic vs. proximal state of nodes.
        // Important: do this as last thing before running the while() loop, otherwise
        // if you add other collision objects after this, they will be collision-transparent.
        mymatter->SetupInitialBonds();
    }


    //--------------NEW PERIDYNAMIC STUFF

    // Create peridynamic matter
    auto mymattersprings = chrono_types::make_shared<ChMatterPeriSprings>();
    mymattersprings->k = 3000;
    mphysicalSystem.Add(mymattersprings);

    // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
    mymattersprings->FillBox(ChVector<>(4, 2, 4),                          // size of box
                      4.0 / 25.0,                                   // resolution step
                      1000,                                         // initial density
                      ChCoordsys<>(ChVector<>(0, -3.9, 0), QUNIT),  // position & rotation of box
                      false,                                         // do a centered cubic lattice initial arrangement
                      1.6);                                         // set the horizon radius (as multiples of step)
                      
    // IMPORTANT!
    // This takes care of the interaction between the particles of the Peridynamics material
    auto my_peri_proximity = chrono_types::make_shared<ChProximityContainerPeri>();
    mphysicalSystem.Add(my_peri_proximity);

    my_peri_proximity->AddMatter(mymattersprings);


    my_peri_proximity->SetupInitialBonds();



    // Create the Irrlicht visualization system
    auto vsys = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vsys->AttachSystem(&mphysicalSystem);
    vsys->SetWindowSize(1024, 768);
    vsys->SetWindowTitle("Peridynamics test");
    vsys->Initialize();
    vsys->AddLogo();
    vsys->AddSkyBox();
    vsys->AddCamera(ChVector<>(-6, 0.3, 2.3), ChVector<>(0, -4, 0));
    vsys->AddLight(ChVector<>(30, 30, 60), 120, ChColor(0.6f, 0.6f, 0.6f));
    vsys->AddLight(ChVector<>(40, 60, 30), 120, ChColor(0.6f, 0.6f, 0.6f));


    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverMaxIterations(25);  // lower the LCP iters, no needed here

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    double timestep = 0.002;

    while (vsys->Run()) {
        vsys->BeginScene();
        vsys->Render();
        
        if(true)
        for  (auto& myiter : mphysicalSystem.Get_otherphysicslist()) {

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
            }

            if (auto mysmatter = std::dynamic_pointer_cast<ChMatterPeriSprings>(myiter)) {
                for (const auto& mnodedata : mysmatter->GetMapOfNodes()) {
                    auto mnode = mnodedata.second.node;

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
                    /*
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
                    */
                    // GetLog() << "Mass i="<< ip << "   m=" << mnode->GetMass() << "\n";
                    // irrlicht::tools::drawSegment(vsys.get(), mnode->GetPos(),
                    // mnode->GetPos()+(mnode->UserForce * 0.1), ChColor(0, 0, 0),false);
                }

            }

        }
        
        vsys->EndScene();
        
        mphysicalSystem.DoStepDynamics(timestep);


    }


    return 0;
}
