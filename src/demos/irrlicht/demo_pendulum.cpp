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
//     - creating a pendulum
//     - apply custom forces using accumulators
//     - creating constraints with limits
//     - 3D viewing with the Irrlicht library
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
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChRealtimeStep.h"

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

// This function will be used to apply forces caused by
// a rotating fan, to all objects in front of it (a simple
// exaple just to demonstrate how to apply custom forces).

void apply_fan_force(ChSystem* msystem,       // contains all bodies
                     ChCoordsys<>& fan_csys,  // pos and rotation of fan
                     double aradius,          // radius of fan
                     double aspeed,           // speed of fan
                     double adensity)         // density (heuristic)
{
    for (unsigned int i = 0; i < msystem->Get_bodylist()->size(); i++) {
        ChBody* abody = (*msystem->Get_bodylist())[i];

        // Remember to reset 'user forces accumulators':
        abody->Empty_forces_accumulators();

        // initialize speed of air (steady, if outside fan stream):
        ChVector<> abs_wind(0, 0, 0);

        // calculate the position of body COG in fan coordinates:
        ChVector<> mrelpos = fan_csys.TransformParentToLocal(abody->GetPos());
        ChVector<> mrelpos_ondisc = mrelpos;
        mrelpos_ondisc.z = 0;

        if (mrelpos.z > 0)  // if not behind fan..
            if (mrelpos_ondisc.Length() < aradius) {
                // OK! we are inside wind stream cylinder..
                // wind is directed as normal to the fan disc
                abs_wind = fan_csys.TransformLocalToParent(ChVector<>(0, 0, 1));
                // wind inside fan stream is constant speed
                abs_wind *= -aspeed;
            }

        // force proportional to relative speed body-wind
        // and fluid density (NOTE! pretty simplified physics..)
        ChVector<> abs_force = (abs_wind - abody->GetPos_dt()) * adensity;
        // apply this force at the body COG
        abody->Accumulate_force(abs_force, abody->GetPos(), false);
    }
}

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"A simple pendulum example", core::dimension2d<u32>(800, 600), false, false,
                         video::EDT_OPENGL);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    //
    // Create all the rigid bodies!!!!
    //

    // ..create the five pendulums (bodies are Irrlicht nodes of
    //   the special class ChBodySceneNode, which encapsulate ChBody items ):

    for (int k = 0; k < 5; k++) {
        double z_step = (double)k * 2.;

        // .. the truss
        ChSharedPtr<ChBodyEasyBox> mrigidBody0(new ChBodyEasyBox(5, 1, 0.5,  // x,y,z size
                                                                 100,        // density
                                                                 false,      // collide enable?
                                                                 true));     // visualization?
        mrigidBody0->SetPos(ChVector<>(0, 0, z_step));
        mrigidBody0->SetBodyFixed(true);  // the truss does not move!
        my_system.Add(mrigidBody0);

        ChSharedPtr<ChBodyEasyBox> mrigidBody1(new ChBodyEasyBox(1, 6, 1,  // x,y,z size
                                                                 1,        // density
                                                                 false,    // collide enable?
                                                                 true));   // visualization?
        mrigidBody1->SetPos(ChVector<>(0, -3, z_step));
        my_system.Add(mrigidBody1);

        ChSharedPtr<ChBodyEasyBox> mrigidBody2(new ChBodyEasyBox(1, 6, 1,  // x,y,z size
                                                                 1,        // density
                                                                 false,    // collide enable?
                                                                 true));   // visualization?
        mrigidBody2->SetPos(ChVector<>(0, -9, z_step));
        my_system.Add(mrigidBody2);

        ChSharedPtr<ChBodyEasyBox> mrigidBody3(new ChBodyEasyBox(6, 1, 1,  // x,y,z size
                                                                 1,        // density
                                                                 false,    // collide enable?
                                                                 true));   // visualization?
        mrigidBody3->SetPos(ChVector<>(3, -12, z_step));
        my_system.Add(mrigidBody3);

        //
        // Create the links between bodies!!!!
        //

        // .. a joint of type 'point on a line', with upper and lower limits on
        //    the X sliding direction, for the pendulum-ground constraint.
        ChSharedPtr<ChLinkLockPointLine> my_link_01(new ChLinkLockPointLine);
        my_link_01->Initialize(mrigidBody1, mrigidBody0, ChCoordsys<>(ChVector<>(0, 0, z_step)));

        my_link_01->GetLimit_X()->Set_active(true);
        my_link_01->GetLimit_X()->Set_max(1.0);
        my_link_01->GetLimit_X()->Set_min(-1.0);

        my_system.AddLink(my_link_01);

        // .. a spherical joint
        ChSharedPtr<ChLinkLockSpherical> my_link_12(new ChLinkLockSpherical);
        my_link_12->Initialize(mrigidBody2, mrigidBody1, ChCoordsys<>(ChVector<>(0, -6, z_step)));
        my_system.AddLink(my_link_12);

        // .. a spherical joint
        ChSharedPtr<ChLinkLockSpherical> my_link_23(new ChLinkLockSpherical);
        my_link_23->Initialize(mrigidBody3, mrigidBody2, ChCoordsys<>(ChVector<>(0, -12, z_step)));
        my_system.AddLink(my_link_23);
        
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // create a 'fan ventilator' object, using Irrlicht mesh
    // loading and handling (this object is here for aesthetical reasons,
    // it is NOT handled by Chrono::Engine).
    double fan_radius = 5.3;
    IAnimatedMesh* fanMesh = application.GetSceneManager()->getMesh(GetChronoDataFile("fan2.obj").c_str());
    IAnimatedMeshSceneNode* fanNode = application.GetSceneManager()->addAnimatedMeshSceneNode(fanMesh);
    fanNode->setScale(irr::core::vector3df((irr::f32)fan_radius, (irr::f32)fan_radius, (irr::f32)fan_radius));

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    //application.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN); // if you need a more precise CCP solver..


    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        // Irrlicht application draws all 3D objects and all GUI items
        application.DrawAll();

        // Draw also a grid on the horizontal XZ plane
        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 20, 20,
                             ChCoordsys<>(ChVector<>(0, -20, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);

        // Update the position of the spinning fan (an Irrlicht
        // node, which is here just for aesthetical reasons!)
        ChQuaternion<> my_fan_rotation;
        my_fan_rotation.Q_from_AngY(my_system.GetChTime() * -0.5);
        ChQuaternion<> my_fan_spin;
        my_fan_spin.Q_from_AngZ(my_system.GetChTime() * 4);
        ChCoordsys<> my_fan_coord(ChVector<>(12, -6, 0), my_fan_rotation);
        ChFrame<> my_fan_framerotation(my_fan_coord);
        ChFrame<> my_fan_framespin(ChCoordsys<>(VNULL, my_fan_spin));
        ChCoordsys<> my_fan_coordsys = (my_fan_framespin >> my_fan_framerotation).GetCoord();
        ChIrrTools::alignIrrlichtNodeToChronoCsys(fanNode, my_fan_coordsys);

        // Apply forces caused by fan & wind if Chrono rigid bodies are
        // in front of the fan, using a simple tutorial function (see above):
        apply_fan_force(&my_system, my_fan_coord, fan_radius, 5.2, 0.5);

        // HERE CHRONO INTEGRATION IS PERFORMED: THE
        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
        // STEP:
        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
