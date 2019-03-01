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
//  Demo code about
//
//  - constraints and 'motor' objects
//  - using IRRLICHT as a realtime 3D viewer of a slider-crank mechanism
//    simulated with Chrono::Engine.
//  - using the real-time step.
//  
// This is just a possible method of integration of Chrono::Engine + Irrlicht;
// many others are possible.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
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

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // 1- Create a Chrono physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.

    ChSystemNSC my_system;

    // 2- Create the rigid bodies of the slider-crank mechanical system
    //   (a crank, a rod, a truss), maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the truss
    auto my_body_A = std::make_shared<ChBody>();
    my_system.AddBody(my_body_A);
    my_body_A->SetBodyFixed(true);  // truss does not move!
    my_body_A->SetName("Ground-Truss");

    // ..the crank
    auto my_body_B = std::make_shared<ChBody>();
    my_system.AddBody(my_body_B);
    my_body_B->SetPos(ChVector<>(1, 0, 0));  // position of COG of crank
    my_body_B->SetMass(2);
    my_body_B->SetName("Crank");

    // ..the rod
    auto my_body_C = std::make_shared<ChBody>();
    my_system.AddBody(my_body_C);
    my_body_C->SetPos(ChVector<>(4, 0, 0));  // position of COG of rod
    my_body_C->SetMass(3);
    my_body_C->SetName("Rod");

    // 3- Create constraints: the mechanical joints between the rigid bodies.

    // .. a revolute joint between crank and rod
    auto my_link_BC = std::make_shared<ChLinkLockRevolute>();
    my_link_BC->SetName("RevJointCrankRod");
    my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2, 0, 0)));
    my_system.AddLink(my_link_BC);

    // .. a slider joint between rod and truss
    auto my_link_CA = std::make_shared<ChLinkLockPointLine>();
    my_link_CA->SetName("TransJointRodGround");
    my_link_CA->Initialize(my_body_C, my_body_A, ChCoordsys<>(ChVector<>(6, 0, 0)));
    my_system.AddLink(my_link_CA);

    // .. a motor between crank and truss
    auto my_link_AB = std::make_shared<ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector<>(0, 0, 0)));
    my_link_AB->SetName("RotationalMotor");
    my_system.AddLink(my_link_AB);
    auto my_speed_function = std::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);
    


    // 4- Create the Irrlicht visualization
    ChIrrApp application(&my_system, L"Simple slider-crank example", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6));

    // Bind assets
    application.AssetBindAll();
    application.AssetUpdateAll();


    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    // This will help choosing an integration step which matches the
    // real-time step of the simulation..
    ChRealtimeStepTimer m_realtime_timer;

    bool removed = false;

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw items belonging to Irrlicht scene, if any
        application.DrawAll();
        // .. draw a grid
        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.5, 0.5);
        // .. draw GUI items belonging to Irrlicht screen, if any
        application.GetIGUIEnvironment()->drawAll();

        // .. draw the rod (from joint BC to joint CA)
        ChIrrTools::drawSegment(application.GetVideoDriver(), my_link_BC->GetMarker1()->GetAbsCoord().pos,
                                my_link_CA->GetMarker1()->GetAbsCoord().pos, video::SColor(255, 0, 255, 0));
        // .. draw the crank (from joint AB to joint BC)
        ChIrrTools::drawSegment(application.GetVideoDriver(), my_link_AB->GetLinkAbsoluteCoords().pos,
                                my_link_BC->GetMarker1()->GetAbsCoord().pos, video::SColor(255, 255, 0, 0));
        // .. draw a small circle at crank origin
        ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));

        /* test: delete a link after 10 seconds
        if (my_system.GetChTime() >10 && (!removed))
        {
                my_system.RemoveLink(my_link_AB);
                removed = true;
        }*/

        // HERE CHRONO INTEGRATION IS PERFORMED: THE
        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
        // STEP:

        my_system.DoStepDynamics(m_realtime_timer.SuggestSimulationStep(0.02));

        // Irrlicht must finish drawing the frame
        application.EndScene();
    }

    return 0;
}
