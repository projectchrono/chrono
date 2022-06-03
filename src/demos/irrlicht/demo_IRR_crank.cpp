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

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // 1- Create a Chrono physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.

    ChSystemNSC sys;

    // 2- Create the rigid bodies of the slider-crank mechanical system
    //   (a crank, a rod, a truss), maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the truss
    auto my_body_A = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_A);
    my_body_A->SetBodyFixed(true);  // truss does not move!
    my_body_A->SetName("Ground-Truss");

    // ..the crank
    auto my_body_B = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_B);
    my_body_B->SetPos(ChVector<>(1, 0, 0));  // position of COG of crank
    my_body_B->SetMass(2);
    my_body_B->SetName("Crank");

    // ..the rod
    auto my_body_C = chrono_types::make_shared<ChBody>();
    sys.AddBody(my_body_C);
    my_body_C->SetPos(ChVector<>(4, 0, 0));  // position of COG of rod
    my_body_C->SetMass(3);
    my_body_C->SetName("Rod");

    // 3- Create constraints: the mechanical joints between the rigid bodies.

    // .. a revolute joint between crank and rod
    auto my_link_BC = chrono_types::make_shared<ChLinkLockRevolute>();
    my_link_BC->SetName("RevJointCrankRod");
    my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2, 0, 0)));
    sys.AddLink(my_link_BC);

    // .. a slider joint between rod and truss
    auto my_link_CA = chrono_types::make_shared<ChLinkLockPointLine>();
    my_link_CA->SetName("TransJointRodGround");
    my_link_CA->Initialize(my_body_C, my_body_A, ChCoordsys<>(ChVector<>(6, 0, 0)));
    sys.AddLink(my_link_CA);

    // .. a motor between crank and truss
    auto my_link_AB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_link_AB->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector<>(0, 0, 0)));
    my_link_AB->SetName("RotationalMotor");
    sys.AddLink(my_link_AB);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
    my_link_AB->SetSpeedFunction(my_speed_function);

    // 4- Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Simple slider-crank example");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, -6));
    vis->AddTypicalLights();

    // Simulation loop

    // Timer for enforcing soft real-time
    ChRealtimeStepTimer realtime_timer;
    double time_step = 0.01;

    // bool removed = false;

    while (vis->Run()) {
        // Irrlicht must prepare frame to draw
        vis->BeginScene();

        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw items belonging to Irrlicht scene, if any
        vis->DrawAll();
        // .. draw a grid
        tools::drawGrid(vis.get(), 0.5, 0.5);
        // .. draw GUI items belonging to Irrlicht screen, if any
        vis->GetGUIEnvironment()->drawAll();

        // .. draw the rod (from joint BC to joint CA)
        tools::drawSegment(vis.get(), my_link_BC->GetMarker1()->GetAbsCoord().pos,
                           my_link_CA->GetMarker1()->GetAbsCoord().pos, ChColor(0, 1, 0));
        // .. draw the crank (from joint AB to joint BC)
        tools::drawSegment(vis.get(), my_link_AB->GetLinkAbsoluteCoords().pos,
                           my_link_BC->GetMarker1()->GetAbsCoord().pos, ChColor(1, 0, 0));
        // .. draw a small circle at crank origin
        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));

        /* test: delete a link after 10 seconds
        if (sys.GetChTime() >10 && (!removed))
        {
                sys.RemoveLink(my_link_AB);
                removed = true;
        }*/

        // ADVANCE SYSTEM STATE BY ONE STEP
        sys.DoStepDynamics(time_step);
        // Enforce soft real-time
        realtime_timer.Spin(time_step);

        // Irrlicht must finish drawing the frame
        vis->EndScene();
    }

    return 0;
}
