// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
//   Demo code about
//
//     - creating a physical system
//     - add/remove rigid bodies
//     - create mechanical joints between bodies
//	   - perform a simulation
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    {
        //
        // EXAMPLE 1:
        //

        GetLog() << " Example: create a physical system.. \n";

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create a bunch of rigid bodies..
        // Note that we use shared pointers, so you don't
        // have to care about the deletion (never use delete.. for
        // objects managed with shared pointers! it will be automatic!)
        auto my_body_A = std::make_shared<ChBody>();
        auto my_body_B = std::make_shared<ChBody>();
        auto my_body_C = std::make_shared<ChBody>();

        // Create some markers..
        // Markers are 'auxiliary coordinate systems' to be added
        // to rigid bodies.
        // Again, note that they are managed by shared pointers.
        auto my_marker_a1 = std::make_shared<ChMarker>();
        auto my_marker_a2 = std::make_shared<ChMarker>();
        auto my_marker_b1 = std::make_shared<ChMarker>();
        auto my_marker_b2 = std::make_shared<ChMarker>();

        // You can create some forces too...
        auto my_force_a1 = std::make_shared<ChForce>();
        auto my_force_a2 = std::make_shared<ChForce>();

        // Here you will add forces and markers to rigid
        // bodies.
        // Note: the same marker shouldn't be added to multiple bodies.
        my_body_A->AddMarker(my_marker_a1);
        my_body_A->AddMarker(my_marker_a2);
        my_body_A->AddForce(my_force_a1);
        my_body_A->AddForce(my_force_a2);
        my_body_B->AddMarker(my_marker_b1);
        my_body_B->AddMarker(my_marker_b2);

        // Ok, remember that rigid bodies must be added to
        // the physical system.
        my_system.AddBody(my_body_A);
        my_system.AddBody(my_body_B);
        my_system.AddBody(my_body_C);

        // Show the hierarchy in the shell window...
        GetLog() << "Here's the system hierarchy which you built: \n\n ";
        my_system.ShowHierarchy(GetLog());

        // Do you want to remove items? Use the
        // Remove...() functions.
        my_body_A->RemoveAllForces();

        // Remove a single body..
        my_system.RemoveBody(my_body_A);

        // Add markers to another body...
        my_body_B->AddMarker(my_marker_a1);
        my_body_B->AddMarker(my_marker_a2);
        my_body_B->AddForce(my_force_a1);
        my_body_B->AddForce(my_force_a2);

        // By the way, you can set an Ascii name for objects as desired:
        my_marker_a1->SetName("JohnFoo");
        // ..so you can later use  my_body_B.SearchMarker("JohnFoo"); etc.

        GetLog() << "\n\n\nHere's the system hierarchy after modifications: \n\n ";
        my_system.ShowHierarchy(GetLog());
    }

    {
        //
        // EXAMPLE 2:
        //

        GetLog() << " Example: create a slider-crank system: \n";

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create three rigid bodies and add them to the system:
        auto my_body_A = std::make_shared<ChBody>();
        auto my_body_B = std::make_shared<ChBody>();
        auto my_body_C = std::make_shared<ChBody>();

        my_body_A->SetName("truss");
        my_body_B->SetName("crank");
        my_body_C->SetName("rod");

        my_system.AddBody(my_body_A);
        my_system.AddBody(my_body_B);
        my_system.AddBody(my_body_C);

        // Set initial position of the bodies (center of mass)
        my_body_A->SetBodyFixed(true);  // truss does not move!
        my_body_B->SetPos(ChVector<>(1, 0, 0));
        my_body_C->SetPos(ChVector<>(4, 0, 0));

        // Create two markers and add them to two bodies:
        // they will be used as references for 'rod-crank'link.
        auto my_marker_b = std::make_shared<ChMarker>();
        auto my_marker_c = std::make_shared<ChMarker>();

        my_marker_b->SetName("crank_rev");
        my_marker_c->SetName("rod_rev");

        my_body_B->AddMarker(my_marker_b);
        my_body_C->AddMarker(my_marker_c);

        // Set absolute position of the two markers,
        // for the initial position of the 'rod-crank' link:
        my_marker_b->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2, 0, 0)));
        my_marker_c->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(2, 0, 0)));

        // Now create a mechanical link (a revolute joint)
        // between these two markers, and insert in system:
        auto my_link_BC = std::make_shared<ChLinkLockRevolute>();
        my_link_BC->Initialize(my_marker_b, my_marker_c);
        my_link_BC->SetName("REVOLUTE crank-rod");
        my_system.AddLink(my_link_BC);

        // Phew! All this 'marker' stuff is boring!
        // Note that there's an easier way to create a link,
        // without needing the two markers (they will be
        // automatically created and added to the two bodies)
        // i.e. is using two bodies and a position as arguments..
        // For example, to create the rod-truss constraint:
        auto my_link_CA = std::make_shared<ChLinkLockPointLine>();
        my_link_CA->Initialize(my_body_C, my_body_A, ChCoordsys<>(ChVector<>(6, 0, 0)));
        my_system.AddLink(my_link_CA);

        my_link_CA->GetMarker1()->SetName("rod_poinline");
        my_link_CA->GetMarker2()->SetName("truss_pointline");
        my_link_CA->SetName("POINTLINE rod-truss");

        // Now create a 'motor' link between crank and truss,
        // in 'imposed speed' mode:
        auto my_link_AB = std::make_shared<ChLinkEngine>();
        my_link_AB->Initialize(my_body_A, my_body_B, ChCoordsys<>(ChVector<>(0, 0, 0)));
        my_link_AB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_AB->Get_spe_funct()))
            mfun->Set_yconst(CH_C_PI);  // speed w=3.145 rad/sec
        my_system.AddLink(my_link_AB);

        my_link_AB->GetMarker1()->SetName("truss_engine");
        my_link_AB->GetMarker2()->SetName("crank_engine");
        my_link_AB->SetName("ENGINE truss-crank");

        GetLog() << "\n\n\nHere's the system hierarchy for slider-crank: \n\n ";
        my_system.ShowHierarchy(GetLog());

        GetLog() << "Now use an interator to scan through already-added constraints:\n\n";
        ChSystem::IteratorLinks myiter = my_system.IterBeginLinks();
        while (myiter != my_system.IterEndLinks()) {
            GetLog() << "   Link class: " << typeid((*myiter)).name()
                     << "  , leaves n.DOFs: " << (*myiter)->GetLeftDOF() << "\n";
            ++myiter;
        }

        // OK! NOW GET READY FOR THE DYNAMICAL SIMULATION!

        // A very simple simulation loop..
        double chronoTime = 0;
        while (chronoTime < 2.5) {
            chronoTime += 0.01;

            // PERFORM SIMULATION UP TO chronoTime
            my_system.DoFrameDynamics(chronoTime);

            // Print something on the console..
            GetLog() << "Time: " << chronoTime
                     << "  Slider X position: " << my_link_CA->GetMarker1()->GetAbsCoord().pos.x()
                     << "  Engine torque: " << my_link_AB->Get_mot_retorque() << "\n";
        }
    }

    return 0;
}
