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
//   Demo code about
//
//     - creating a physical system
//     - add/remove rigid bodies
//     - create mechanical joints between bodies
//	   - perform a simulation
//
// =============================================================================

#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    {
        //
        // EXAMPLE 1:
        //

        std::cout << " Example: create a physical system..." << std::endl;

        // The physical system: it contains all physical objects.
        ChSystemNSC sys;

        // Create a bunch of rigid bodies..
        // Note that we use shared pointers, so you don't
        // have to care about the deletion (never use delete.. for
        // objects managed with shared pointers! it will be automatic!)
        auto my_body_A = chrono_types::make_shared<ChBody>();
        auto my_body_B = chrono_types::make_shared<ChBody>();
        auto my_body_C = chrono_types::make_shared<ChBody>();

        // Create some markers..
        // Markers are 'auxiliary coordinate systems' to be added
        // to rigid bodies.
        // Again, note that they are managed by shared pointers.
        auto my_marker_a1 = chrono_types::make_shared<ChMarker>();
        auto my_marker_a2 = chrono_types::make_shared<ChMarker>();
        auto my_marker_b1 = chrono_types::make_shared<ChMarker>();
        auto my_marker_b2 = chrono_types::make_shared<ChMarker>();

        // You can create some forces too...
        auto my_force_a1 = chrono_types::make_shared<ChForce>();
        auto my_force_a2 = chrono_types::make_shared<ChForce>();

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
        sys.AddBody(my_body_A);
        sys.AddBody(my_body_B);
        sys.AddBody(my_body_C);

        // Show the hierarchy in the shell window...
        std::cout << "Here's the system hierarchy which you built:" << std::endl;
        sys.ShowHierarchy(std::cout);

        // Do you want to remove items? Use the
        // Remove...() functions.
        my_body_A->RemoveAllForces();

        // Remove a single body..
        sys.RemoveBody(my_body_A);

        // Add markers to another body...
        my_body_B->AddMarker(my_marker_a1);
        my_body_B->AddMarker(my_marker_a2);
        my_body_B->AddForce(my_force_a1);
        my_body_B->AddForce(my_force_a2);

        // By the way, you can set an Ascii name for objects as desired:
        my_marker_a1->SetName("JohnFoo");
        // ..so you can later use  my_body_B.SearchMarker("JohnFoo"); etc.

        std::cout << std::endl << std::endl << "Here's the system hierarchy after modifications:\n" << std::endl;
        sys.ShowHierarchy(std::cout);
    }

    {
        //
        // EXAMPLE 2:
        //

        std::cout << " Example: create a slider-crank system:" << std::endl;

        // The physical system: it contains all physical objects.
        ChSystemNSC sys;

        // Create three rigid bodies and add them to the system:
        auto my_body_A = chrono_types::make_shared<ChBody>();
        auto my_body_B = chrono_types::make_shared<ChBody>();
        auto my_body_C = chrono_types::make_shared<ChBody>();

        my_body_A->SetName("truss");
        my_body_B->SetName("crank");
        my_body_C->SetName("rod");

        sys.AddBody(my_body_A);
        sys.AddBody(my_body_B);
        sys.AddBody(my_body_C);

        // Set initial position of the bodies (center of mass)
        my_body_A->SetFixed(true);  // truss does not move!
        my_body_B->SetPos(ChVector3d(1, 0, 0));
        my_body_C->SetPos(ChVector3d(4, 0, 0));

        // Create two markers and add them to two bodies:
        // they will be used as references for 'rod-crank'link.
        auto my_marker_b = chrono_types::make_shared<ChMarker>();
        auto my_marker_c = chrono_types::make_shared<ChMarker>();

        my_marker_b->SetName("crank_rev");
        my_marker_c->SetName("rod_rev");

        my_body_B->AddMarker(my_marker_b);
        my_body_C->AddMarker(my_marker_c);

        // Set absolute position of the two markers,
        // for the initial position of the 'rod-crank' link:
        my_marker_b->ImposeAbsoluteTransform(ChFrame<>(ChVector3d(2, 0, 0)));
        my_marker_c->ImposeAbsoluteTransform(ChFrame<>(ChVector3d(2, 0, 0)));

        // Now create a mechanical link (a revolute joint)
        // between these two markers, and insert in system:
        auto my_link_BC = chrono_types::make_shared<ChLinkLockRevolute>();
        my_link_BC->Initialize(my_marker_b, my_marker_c);
        my_link_BC->SetName("REVOLUTE crank-rod");
        sys.AddLink(my_link_BC);

        // Phew! All this 'marker' stuff is boring!
        // Note that there's an easier way to create a link,
        // without needing the two markers (they will be
        // automatically created and added to the two bodies)
        // i.e. is using two bodies and a position as arguments..
        // For example, to create the rod-truss constraint:
        auto my_link_CA = chrono_types::make_shared<ChLinkLockPointLine>();
        my_link_CA->Initialize(my_body_C, my_body_A, ChFrame<>(ChVector3d(6, 0, 0)));
        sys.AddLink(my_link_CA);

        my_link_CA->GetMarker1()->SetName("rod_poinline");
        my_link_CA->GetMarker2()->SetName("truss_pointline");
        my_link_CA->SetName("POINTLINE rod-truss");

        // Now create a 'motor' link between crank and truss, in 'imposed speed' mode:
        auto my_motor_AB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_motor_AB->SetName("MOTOR truss-crank");
        my_motor_AB->Initialize(my_body_A, my_body_B, ChFrame<>(ChVector3d(0, 0, 0)));
        my_motor_AB->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(CH_PI));
        sys.AddLink(my_motor_AB);

        std::cout << std::endl << std::endl << "Here's the system hierarchy for slider-crank:\n" << std::endl;
        sys.ShowHierarchy(std::cout);

        std::cout << "Now use an iterator to scan through already-added constraints:" << std::endl;
        for (auto link : sys.GetLinks()) {
            auto& rlink = *link.get();
            std::cout << "   Link class: " << typeid(rlink).name() << std::endl;
        }
        std::cout << std::endl;

        // Simulation loop
        double end_time = 2.5;
        double step_size = 0.01;

        for (double frame_time = 0.05; frame_time < end_time; frame_time += 0.05) {
            // Perform simulation up to frame_time
            sys.DoFrameDynamics(frame_time, step_size);

            std::cout << "Time: " << frame_time << "  Steps: " << sys.GetNumSteps()
                      << "  Slider X position: " << my_link_CA->GetMarker1()->GetAbsCoordsys().pos.x()
                      << "  Engine torque: " << my_motor_AB->GetMotorTorque() << std::endl;
        }
    }

    return 0;
}
