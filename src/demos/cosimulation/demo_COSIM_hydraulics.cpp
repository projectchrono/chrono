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
// Advanced demo showing how to implement a cosimulation with Chrono and
// Simulink. The SimHydraulics toolbox of Simulink is used here to simulate a
// hydraulic circuit that interacts with a Chrono mechanism.
//
// This example needs test_cosim_hydraulics.mdl to be loaded and run in Simulink
//
// =============================================================================

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_cosimulation/ChCosimulation.h"
#include "chrono_cosimulation/ChExceptionSocket.h"

using namespace chrono;
using namespace chrono::cosimul;

int main(int argc, char* argv[]) {
    // To write something to the console, use the chrono::GetLog()

    GetLog() << "CHRONO SimHydraulics cosimulation \n\n";
    GetLog() << "NOTE! This requires a copy of Simulink with SimHydraulics. \n\n";

    try {
        // Test
        // Create a cosimulation interface for simulating an hydraulic
        // system in Simulink, connected to a mechanical system in
        // Chrono::Engine.
        // It requires a license of the SimScape/SimHydraulics Matlab toolbox.

        // 1) Create a very simple ChronoEngine system, without user interface

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create rigid bodies and add them to the system:
        auto my_body_A = std::make_shared<ChBody>();  // truss
        my_body_A->SetBodyFixed(true);          // truss does not move!
        my_system.AddBody(my_body_A);

        auto my_body_B = std::make_shared<ChBody>();  // moving body
        my_body_B->SetMass(114);
        my_body_B->SetInertiaXX(ChVector<>(50, 50, 50));
        my_body_B->SetPos(ChVector<>(1, 1, 0));
        my_system.AddBody(my_body_B);

        // Now create a mechanical link (a revolute joint in 0,0,0)
        // between these two markers, and insert in system:
        auto my_link_BA = std::make_shared<ChLinkLockRevolute>();
        my_link_BA->Initialize(my_body_B, my_body_A, ChCoordsys<>(ChVector<>(0, 1, 0)));
        my_system.AddLink(my_link_BA);

        // Now create a 'dead' linear actuator between two points using a
        // ChLinkSpring with zero stiffness and damping. This will
        // be used to apply the force between the two bodies as a
        // cylinder with spherical ball ends.
        auto my_link_actuator = std::make_shared<ChLinkSpring>();
        my_link_actuator->Initialize(my_body_B, my_body_A, false, ChVector<>(1, 0, 0), ChVector<>(1, 1, 0));
        my_link_actuator->Set_SpringK(0);
        my_link_actuator->Set_SpringR(0);
        my_link_actuator->Set_SpringRestLength(my_link_actuator->GetDist());
        my_system.AddLink(my_link_actuator);

        // Create also a spring-damper to have some load when moving:
        auto my_link_springdamper = std::make_shared<ChLinkSpring>();
        my_link_springdamper->Initialize(my_body_B, my_body_A, false, ChVector<>(1, 0, 0), ChVector<>(1, 1, 0));
        my_link_springdamper->Set_SpringK(4450);
        my_link_springdamper->Set_SpringR(284);
        my_link_springdamper->Set_SpringRestLength(my_link_springdamper->GetDist());
        my_system.AddLink(my_link_springdamper);

        my_system.Set_G_acc(ChVector<>(0, 0, 0));
        my_system.SetMaxItersSolverSpeed(20);
        my_system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

        // 2) Add a socket framework object
        ChSocketFramework socket_tools;

        // 3) Create the cosimulation interface:

        ChCosimulation cosimul_interface(socket_tools,
                                         1,   // n.input values from Simulink
                                         2);  // n.output values to Simulink

        // Prepare the two column vectors of data that will be swapped
        // back and forth between Chrono and Simulink. In detail we will
        // - receive 1 variable from Simulink (the hydraulic cylinder force)
        // - send 2 variables to Simulink (the hydraulic cylinder velocity and displacement)
        ChMatrixDynamic<double> data_in(1, 1);
        ChMatrixDynamic<double> data_out(2, 1);

        // 4) Wait client (Simulink) to connect...

        GetLog() << " *** Waiting Simulink to start... *** \n     (load 'data/cosimulation/test_cosim_hydraulics.mdl' "
                    "in Simulink and press Start...)\n\n";

        int PORTNUM = 50009;

        cosimul_interface.WaitConnection(PORTNUM);

        double mytime = 0;
        double histime = 0;

        // Here the 'dt' must be the same of the sampling period that is
        // entered in the CEcosimulation block

        double dt = 0.001;

        // 5) Run the co-simulation

        while (true) {
            // A) ----------------- ADVANCE THE Chrono SIMULATION

            if (dt > 0)
                my_system.DoStepDynamics(dt);

            mytime += dt;

            // B) ----------------- SYNCHRONIZATION

            // B.1) - SEND data

            // - Set the Chrono variables into the vector that must
            //   be sent to Simulink at the next timestep:
            //      * the velocity of the hydraulic actuator
            //      * the displacement of the hydraulic actuator

            data_out(0) = my_link_actuator->GetDist_dt();
            data_out(1) = my_link_actuator->GetDist() -
                          my_link_actuator->Get_SpringRestLength();  // subtract initial length so starts at 0.

            // GetLog() << "Send \n";
            cosimul_interface.SendData(mytime, &data_out);  // --> to Simulink

            // B.2) - RECEIVE data

            // GetLog() << "Receive \n";
            cosimul_interface.ReceiveData(histime, &data_in);  // <-- from Simulink

            // - Update the Chrono system with the force value that we received
            //   from Simulink using the data_in vector, that contains:
            //      * the force of the hydraulic actuator

            my_link_actuator->Set_SpringF(data_in(0));

            GetLog() << "--- time: " << mytime << "\n";
        }

    } catch (ChExceptionSocket exception) {
        GetLog() << " ERRROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}
