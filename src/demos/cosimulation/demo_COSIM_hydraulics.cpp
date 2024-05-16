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
// Advanced demo showing how to implement a cosimulation with Chrono and
// Simulink. The SimHydraulics toolbox of Simulink is used here to simulate a
// hydraulic circuit that interacts with a Chrono mechanism.
//
// This example needs test_cosim_hydraulics.mdl to be loaded and run in Simulink
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChSocketCommunication.h"

using namespace chrono;
using namespace chrono::utils;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\n"
              << "Chrono version: " << CHRONO_VERSION << std::endl;

    std::cout << "CHRONO SimHydraulics cosimulation\n" << std::endl;
    std::cout << "NOTE! This requires a copy of Simulink with SimHydraulics.\n" << std::endl;

    try {
        // Test
        // Create a cosimulation interface for simulating an hydraulic
        // system in Simulink, connected to a mechanical system in
        // Chrono::Engine.
        // It requires a license of the SimScape/SimHydraulics Matlab toolbox.

        // 1) Create a very simple ChronoEngine system, without user interface

        // The physical system: it contains all physical objects.
        ChSystemNSC my_system;

        // Create rigid bodies and add them to the system:
        auto my_body_A = chrono_types::make_shared<ChBody>();  // truss
        my_body_A->SetFixed(true);                             // truss does not move!
        my_system.AddBody(my_body_A);

        auto my_body_B = chrono_types::make_shared<ChBody>();  // moving body
        my_body_B->SetMass(114);
        my_body_B->SetInertiaXX(ChVector3d(50, 50, 50));
        my_body_B->SetPos(ChVector3d(1, 1, 0));
        my_system.AddBody(my_body_B);

        // Now create a mechanical link (a revolute joint in 0,0,0)
        // between these two markers, and insert in system:
        auto my_link_BA = chrono_types::make_shared<ChLinkLockRevolute>();
        my_link_BA->Initialize(my_body_B, my_body_A, ChFrame<>(ChVector3d(0, 1, 0)));
        my_system.AddLink(my_link_BA);

        // Now create a 'dead' linear actuator between two points using a ChLinkTSDA with zero stiffness and damping.
        // This will be used to apply the force between the two bodies as a cylinder with spherical ball ends.
        auto my_link_actuator = chrono_types::make_shared<ChLinkTSDA>();
        my_link_actuator->Initialize(my_body_B, my_body_A, false, ChVector3d(1, 0, 0), ChVector3d(1, 1, 0));
        my_link_actuator->SetSpringCoefficient(0);
        my_link_actuator->SetDampingCoefficient(0);
        my_link_actuator->SetRestLength(my_link_actuator->GetLength());
        my_system.AddLink(my_link_actuator);

        // Create also a spring-damper to have some load when moving:
        auto my_link_springdamper = chrono_types::make_shared<ChLinkTSDA>();
        my_link_springdamper->Initialize(my_body_B, my_body_A, false, ChVector3d(1, 0, 0), ChVector3d(1, 1, 0));
        my_link_springdamper->SetSpringCoefficient(4450);
        my_link_springdamper->SetDampingCoefficient(284);
        my_link_springdamper->SetRestLength(my_link_springdamper->GetLength());
        my_system.AddLink(my_link_springdamper);

        my_system.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
        my_system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        my_system.GetSolver()->AsIterative()->SetMaxIterations(20);

        // 2) Add a socket framework object
        ChSocketFramework socket_tools;

        // 3) Create the cosimulation interface:

        ChSocketCommunication cosimul_interface(socket_tools,
                                                1,   // n.input values from Simulink
                                                2);  // n.output values to Simulink

        // Prepare the two column vectors of data that will be swapped
        // back and forth between Chrono and Simulink. In detail we will
        // - receive 1 variable from Simulink (the hydraulic cylinder force)
        // - send 2 variables to Simulink (the hydraulic cylinder velocity and displacement)
        ChVectorDynamic<double> data_in(1);
        ChVectorDynamic<double> data_out(2);
        data_in.setZero();
        data_out.setZero();

        // 4) Wait client (Simulink) to connect...

        std::cout << " *** Waiting Simulink to start... ***\n"
                  << "(load 'data/cosimulation/test_cosim_hydraulics.mdl' in Simulink and press Start...)\n"
                  << std::endl;

        int PORT_NUMBER = 50009;

        cosimul_interface.WaitConnection(PORT_NUMBER);

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

            data_out(0) = my_link_actuator->GetVelocity();
            data_out(1) = my_link_actuator->GetDeformation();

            // std::cout << "Send" << std::endl;
            cosimul_interface.SendData(mytime, data_out);  // --> to Simulink

            // B.2) - RECEIVE data

            // std::cout << "Receive" << std::endl;
            cosimul_interface.ReceiveData(histime, data_in);  // <-- from Simulink

            // - Update the Chrono system with the force value that we received
            //   from Simulink using the data_in vector, that contains:
            //      * the force of the hydraulic actuator

            my_link_actuator->SetActuatorForce(data_in(0));

            std::cout << "--- time: " << mytime << std::endl;
        }

    } catch (std::exception exception) {
        std::cerr << " ERRROR with socket system:\n" << exception.what() << std::endl;
    }

    return 0;
}
