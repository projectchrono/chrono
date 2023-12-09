// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Basic Chrono system demo
//
// =============================================================================

using System;
using static ChronoGlobals;

namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {
            // Set the path to the Chrono data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            ChSystemNSC sys = new ChSystemNSC();

            sys.Set_G_acc(new ChVectorD(0, -10, 0));

            // ..the truss
            ChBody my_body_A = new ChBody();
            sys.AddBody(my_body_A);
            my_body_A.SetBodyFixed(true);  // truss does not move!
            my_body_A.SetName("Ground-Truss");

            // ..the crank
            ChBody my_body_B = new ChBody();
            sys.AddBody(my_body_B);
            my_body_B.SetPos(new ChVectorD(1, 0, 0));  // position of COG of crank
            my_body_B.SetMass(2);
            my_body_B.SetName("Crank");

            // ..the rod
            ChBody my_body_C = new ChBody();
            sys.AddBody(my_body_C);
            my_body_C.SetPos(new ChVectorD(1, 0, 0));  // position of COG of rod
            my_body_C.SetMass(3);
            my_body_C.SetName("Rod");

            // 3- Create constraints: the mechanical joints between the rigid bodies.

            // .. a revolute joint between crank and rod
            ChLinkLockRevolute my_link_BC = new ChLinkLockRevolute();
            my_link_BC.SetName("RevJointCrankRod");
            my_link_BC.Initialize(my_body_B, my_body_C, new ChCoordsysD(new ChVectorD(2, 0, 0)));
            sys.AddLink(my_link_BC);

            // .. a slider joint between rod and truss
            ChLinkLockPointLine my_link_CA = new ChLinkLockPointLine();
            my_link_CA.SetName("TransJointRodGround");
            my_link_CA.Initialize(my_body_C, my_body_A, new ChCoordsysD(new ChVectorD(6, 0, 0)));
            sys.AddLink(my_link_CA);

            // .. a motor between crank and truss
            ChLinkMotorRotationSpeed my_link_AB = new ChLinkMotorRotationSpeed();
            my_link_AB.Initialize(my_body_A, my_body_B, new ChFrameD());
            my_link_AB.SetName("RotationalMotor");
            sys.AddLink(my_link_AB);
            ChFunction_Const my_speed_function = new ChFunction_Const(3.14);  // speed w=3.145 rad/sec
            my_link_AB.SetSpeedFunction(my_speed_function);

            ChMaterialSurfaceNSC mat = new ChMaterialSurfaceNSC();
            mat.SetFriction(0.5F);
            mat.SetRestitution(0.5F);

            ChBodyEasyBox myEasyBox = new ChBodyEasyBox(0.1, 0.2, 0.3, 1000, true, true, mat);
            ChBodyAuxRef myBAuxRef = new ChBodyAuxRef();
            myBAuxRef.SetBodyFixed(true);

            sys.Add(myBAuxRef);
            sys.Add(myEasyBox);

            ChLinkMateFix myMate = new ChLinkMateFix();
            ChBodyFrame converted_body = chrono.CastToChBodyFrame(myEasyBox);
            ChBodyFrame converted_body2 = chrono.CastToChBodyFrame(myBAuxRef);
            myMate.Initialize(converted_body, converted_body2, new ChFrameD(new ChVectorD(2, 0, 0)));
            sys.Add(myMate);

            // Timer for enforcing soft real-time
            ChRealtimeStepTimer realtime_timer = new ChRealtimeStepTimer();
            double time_step = 0.01;

            sys.SerializeToJSON("ChronoCSharp.json");

            while (sys.GetChTime() < 5)
            {

                sys.DoStepDynamics(time_step);
                realtime_timer.Spin(time_step);
                Console.WriteLine("Time: " + sys.GetChTime() + "  Body x: " + my_body_B.GetPos().x);

            }

            Console.WriteLine("Done");
        }
    }
}
