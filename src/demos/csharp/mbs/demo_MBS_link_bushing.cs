// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Antonio Recuero, Alessandro Tasora, Radu Serban, Josh Diyn
// =============================================================================
//
// Demonstration of flexible bushing between two bodies
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

using System;
using static ChronoGlobals;

namespace ChronoDemo
{
    class FlexibleBushingDemo
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + ChronoGlobals.CHRONO_VERSION);

            // Select one of the following examples:
            //
            // 1 - ChLoadBodyBodyBushingGeneric
            //     This type of bushing requires two 6x6 matrices for generic stiffness and damping, for both translation and
            //     rotation. Optionally, it also supports initial pre-displacement and pre-stress
            // 2 - ChLoadBodyBodyBushingMate
            //     This type of bushing is like a simplified version of ChLoadBodyBodyBushingGeneric, it adds compliance to both
            //     translation and rotation, using three x y z and three Rx Ry Rz stiffness values.
            // 3 - ChLoadBodyBodyBushingPlastic
            //     A special type of ChLoadBodyBodyBushingSpherical that also provides plastic deformation with a plastic yeld.
            // 4 - ChLinkBushing
            //     The ChLinkBushing is inherited from the ChLink classes. Differently from the previous example, it does NOT
            //     support stiffness matrices, so it should NOT be used for very stiff problems.
            // 5 - ChLoadBodyBodyBushingGeneric
            //     No stiffness and damping in one rotational direction

            Console.WriteLine("1  : ChLoadBodyBodyBushingGeneric");
            Console.WriteLine("     This type of bushing requires two 6x6 matrices for generic stiffness and damping,");
            Console.WriteLine("     for both translation and rotation. Optionally, it also supports initial");
            Console.WriteLine("     pre-displacement and pre-stress");
            Console.WriteLine("2  : ChLoadBodyBodyBushingMate");
            Console.WriteLine("     This type of bushing is like a simplified version of ChLoadBodyBodyBushingGeneric,");
            Console.WriteLine("     it adds compliance to both translation and rotation, using three x y z and three");
            Console.WriteLine("     Rx Ry Rz stiffness values.");
            Console.WriteLine("3  : ChLoadBodyBodyBushingPlastic");
            Console.WriteLine("     A special type of ChLoadBodyBodyBushingSpherical that also provides plastic deformation");
            Console.WriteLine("     with a plastic yeld.");
            Console.WriteLine("4  : ChLinkBushing");
            Console.WriteLine("     The ChLinkBushing is inherited from the ChLink classes. Differently from the previous example,");
            Console.WriteLine("     it does NOT support stiffness matrices, so it should NOT be used for very stiff problems.");
            Console.WriteLine("5  : ChLoadBodyBodyBushingGeneric");
            Console.WriteLine("     No stiffness and damping in one rotational direction");

            Console.WriteLine("\nSelect option (1-5): ");
            string example = Console.ReadLine();

            // Set the path to the Chrono data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            // Create the system
            ChSystemNSC sys = new ChSystemNSC();
            sys.SetGravitationalAcceleration(new ChVector3d(0, 0, -9.81));

            // Create the ground body
            ChBody ground = new ChBodyEasyBox(0.6, 0.6, 0.15, 10000, true, false);
            sys.AddBody(ground);
            ground.SetFixed(true);

            // Create a moving body that will 'bounce' thanks to a flexible bushing.
            // Give it an initial angular velocity and attach also a small sphere to show the anchoring of the bushing.
            ChBody body = new ChBodyEasyBox(0.9, 0.9, 0.15, 1000, true, false);
            sys.Add(body);
            body.SetFixed(false);
            body.SetPos(new ChVector3d(1.0, 0.0, 0.0));
            body.SetAngVelLocal(new ChVector3d(1.5, 1.5, -1.5));
            body.SetPosDt(new ChVector3d(1.0, -0.4, 0.2));
            body.GetVisualShape(0).SetColor(new ChColor(0.6f, 0, 0));

            var symbol_bushing = new ChVisualShapeSphere(0.1);
            body.AddVisualShape(symbol_bushing, new ChFramed(new ChVector3d(-1, 0, 0), chrono.QUNIT));

            // Now create the bushing connecting the "body" to the "ground".
            // A bushing is like an invisible connection between two bodies, but differently from constraints, it has some
            // compliance.

            // Bushings are inherited from ChLoad, so they require a 'load container'
            ChLoadContainer load_container = new ChLoadContainer();
            sys.Add(load_container);

            ChMatrix66d K_matrix = new ChMatrix66d();
            ChMatrix66d R_matrix = new ChMatrix66d();

            // EXAMPLE 1: use  ChLoadBodyBodyBushingGeneric
            //
            // This type of bushing requires two 6x6 matrices for generic stiffness and damping, for both translation and
            // rotation. Optionally, it also supports initial pre-displacement and pre-stress

            K_matrix.SetZero();
            R_matrix.SetZero();
            for (int i = 0; i < 6; i++)
            {
                K_matrix.SetItem(i, i, 1e5);
                R_matrix.SetItem(i, i, 1e3);
            }

            var bushing_generic = new ChLoadBodyBodyBushingGeneric(
                body,                                         // body A
                ground,                                       // body B
                new ChFramed(new ChVector3d(0.0, 0.0, 0.0)),  // initial frame of bushing in abs space
                K_matrix,                                     // the 6x6 (translation+rotation) K matrix in local frame
                R_matrix                                      // the 6x6 (translation+rotation) R matrix in local frame
            );
            bushing_generic.SetNeutralForce(new ChVector3d(100, 0, 0));
            bushing_generic.NeutralDisplacement().SetPos(new ChVector3d(0.02, 0, 0));
            if (example.Equals("1"))
            {
                load_container.Add(bushing_generic);
            }

            // EXAMPLE 2: use  ChLoadBodyBodyBushingMate
            //
            // This type of bushing is like a simplified version of ChLoadBodyBodyBushingGeneric, it adds compliance to both
            // translation and rotation, using three x y z and three Rx Ry Rz stiffness values.

            var bushing_mate = new ChLoadBodyBodyBushingMate(
                body,                                         // body A
                ground,                                       // body B
                new ChFramed(new ChVector3d(0.0, 0.0, 0.0)),  // initial frame of bushing in abs space
                new ChVector3d(95000.0),                      // K stiffness in local frame  [N/m]
                new ChVector3d(100.0),                        // R damping in local frame  [N/m/s]
                new ChVector3d(95000.0),                      // K rotational stiffness,in local frame [Nm/rad]
                new ChVector3d(100.0)                         // R rotational damping, in local frame [Nm/rad/s]
            );
            if (example.Equals("2"))
            {
                load_container.Add(bushing_mate);
            }

            // EXAMPLE 3: use  ChLoadBodyBodyBushingPlastic
            //
            // A special type of ChLoadBodyBodyBushingSpherical that also provides plastic deformation with a plastic yeld.

            var bushing_plastic = new ChLoadBodyBodyBushingPlastic(
                body,                                         // body A
                ground,                                       // body B
                new ChFramed(new ChVector3d(0.0, 0.0, 0.0)),  // initial frame of bushing in abs space
                new ChVector3d(95000.0),                      // K stiffness in local frame  [N/m]
                new ChVector3d(100.0),                        // R damping in local frame  [N/m/s]
                new ChVector3d(18000.0)                       // plastic yield [N/m]
            );
            if (example.Equals("3"))
            {
                load_container.Add(bushing_plastic);
            }

            // EXAMPLE 4: use  ChLinkBushing
            //
            // Note, the ChLinkBushing is inherited from the ChLink classes. Differently from the previous example, it does NOT
            // support stiffness matrices, so it should NOT be used for very stiff problems. This behaves in various ways
            // according to the types in its enums:
            // - ChLinkBushing::Spherical: Rotational dofs are free, translational defined by stiffness/damping matrices
            // - ChLinkBushing::Revolute: One rotational dof is free, rest of dofs defined by stiffness/damping matrices
            // - ChLinkBushing::Mount: All six dofs defined by stiffness/damping matrices

            var bushing_link = new ChLinkBushing(ChLinkBushing.Type.Mount);
            bushing_link.Initialize(
                body,                                                                        // body A
                ground,                                                                      // body B
                new ChFramed(new ChVector3d(0.0, 0.0, 0.0), new ChQuaterniond(1, 0, 0, 0)),  // initial frame of bushing in abs space
                K_matrix,                                                                    // K stiffness in local frame
                R_matrix);                                                                   // R damping in local frame

            if (example.Equals("4"))
            {
                sys.Add(bushing_link);
            }

            // EXAMPLE 5: ChLoadBodyBodyBushingGeneric
            //
            // Same as example 1, but set the rotational stiffness and damping about Y axis to 0.

            if (example.Equals("5"))
            {
                K_matrix.SetItem(4, 4, 0);
                R_matrix.SetItem(4, 4, 0);
                bushing_generic.SetStiffnessMatrix(K_matrix);
                bushing_generic.SetDampingMatrix(R_matrix);
                load_container.Add(bushing_generic);
            }

            // Finally, add a force that tends to bend the body
            var force = new ChLoadBodyForce(body, new ChVector3d(0, 0, -8e3), false, new ChVector3d(1, 0, 0));
            load_container.Add(force);

            // Create the Irrlicht visualization sys
            var vis = new ChVisualSystemIrrlicht();
            vis.AttachSystem(sys);
            vis.SetCameraVertical(CameraVerticalDir.Z);
            vis.SetWindowSize(800, 600);
            vis.SetWindowTitle("ChLinkBushing");
            vis.Initialize();
            vis.AddLogo();
            vis.AddSkyBox();
            vis.AddCamera(new ChVector3d(3, 3, 0));
            vis.AddTypicalLights();

            vis.EnableBodyFrameDrawing(true);

            // Change some solver settings

            // Barzilai-Borwein does not support stiffness matrics => explicit integration for stiff bushings
            ////sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

            // MINRES (or GMRES) support stiffness matrices => implicit integration of the stiff bushings
            var solver = new ChSolverMINRES();
            sys.SetSolver(solver);
            solver.SetMaxIterations(200);
            solver.SetTolerance(1e-10);
            solver.EnableDiagonalPreconditioner(true);
            solver.EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
            //solver.SetVerbose(false);

            sys.SetTimestepperType(ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED);

            // Simulation loop
            while (vis.Run())
            {
                vis.BeginScene();
                vis.Render();
                vis.EndScene();
                sys.DoStepDynamics(1e-4);
            }

            return;
        }
    }
}

