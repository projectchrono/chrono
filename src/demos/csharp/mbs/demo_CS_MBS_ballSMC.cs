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
// Bouncing ball demo
// Supports both Irrlicht or VSG
//
// =============================================================================

using System;
using static ChronoGlobals;
using static chrono;

namespace ChronoDemo
{
    internal class Program
    {
        // Helper method to create visualisation system
        static ChVisualSystem CreateVisualizationSystem(ChSystemSMC sys)
        {
#if CHRONO_VSG
            ChVisualSystemVSG vis = new ChVisualSystemVSG();
            vis.AttachSystem(sys);
            vis.SetWindowTitle("[C#] SMC demonstration - VSG");
            vis.SetWindowSize(new ChVector2i(800, 600));
            vis.SetWindowPosition(new ChVector2i(100, 100));
            vis.AddCamera(new ChVector3d(0, 3, -6), new ChVector3d(0, 0, 0));
            vis.SetCameraVertical(CameraVerticalDir.Y);
            vis.SetLightIntensity(1.0f);
            vis.SetLightDirection(1.5 * chrono.CH_PI_2, chrono.CH_PI_4);
            vis.EnableSkyBox();
            vis.Initialize();
            
            Console.WriteLine("Using VSG visualization");
            return vis;
#elif CHRONO_IRRLICHT
            ChVisualSystemIrrlicht vis = new ChVisualSystemIrrlicht();
            vis.SetWindowSize(800, 600);
            vis.SetWindowTitle("[C#] SMC demonstration - Irrlicht");
            vis.Initialize();
            vis.AddLogo();
            vis.AddSkyBox();
            vis.AddTypicalLights();
            vis.AddCamera(new ChVector3d(0, 3, -6));
            vis.AttachSystem(sys);
            vis.AddGrid(0.2, 0.2, 20, 20,
                        new ChCoordsysd(new ChVector3d(0, 0.11, 0), QuatFromAngleX(CH_PI_2)),
                        new ChColor(0.1f, 0.1f, 0.1f));
            
            Console.WriteLine("Using Irrlicht visualization");
            return vis;
#else
            Console.WriteLine("Error: No visualization system available!");
            Environment.Exit(1);
            return null;
#endif
        }

        // Render frame for irrlicht
        static void RenderBallFrame(ChVisualSystem vis, ChBody ball, double radius)
        {
#if CHRONO_IRRLICHT
            ChVisualSystemIrrlicht vis_irr = vis as ChVisualSystemIrrlicht;
            if (vis_irr != null)
            {
                vis_irr.RenderFrame(new ChFramed(CastToChBodyFrame(ball).GetCoordsys()), 1.2 * radius);
            }
#endif
        }

        static void Main(string[] args)
        {
            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);

            ChCollisionSystem.Type coll_type = ChCollisionSystem.Type.BULLET;

            // Set the path to the Chrono data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            // Simulation parameters
            double gravity = -9.81;
            double time_step = 0.00001;
            double out_step = 2000 * time_step;

            // Parameters for the falling ball
            double radius = 1;
            double mass = 1000;
            ChVector3d pos = new ChVector3d(0, 2, 0);
            ChQuaterniond rot = new ChQuaterniond(1, 0, 0, 0);
            ChVector3d init_vel = new ChVector3d(0, 0, 0);

            // Parameters for the containing bin
            double width = 2;
            double length = 2;
            ////double height = 1;
            double thickness = 0.1;

            // Create the system
            ChSystemSMC sys = new ChSystemSMC();

            sys.SetGravitationalAcceleration(new ChVector3d(0, gravity, 0));
            sys.SetCollisionSystemType(coll_type);

            // The following two lines are optional, since they are the default options. They are added for future reference,
            // i.e. when needed to change those models.
            sys.SetContactForceModel(ChSystemSMC.ContactForceModel.Hertz);
            sys.SetAdhesionForceModel(ChSystemSMC.AdhesionForceModel.Constant);

            // Change the default collision effective radius of curvature
            ChCollisionInfo.SetDefaultEffectiveCurvatureRadius(1);

            // Create a material (will be used by both objects)
            ChContactMaterialSMC material = new ChContactMaterialSMC();
            material.SetRestitution(0.1f);
            material.SetFriction(0.4f);
            material.SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

            // Create the falling ball
            ChBody ball = new ChBody();

            ball.SetMass(mass);
            // TODO: cannot use operator between double and ChVector3d
            //ChVector3d onev = new ChVector3d(1, 1, 1);
            //ball.SetInertiaXX(0.4 * mass * radius * radius * onev);
            ball.SetPos(pos);
            ball.SetRot(rot);
            ball.SetLinVel(init_vel);
            // ball.SetWvel_par(new ChVector3d(0,0,3));
            ball.SetFixed(false);

            ChCollisionShapeSphere sphere_coll = new ChCollisionShapeSphere(material, radius);
            ball.AddCollisionShape(sphere_coll, new ChFramed());
            ball.EnableCollision(true);

            ChVisualShapeSphere sphere_vis = new ChVisualShapeSphere(radius);


            Console.WriteLine("data file: " + chrono.GetChronoDataFile("textures/bluewhite.png"));


            sphere_vis.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"));
            sphere_vis.SetOpacity(1.0f);
            ball.AddVisualShape(sphere_vis);

            sys.AddBody(ball);

            // Create container
            ChBody bin = new ChBody();

            bin.SetMass(1);
            bin.SetPos(new ChVector3d(0, 0, 0));
            bin.SetRot(new ChQuaterniond(1, 0, 0, 0));
            bin.SetFixed(true);

            ChCollisionShapeBox box_coll = new ChCollisionShapeBox(material, width * 2, thickness * 2, length * 2);
            bin.AddCollisionShape(box_coll, new ChFramed());
            bin.EnableCollision(true);

            ChVisualShapeBox box_vis = new ChVisualShapeBox(width * 2, thickness * 2, length * 2);
            box_vis.SetColor(new ChColor(0.8f, 0.2f, 0.2f));
            box_vis.SetOpacity(0.8f);
            bin.AddVisualShape(box_vis);

            sys.AddBody(bin);

            // visualisation system depends on which Chrono module was compiled
            ChVisualSystem vis = CreateVisualizationSystem(sys);

            // The soft-real-time cycle
            double time = 0.0;
            double out_time = 0.0;

            while (vis.Run())
            {
                vis.BeginScene();
                vis.Render();
                
                RenderBallFrame(vis, ball, radius); // todo:- is this still needed?
                
                vis.EndScene();

                while (time < out_time)
                {
                    sys.DoStepDynamics(time_step);
                    time += time_step;
                }
                out_time += out_step;
            }
            // On exit with VSG, Win32 window destruction warning from VSG window is expected and is a garbage cleanup VSG issue


        }

    }
}
