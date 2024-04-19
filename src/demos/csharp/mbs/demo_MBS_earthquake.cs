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
// Authors: Josh Diyn, Radu Serban
// =============================================================================
//
//   Demo code about
//     - ChEasyBody objects
//     - collisions and contacts
//     - imposing a ground-relative motion to a body
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
            // Create a tapered column segment as a faceted convex hull.
            // For convex hulls, you just need to build a vector of points, it does not matter the order,
            // because they will be considered 'wrapped' in a convex hull anyway.
            ChBody CreateColumn(ChSystemNSC system,
                                ChCoordsysd base_pos,
                                ChContactMaterial material,
                                int col_nedges = 10,
                                double col_radius_hi = 0.45,
                                double col_radius_lo = 0.5,
                                double col_height = 3,
                                double col_density = 3000)
            {
                double col_base = 0;
                vector_ChVector3d points = new vector_ChVector3d();
                for (int i = 0; i < col_nedges; ++i)
                {
                    double alpha = chrono.CH_2PI * ((double)i / (double)col_nedges);  // polar coord
                    double x = col_radius_hi * Math.Cos(alpha);
                    double z = col_radius_hi * Math.Sin(alpha);
                    double y = col_base + col_height;
                    points.Add(new ChVector3d(x, y, z));
                }
                for (int i = 0; i < col_nedges; ++i)
                {
                    double alpha = chrono.CH_2PI * ((double)i / (double)col_nedges);  // polar coord
                    double x = col_radius_lo * Math.Cos(alpha);
                    double z = col_radius_lo * Math.Sin(alpha);
                    double y = col_base;
                    points.Add(new ChVector3d(x, y, z));
                }
                var body = new ChBodyEasyConvexHull(points, col_density, true, true, material);

                // Set segment COM 
                // (need explicit upcast to get access to the ChBodyFrame methods)
                ChCoordsysd column_COM = new ChCoordsysd(new ChVector3d(0, col_base + col_height / 2, 0));
                ChCoordsysd column_COM_abs = column_COM.TransformLocalToParent(base_pos);
                chrono.CastToChBodyFrame(body).SetCoordsys(column_COM_abs);

                system.Add(body);

                return body;
            }

            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);

            // Set the path to the Chrono data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            // Create a Chrono physical system
            ChSystemNSC sys = new ChSystemNSC();
            sys.SetCollisionSystemType(ChCollisionSystem.Type.BULLET);

            // Create a floor that is fixed (that is used also to represent the absolute reference)
            var floor_body = new ChBodyEasyBox(20, 2, 20, 3000, true, false);
            floor_body.SetPos(new ChVector3d(0, -2, 0));
            floor_body.SetFixed(true);
            floor_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"));
            sys.Add(floor_body);

            // Create the table that is subject to earthquake
            var table_mat = new ChContactMaterialNSC();

            var table_body = new ChBodyEasyBox(15, 1, 15, 3000, true, true, table_mat);
            table_body.SetPos(new ChVector3d(0, -0.5, 0));
            table_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"));
            sys.Add(table_body);

            // Create the constraint between ground and table. If no earthquake, it just
            // keeps the table in position.

            var motion_x = new ChFunctionSine(0.2, 0.6);  // amplitude, frequency, phase = 0
            var link_earthquake = new ChLinkLockLock();
            link_earthquake.Initialize(table_body, floor_body, new ChFramed(new ChVector3d(0, 0, 0)));
            link_earthquake.SetMotionX(motion_x);
            sys.Add(link_earthquake);

            // Contact material shared among all column segments
            var column_mat = new ChContactMaterialNSC();

            // Create 5 columns from 3 pieces each
            double spacing = 1.6;
            double density = 3000;

            for (int icol = 0; icol < 5; ++icol)
            {
                ChCoordsysd base_position1 = new ChCoordsysd(new ChVector3d(icol * spacing, 0, 0));
                CreateColumn(sys, base_position1, column_mat, 10, 0.45, 0.5, 1.5, density);

                ChCoordsysd base_position2 = new ChCoordsysd(new ChVector3d(icol * spacing, 1.5, 0));
                CreateColumn(sys, base_position2, column_mat, 10, 0.40, 0.45, 1.5, density);

                ChCoordsysd base_position3 = new ChCoordsysd(new ChVector3d(icol * spacing, 3.0, 0));
                ChBody top_segment = CreateColumn(sys, base_position3, column_mat, 10, 0.35, 0.40, 1.5, density);

                // Add a top body to first 4 columns (rigidly attached with a ChLinkMateFix joint)
                if (icol < 4)
                {
                    ChBody top_body = new ChBodyEasyBox(spacing, 0.4, 1.2,  // x y z sizes
                                                       density,             // density
                                                       true, true,          // visualize?, collision?
                                                       column_mat);         // contact material

                    ChCoordsysd top_COM = new ChCoordsysd(new ChVector3d(icol * spacing + spacing / 2, 4.5 + 0.4 / 2, 0));

                    // (need explicit upcast to get access to the ChBodyFrame methods)
                    chrono.CastToChBodyFrame(top_body).SetCoordsys(top_COM);
                    sys.Add(top_body);

                    // Weld the top body to the top segment
                    // (need explicit upcast to get access to the ChBodyFrame methods)
                    ChLinkMateFix weld_joint = new ChLinkMateFix();
                    weld_joint.Initialize(chrono.CastToChBodyFrame(top_segment), chrono.CastToChBodyFrame(top_body));
                    sys.AddLink(weld_joint);
                }
            }

            // Create the Irrlicht visualization sys
            var vis = new ChVisualSystemIrrlicht();
            vis.AttachSystem(sys);
            vis.SetCameraVertical(CameraVerticalDir.Y);
            vis.SetWindowSize(800, 600);
            vis.SetWindowTitle("Collisions between objects");
            vis.Initialize();
            vis.AddLogo();
            vis.AddSkyBox();
            vis.AddCamera(new ChVector3d(1, 3, -10));
            vis.AddTypicalLights();
            vis.AddLightWithShadow(new ChVector3d(1.0, 25.0, -5.0), new ChVector3d(0, 0, 0), 35, 0.2, 35, 35, 512,
                        new ChColor(0.6f, 0.8f, 1.0f));
            vis.EnableShadows();

            vis.EnableBodyFrameDrawing(true);

            // Modify some setting of the physical system for the simulation
            sys.SetSolverType(ChSolver.Type.PSOR);
            sys.GetSolver().AsIterative().SetMaxIterations(50);

            // Simulation loop
            double timestep = 0.005;
            ChRealtimeStepTimer m_realtime_timer = new ChRealtimeStepTimer();
            while (vis.Run())
            {
                vis.BeginScene();
                vis.Render();
                vis.EndScene();

                sys.DoStepDynamics(timestep);
                m_realtime_timer.Spin(timestep);
            }

            return;

        }
    }
}

