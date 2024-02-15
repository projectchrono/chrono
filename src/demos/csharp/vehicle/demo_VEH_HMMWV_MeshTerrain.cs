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
// Authors: Josh Diyn
// =============================================================================
//
// Illustration of Y-Up option using a vectorised height map/mesh as
// RigidTerrain. Method includes averaging of heights over grid squares to
// 'smooth' the terrain.
//
// The standard vehicle reference frame has Z up, X towards the front of the
// vehicle, and Y pointing to the left. This demo includes the option for
// reorientation to a Y-up world (RHF).
//
// =============================================================================

using System;
using static ChronoGlobals;
using static chrono_vehicle;

namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {
            //------------------------------------------------------------
            // Basic set up & world orientation
            //------------------------------------------------------------

            bool isYUp = false; // default declaration regarding use of the y-up world.
            Console.WriteLine("Do you want to use a Y-Up world Orientation (Y/N)? (default N):");
            string orientationInput = Console.ReadLine();
            if (orientationInput.Equals("y") && !string.IsNullOrWhiteSpace(orientationInput))
            {
                isYUp = true;
                Console.WriteLine("\n\nY-Up World selected.\n\n");
            } else {
                isYUp = false;  // default
                Console.WriteLine("\n\nDefault Z-Up Chrono world set\n\n");
            }

            // TODO: correct CHRONO_VERSION call
            //Console.WriteLine(chrono.GetLog() + "Copyright (c) 2017 projectchrono.org\nChrono version: " + CHRONO_VERSION + "\n\n");

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetDataPath(CHRONO_VEHICLE_DATA_DIR);

            // Simulation step size
            double step_size = 2e-3;

            //------------------------------------------
            // Visualisation, Tracking and Vehicle Setup
            //------------------------------------------

            if (isYUp)
            {
                ChWorldFrame.SetYUP();
            }
            // Initial vehicle location and orientation
            ChVectorD initLoc = new ChVectorD(0, (isYUp) ? 0.5 : 0, (isYUp) ? 0 : 0.5); // move away from the center

            // Create and configure the vehicle
            HMMWV_Full hmmwv = new HMMWV_Full();
            // Vehicle Collisions
            hmmwv.SetContactMethod(ChContactMethod.NSC);
            hmmwv.SetChassisCollisionType(CollisionType.HULLS); // Enable collision for the chassis
            //hmmwv.SetCollisionSystemType(ChCollisionSystem.Type.BULLET);.
            // Configure vehicle specifics
            hmmwv.SetInitPosition(new ChCoordsysD(initLoc, chrono.QUNIT));
            hmmwv.SetEngineType(EngineModelType.SHAFTS);
            hmmwv.SetTransmissionType(TransmissionModelType.AUTOMATIC_SHAFTS);
            hmmwv.SetDriveType(DrivelineTypeWV.AWD);
            hmmwv.UseTierodBodies(true);
            hmmwv.SetSteeringType(SteeringTypeWV.PITMAN_ARM);
            hmmwv.SetBrakeType(BrakeType.SHAFTS);
            hmmwv.SetTireType(TireModelType.FIALA);
            hmmwv.SetTireCollisionType(ChTire.CollisionType.SINGLE_POINT);
            hmmwv.SetTireStepSize(step_size);
            hmmwv.Initialize();
            if (isYUp) { hmmwv.GetSystem().Set_G_acc(new ChVectorD(0, -9.81, 0)); } // adjust the gravity to the world

            // Visualisation of vehicle
            hmmwv.SetChassisVisualizationType(VisualizationType.MESH);
            hmmwv.SetSteeringVisualizationType(VisualizationType.MESH);
            hmmwv.SetSuspensionVisualizationType(VisualizationType.MESH);
            hmmwv.SetWheelVisualizationType(VisualizationType.MESH);
            hmmwv.SetTireVisualizationType(VisualizationType.MESH);

            // Optionally, enable collision for the vehicle wheels.
            // In this case, you must also disable collision between the chassis and wheels (unless the chassis collision model
            // is accurate enough to account for the wheel wells).
            ////hmmwv.GetVehicle().SetWheelCollide(true);
            ////hmmwv.GetVehicle().SetChassisVehicleCollide(false);


            //------------------------------------------------------------
            // Create the terrain from mathematical/programmatic entry
            //------------------------------------------------------------
            RigidTerrain terrain = new RigidTerrain(hmmwv.GetSystem());

            var patch_mat = new ChMaterialSurfaceNSC();
            patch_mat.SetFriction(0.8f);
            patch_mat.SetRestitution(0.001f);

            vector_ChVectorD point_cloud = new vector_ChVectorD();
   
            // 1.
            // create undulating terrain from sin/cos
            // Parameters for the terrain
            int numPointsX = 1200; // Adjust for resolution
            int numPointsY = 1200; // Adjust for resolution
            double length = 50; // Total length in X
            double width = 50; // Total width in Y
            double amplitude = 3.0; // Amplitude of undulation
            double startX = -55; // Starting X position
            double startY = -15; // Starting Y position

            // Create height vectors for undulating terrain
            double dx = length / (numPointsX - 1);
            double dy = width / (numPointsY - 1);
            for (int i = 0; i < numPointsX; i++)
            {
                for (int j = 0; j < numPointsY; j++)
                {
                    double x = i * dx + startX;
                    double y = j * dy + startY;
                    double z = amplitude * Math.Sin(2 * Math.PI * x / length) * Math.Cos(2 * Math.PI * y / width);
                    point_cloud.Add(new ChVectorD(x, y, z));
                }
            }
            
            // 2.
            // generate a ramp and platform
            // ramp
            for (double x = 10; x <= 40; x += 0.05)
            {
                double height = ( (x -10)/ 30) * 5; // Linearly increasing height
                for (double y = -3; y <= 3; y += 0.05)
                {
                    point_cloud.Add(new ChVectorD(x, y, height));
                }
            }
            // Platform
            for (double x = 40; x <= 53; x += 0.05)
            {
                for (double y = -10; y <= 10; y += 0.05)
                {
                    point_cloud.Add(new ChVectorD(x, y, 5));
                }
            }

            // 3. Generate a spiral
            double centerX = -25;
            double centerY = -25;
            double spiralWidth = 60;
            int numTurns = 4;
            int pointsPerTurn = 3000;
            double vShapeWidth = 2;
            double vShapeDepth = 3;
            int vShapeDetail = 20;
            double angleIncrement = (2 * Math.PI) / pointsPerTurn;
            double radiusIncrement = spiralWidth / (2 * numTurns * pointsPerTurn);

            for (int turn = 0; turn < numTurns; turn++)
            {

                for (int point = 0; point < pointsPerTurn; point++)
                {
                    double angle = turn * 2 * Math.PI + point * angleIncrement;
                    double radius = radiusIncrement * (turn * pointsPerTurn + point);

                    for (int vPointIdx = 0; vPointIdx < vShapeDetail; vPointIdx++)
                    {
                        double vPointFraction = (double)vPointIdx / (vShapeDetail - 1);
                        double vPointPos = vPointFraction * vShapeWidth - vShapeWidth / 2;
                        double vPointDepth = vShapeDepth * Math.Sin(Math.PI * vPointFraction);

                        double adjustedRadius = radius + vPointPos;
                        double x = centerX + adjustedRadius * Math.Cos(angle);
                        double y = centerY + adjustedRadius * Math.Sin(angle);
                        double z = -vPointDepth; // V-shape depth

                        point_cloud.Add(new ChVectorD((float)x, (float)y, (float)z));
                    }
                }
            }

            //4. Generate a staircase
            double stepHeight = 2.0; // Height of each step
            double platformLength = 7.0; // Length of each platform
            double platformWidth = 7.0; // Width of each platform
            double stairStartX = 10.0; // Starting X position
            double stairStartY = -3.0; // Starting Y position
            double endY = -50.0; // Ending Y position
            double pointSpacing = 0.05; // Spacing between points on the platform

            // Determine the number of steps based on the total Y distance and platform length
            int numSteps = (int)Math.Abs((endY - stairStartY) / platformLength);

            // Create height vectors for each step
            for (int i = 0; i < numSteps; i++)
            {
                double y_start = stairStartY - i * platformLength;
                double z = stepHeight * i;
                // Iterate over the surface of each platform
                for (double x = stairStartX; x < stairStartX + platformWidth; x += pointSpacing)
                {
                    for (double y = y_start; y > y_start - platformLength; y -= pointSpacing)
                    {
                        point_cloud.Add(new ChVectorD(x, y, z));
                    }
                }
            }
            
            // Adjust location and position
            // Note the differences in worldframes
            ChVectorD positionZ = new ChVectorD(0,10,0); // ZUp
            ChVectorD positionY = new ChVectorD(0, 0, -10); // YUp
            ChQuaternionD rotationZ = chrono.Q_from_AngZ(-170); // Rotation in ZUp
            ChQuaternionD rotationY = chrono.Q_from_AngY(-170); // Rotation in Yup

            // Generate the patch of terrain built from the combined point cloud
            var patch = terrain.AddPatch(patch_mat, new ChCoordsysD((isYUp ? positionY : positionZ), (isYUp ? rotationY : rotationZ)),
                                                                                point_cloud, 100, 100, 100, 500, 5, 20, 0.3, 1.0);
            patch.SetColor(new ChColor(0.7f, 0.7f, 0.7f));
            // Initialize the terrain
            terrain.Initialize();

            //------------------------------------------------------------
            // Create the vehicle Irrlicht interface
            //------------------------------------------------------------
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Mesh Terrain Demo");
            if (isYUp) { vis.SetCameraVertical(CameraVerticalDir.Y); } // Adjustment for Y-Up world
            vis.SetChaseCamera(new ChVectorD(0.0, 0.0, 2.0), 5.0, 0.05);
            vis.Initialize();
            if (isYUp)
            { // add a light that's noticeably different for y up (green for y-axis)
                vis.AddLight(new ChVectorD(30, 120, 30), 300, new ChColor(0.5f, 0.5f, 0.5f));
            } else
            {
                vis.AddLightDirectional(80, 10);
            }
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(hmmwv.GetVehicle());


            // Enable visualisation for reference
            vis.EnableCollisionShapeDrawing(false);
            vis.EnableBodyFrameDrawing(false);

            //------------------------------------------------------------
            // Driver Setup
            //------------------------------------------------------------

            // Set the time response for steering and throttle keyboard inputs.
            double render_step_size = 1.0 / 50;  // FPS = 50
            double steering_time = 1.0;          // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 1.0;          // time to go from 0 to +1
            double braking_time = 0.3;           // time to go from 0 to +1

            ChInteractiveDriverIRR driver = new ChInteractiveDriverIRR(vis);
            driver.SetSteeringDelta(render_step_size / steering_time);
            driver.SetThrottleDelta(render_step_size / throttle_time);
            driver.SetBrakingDelta(render_step_size / braking_time);
            driver.Initialize();

            //------------------------------------------------------------
            // Simulation loop
            //------------------------------------------------------------
            hmmwv.GetVehicle().EnableRealtime(true);
            while (vis.Run())
            {
                double time = hmmwv.GetSystem().GetChTime();

                vis.BeginScene();
                vis.Render();
                vis.EndScene();

                // Get driver inputs
                DriverInputs driver_inputs = driver.GetInputs();

                // Update modules (process inputs from other modules)
                driver.Synchronize(time);
                hmmwv.Synchronize(time, driver_inputs, terrain);
                terrain.Synchronize(time);
                vis.Synchronize(time, driver_inputs);

                // Advance simulation for one timestep for all modules
                driver.Advance(step_size);
                hmmwv.Advance(step_size);
                terrain.Advance(step_size);
                vis.Advance(step_size);

            }

        }   // Main
    }       // Program
}           // Namespace


