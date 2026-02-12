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
// Authors: Radu Serban, Josh Diyn
// =============================================================================
//
// Illustration of using collision geometry for the vehicle chassis
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// This demo includes the option for testing a Y-up world and
// terrain orientation.
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
            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);

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

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetVehicleDataPath(CHRONO_VEHICLE_DATA_DIR);

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
            ChVector3d initLoc = new ChVector3d(0, (isYUp) ? 0.5 : 0, (isYUp) ? 0 : 0.5);

            // Create and configure the vehicle
            HMMWV_Full hmmwv = new HMMWV_Full();
            // Vehicle Collisions
            hmmwv.SetContactMethod(ChContactMethod.NSC);
            hmmwv.SetChassisCollisionType(CollisionType.HULLS); // automatically enables collision for the chassis
            // Configure vehicle specifics
            hmmwv.SetInitPosition(new ChCoordsysd(initLoc, chrono.QUNIT));
            hmmwv.SetEngineType(EngineModelType.SHAFTS);
            hmmwv.SetTransmissionType(TransmissionModelType.AUTOMATIC_SHAFTS);
            hmmwv.SetDriveType(DrivelineTypeWV.AWD);
            hmmwv.UseTierodBodies(false);
            hmmwv.SetSteeringType(SteeringTypeWV.PITMAN_ARM);
            hmmwv.SetBrakeType(BrakeType.SHAFTS);
            hmmwv.SetTireType(TireModelType.TMEASY);
            hmmwv.SetTireStepSize(step_size);
            hmmwv.Initialize();
            if (isYUp) { hmmwv.GetSystem().SetGravitationalAcceleration(new ChVector3d(0, -9.81, 0)); } // adjust the gravity

            // Visualisation of vehicle
            hmmwv.SetChassisVisualizationType(VisualizationType.MESH);
            hmmwv.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
            hmmwv.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES);
            hmmwv.SetWheelVisualizationType(VisualizationType.MESH);
            hmmwv.SetTireVisualizationType(VisualizationType.MESH);

            // Optionally, enable collision for the vehicle wheels.
            // In this case, you must also disable collision between the chassis and wheels (unless the chassis collision model
            // is accurate enough to account for the wheel wells).
            ////hmmwv.GetVehicle().SetWheelCollide(true);
            ////hmmwv.GetVehicle().SetChassisVehicleCollide(false);

            // Create the terrain
            RigidTerrain terrain = new RigidTerrain(hmmwv.GetSystem());

            ChContactMaterialData minfo = new ChContactMaterialData();
            minfo.mu = 0.9f;
            minfo.cr = 0.1f;
            minfo.Y = 2e7f;
            var terrain_mat = minfo.CreateMaterial(hmmwv.GetSystem().GetContactMethod());


            // NB: in Y-Up the +vs Zaxis is to the right of the vehicle, and a +ve slope angle causes an upwards gradient ramp
            ChQuaterniond slope = (isYUp) ? chrono.QuatFromAngleZ(15 * chrono.CH_DEG_TO_RAD) : chrono.QuatFromAngleY(-15 * chrono.CH_DEG_TO_RAD);

            // Rotate for Y-Up world
            ChQuaterniond rotateYUp = new ChQuaterniond(chrono.Q_ROTATE_Z_TO_Y);  // alternatively, could use the call chrono.QuatFromAngleX(-chrono.CH_PI_2));
            // Adjust the slope rotation for the Y-Up world
            ChQuaterniond resultantSlopeRot = new ChQuaterniond();
            resultantSlopeRot.Cross(slope, rotateYUp);

            // Create the ramp path. Place and rotate as per the world rotation
            var ramp = terrain.AddPatch(terrain_mat, new ChCoordsysd(new ChVector3d(20, (isYUp ? 0 : 3), (isYUp ? -3 : 0)), (isYUp ? resultantSlopeRot : slope)), 20, 6, 0.5);
            ramp.SetTexture(GetVehicleDataFile("terrain/textures/concrete.jpg"), 2, 2);

            // Ground patch. Rotate as per the world rotation
            var patch = terrain.AddPatch(terrain_mat, new ChCoordsysd(new ChVector3d(), (isYUp ? rotateYUp : chrono.QUNIT)), 100.0, 100.0, 0.5);
            patch.SetTexture(GetVehicleDataFile("terrain/textures/dirt.jpg"), 20, 20);
            terrain.Initialize();
            
            
            //-------------------------------------------------
            // Call line drawing of mesh height (method adjusts depending on isYUp)
            //-------------------------------------------------

            VisualiseTerrain(); // See below after simulation loop


            // Create the vehicle Irrlicht interface
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Rollover Demo");
            if (isYUp) { vis.SetCameraVertical(CameraVerticalDir.Y); } // Adjustment for Y-Up world
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
            vis.Initialize();
            if (isYUp)
            { // add a light in the correct location for Y-Up
                vis.AddLight(new ChVector3d(30, 120, 30), 300, new ChColor(0.4f, 0.4f, 0.4f));
            } else {
                vis.AddLightDirectional(80, 10);
            }
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(hmmwv.GetVehicle());


            // ---------------
            // Simulation loop
            // ---------------
            hmmwv.GetVehicle().EnableRealtime(true);
            while (vis.Run())
            {
                double time = hmmwv.GetSystem().GetChTime();

                vis.BeginScene();
                vis.Render();
                vis.EndScene();

                // Get driver inputs
                DriverInputs driver_inputs = new DriverInputs();
                driver_inputs.m_steering = 0;
                driver_inputs.m_throttle = 0.5;
                driver_inputs.m_braking = 0;

                // Check rollover -- detach chase camera
                // this is a bit clunky between yup and not. could be streamlined.
                if (isYUp)
                {

                    if (chrono.Vdot(hmmwv.GetChassisBody().GetRotMat().GetAxisZ(), ChWorldFrame.Vertical()) < 0)
                    {
                        var camera = vis.GetChaseCamera();
                        var camera_pos = vis.GetCameraPosition();
                        camera_pos.x = 20;
                        camera_pos.y = 2;
                        camera_pos.z = 12;
                        camera.SetCameraPos(camera_pos);
                        vis.SetChaseCameraState(ChChaseCamera.State.Free);
                        vis.SetChaseCameraAngle(chrono.CH_PI_2 / 2); // point camera towards the vehicle and ramp after freeing it
                    }
                }
                else if (!isYUp)
                {
                    if (chrono.Vdot(hmmwv.GetChassisBody().GetRotMat().GetAxisZ(), ChWorldFrame.Vertical()) < 0)
                    {
                        var camera = vis.GetChaseCamera();
                        var camera_pos = vis.GetCameraPosition();
                        camera_pos.x = 20;
                        camera_pos.y = -12;
                        camera_pos.z = 2;
                        camera.SetCameraPos(camera_pos);
                        vis.SetChaseCameraState(ChChaseCamera.State.Free);
                        vis.SetChaseCameraAngle(chrono.CH_PI_2 / 2); // point camera towards the vehicle and ramp after freeing it
                    }
                }

                // Update modules (process inputs from other modules)
                hmmwv.Synchronize(time, driver_inputs, terrain);
                terrain.Synchronize(time);
                vis.Synchronize(time, driver_inputs);

                // Advance simulation for one timestep for all modules
                hmmwv.Advance(step_size);
                terrain.Advance(step_size);
                vis.Advance(step_size);

            }


            /// Wireframe grid height visualisation method
            // Create a basic polyline based grid across the ground plane to display the terrain height
            // Grid is oriented based on the worldframe (z-up or y-up)
            void VisualiseTerrain()
            {
                double gridSize = 40; // The size of the grid
                double interval = 1.0; // The interval between points, make smaller to refine grid (slows simulation down however)
                // Create a rigid body to store the terrian grid polylines
                ChBody gridBody = new ChBody();
                gridBody.SetFixed(true);

                // Calculate the start and end positions based on the grid size
                double halfGridSize = gridSize / 2;
                ChVector3d gridCentre = new ChVector3d(20,0,0); // Centre of the grid

                int pointsPerSide = (int)(gridSize / interval) + 1;  // Number of points per side

                // Draw horizontal lines
                for (int j = 0; j < pointsPerSide; j++)
                {
                    double gridAxis = -halfGridSize + j * interval + (isYUp ? gridCentre.z : gridCentre.y); // change the axis depending on y-up or z-up
                    var polyline = new ChLinePoly(pointsPerSide);

                    for (int i = 0; i < pointsPerSide; i++)
                    {
                        double x = -halfGridSize + i * interval + gridCentre.x;
                        double height = terrain.GetHeight(new ChVector3d(x, (isYUp ? 1000 : gridAxis), (isYUp ? gridAxis : 1000))); // Height query of terrain at set point
                        polyline.SetPoint(i, new ChVector3d(x, (isYUp ? height : gridAxis), (isYUp ? gridAxis : height)));  // Set each point along the polyline
                    }

                    // Add polyline to visualisation
                    var visual = new ChVisualShapeLine();
                    visual.SetLineGeometry(polyline);
                    visual.SetColor(new ChColor(0.7f, (isYUp ? 1.0f : 0.7f), 0.7f));
                    gridBody.AddVisualShape(visual);  // Add visual to the body
                }

                // Draw vertical lines
                for (int i = 0; i < pointsPerSide; i++)
                {
                    double x = -halfGridSize + i * interval + gridCentre.x;
                    var polyline = new ChLinePoly(pointsPerSide);

                    for (int j = 0; j < pointsPerSide; j++)
                    {
                        double crossAxis = -halfGridSize + j * interval + (isYUp ? gridCentre.z : gridCentre.y);
                        double height = terrain.GetHeight(new ChVector3d(x, (isYUp ? 1000 : crossAxis), (isYUp ? crossAxis : 1000))); // Query the height
                        polyline.SetPoint(j, new ChVector3d(x, (isYUp ? height : crossAxis), (isYUp ? crossAxis : height)));  // Set each point along the polyline
                    }

                    // Add polyline to visualisation
                    var visual = new ChVisualShapeLine();
                    visual.SetLineGeometry(polyline);
                    visual.SetColor(new ChColor(0.8f, 0.8f, 0.8f));
                    gridBody.AddVisualShape(visual);  // Add visual to the body
                }

                hmmwv.GetSystem().AddBody(gridBody);
            }


        }



    }
}


