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
// Main driver function for a vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
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

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetDataPath(CHRONO_VEHICLE_DATA_DIR);

            // Rigid terrain dimensions
            double terrainLength = 100.0;  // size in X direction
            double terrainWidth = 100.0;   // size in Y direction

            // Simulation step size
            double step_size = 1e-3;
            double tire_step_size = 1e-3;

            // Time interval between two render frames
            double render_step_size = 1.0 / 50;  // FPS = 50

            // Point on chassis tracked by the camera
            ChVector3d trackPoint = new ChVector3d(0.0, 0.0, 1.75);
            // Create and configure the Kraz truck
            Kraz truck = new Kraz();

            truck.SetChassisFixed(false);
            // To mimick c++, adjust this to variables, not hard coded coords
            truck.SetInitPosition(new ChCoordsysd(new ChVector3d(0, 0, 0.5), new ChQuaterniond(1, 0, 0, 0)));
            truck.SetTireStepSize(tire_step_size);
            truck.SetInitFwdVel(0.0);

            truck.Initialize();

            truck.SetChassisVisualizationType(VisualizationType.MESH, VisualizationType.PRIMITIVES);
            truck.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
            truck.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES, VisualizationType.PRIMITIVES);
            truck.SetWheelVisualizationType(VisualizationType.MESH, VisualizationType.MESH);
            truck.SetTireVisualizationType(VisualizationType.MESH, VisualizationType.MESH);

            // Containing system
            ChSystem system = truck.GetSystem();
            // Associate a collision system
            system.SetCollisionSystemType(ChCollisionSystem.Type.BULLET);

            // Create the terrain
            // (not yet patch terrain)
            //RigidTerrain terrain = new RigidTerrain(system, GetDataFile("terrain/RigidPlane.json"));
            //terrain.Initialize();

            // Create the terrain
            RigidTerrain terrain = new RigidTerrain(truck.GetSystem());
            var patch_mat = new ChContactMaterialNSC();
            patch_mat.SetFriction(0.9f);
            patch_mat.SetRestitution(0.01f);
            var patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth);
            patch.SetColor(new ChColor(0.5f, 0.5f, 1));
            patch.SetTexture(GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            terrain.Initialize();

            // Create the interactive Irrlicht driver system
            ChInteractiveDriver driver = new ChInteractiveDriver(truck.GetTractor());
            double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 1.0;  // time to go from 0 to +1
            double braking_time = 0.3;   // time to go from 0 to +1
            driver.SetSteeringDelta(render_step_size / steering_time);
            driver.SetThrottleDelta(render_step_size / throttle_time);
            driver.SetBrakingDelta(render_step_size / braking_time);
            driver.Initialize();

            // Create the vehicle Irrlicht interface
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Semi-trailer truck :: Open Loop");
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 1.75), 6, 0.5);
            vis.Initialize();
            vis.AddLightDirectional();
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(truck.GetTractor());
            vis.AttachDriver(driver);

            // Number of simulation steps between two 3D view render frames
            int render_steps = (int)Math.Ceiling(render_step_size / step_size);

            // Initialize simulation frame counter
            int step_number = 0;


            truck.GetTractor().EnableRealtime(true);
            while (vis.Run())
            {
                // Render scene
                if (step_number % render_steps == 0)
                {
                    vis.BeginScene();
                    vis.Render();
                    vis.EndScene();
                }

                // Get driver inputs
                DriverInputs driver_inputs = driver.GetInputs();

                // Update modules (process inputs from other modules)
                double time = truck.GetSystem().GetChTime();
                driver.Synchronize(time);
                truck.Synchronize(time, driver_inputs, terrain);
                terrain.Synchronize(time);
                vis.Synchronize(time, driver_inputs);

                // Advance simulation for one timestep for all modules
                driver.Advance(step_size);
                truck.Advance(step_size);
                terrain.Advance(step_size);
                vis.Advance(step_size);

                // Increment frame number
                step_number++;
            }
        }

    }
}
