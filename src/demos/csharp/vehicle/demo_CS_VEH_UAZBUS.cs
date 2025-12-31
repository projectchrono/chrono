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
// Authors: Rainer Gericke, Josh Diyn
// =============================================================================
//
// Demo program for UAZBUS simulation.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
// All units SI.
//
// =============================================================================

using System;
using System.IO; // For directory creation
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

            // Local variables for C# 7.3 compatability
            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetVehicleDataPath(CHRONO_VEHICLE_DATA_DIR);

            // Rigid terrain dimensions
            double terrainLength = 300.0;  // size in X direction
            double terrainWidth = 300.0;   // size in Y direction

            // Simulation step size
            double step_size = 3e-3;
            double tire_step_size = 1e-3;

            // Simulation end time
            double tend = 15;

            // Time interval between two render frames
            double render_step_size = 1.0 / 50;  // FPS = 50
            

            bool povray_output = false;

            //---------------------------
            // POVRAY PROMPT
            //---------------------------
            Console.WriteLine("Do you want POVRAY output? (y/n)");
            while (true)
            {
                ConsoleKeyInfo key = Console.ReadKey(true); // true to not echo the character
                if (key.Key == ConsoleKey.Y)
                {
                    povray_output = true;
                    Console.WriteLine("You selected 'yes'.");
                    break;
                }
                else if (key.Key == ConsoleKey.N)
                {
                    povray_output = false;
                    Console.WriteLine("You selected 'no'.");
                    break;
                }
                else
                {
                    Console.WriteLine("Invalid input. Please press 'y' for yes or 'n' for no.");
                }
            }

            // Output directories
            string out_dir = chrono.GetChronoOutputPath() + "UAZBUS";
            string pov_dir = out_dir + "/POVRAY";
            
            //------------------------------------------
            // Visualisation, Tracking and Vehicle Setup
            //------------------------------------------

            // Point on chassis tracked by the camera
            ChVector3d trackPoint = new ChVector3d(0.0, 0.0, 1.75);

            // Initial vehicle location and orientation
            ChVector3d initLoc = new ChVector3d(0, 0, 0.4);
            ChQuaterniond initRot = new ChQuaterniond(1, 0, 0, 0);

            // Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
            VisualizationType chassis_vis_type = VisualizationType.MESH;
            VisualizationType suspension_vis_type = VisualizationType.PRIMITIVES;
            VisualizationType steering_vis_type = VisualizationType.PRIMITIVES;
            VisualizationType wheel_vis_type = VisualizationType.MESH;
            VisualizationType tire_vis_type = VisualizationType.MESH;

            // Type of tire model (RIGID, TMEASY, PAC02)
            TireModelType tire_model = TireModelType.PAC02;
            
            // Create and configure the UAZBUS
            UAZBUS uaz = new UAZBUS();
            uaz.SetContactMethod(ChContactMethod.NSC);
            uaz.SetChassisFixed(false);
            uaz.SetInitPosition(new ChCoordsysd(initLoc, initRot));
            uaz.SetTireType(tire_model);
            uaz.SetTireStepSize(tire_step_size);
            uaz.SetInitFwdVel(0.0);
            uaz.Initialize();

            // Visualisation setup of the UAZBUS
            uaz.SetChassisVisualizationType(chassis_vis_type);
            uaz.SetSuspensionVisualizationType(suspension_vis_type);
            uaz.SetSteeringVisualizationType(steering_vis_type);
            uaz.SetWheelVisualizationType(wheel_vis_type);
            uaz.SetTireVisualizationType(tire_vis_type);

            // Output to console
            {
                var suspF = CastToChToeBarLeafspringAxle(uaz.GetVehicle().GetSuspension(0));

                var springFL = suspF.GetSpring(VehicleSide.LEFT);
                var shockFL = suspF.GetShock(VehicleSide.RIGHT);

                Console.WriteLine("Spring rest length front: " + springFL.GetRestLength());
                Console.WriteLine("Shock rest length front:  " + shockFL.GetRestLength());
            }
            {
                var suspR = CastToChLeafspringAxle(uaz.GetVehicle().GetSuspension(1));
                var springRL = suspR.GetSpring(VehicleSide.LEFT);
                var shockRL = suspR.GetShock(VehicleSide.RIGHT);

                Console.WriteLine("Spring rest length rear: " + springRL.GetRestLength());
                Console.WriteLine("Shock rest length rear:  " + shockRL.GetRestLength());
            }
            Console.WriteLine("Vehicle mass: " + uaz.GetVehicle().GetMass());

            // Containing system, use system instance
            ChSystem system = uaz.GetSystem();
            // Associate a collision system
            system.SetCollisionSystemType(ChCollisionSystem.Type.BULLET);

            // ------------------
            // Create the terrain
            // ------------------
            RigidTerrain terrain = new RigidTerrain(system);
            var patch_mat = new ChContactMaterialNSC();
            patch_mat.SetFriction(0.9f);
            patch_mat.SetRestitution(0.01f);
            var patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth);
            patch.SetColor(new ChColor(0.8f, 0.8f, 1.0f));
            patch.SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 1200, 1200);
            terrain.Initialize();

            // Create the interactive Irrlicht driver system
            ChInteractiveDriver driver = new ChInteractiveDriver(uaz.GetVehicle());
            double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 1.0;  // time to go from 0 to +1
            double braking_time = 0.3;   // time to go from 0 to +1
            driver.SetSteeringDelta(render_step_size / steering_time);
            driver.SetThrottleDelta(render_step_size / throttle_time);
            driver.SetBrakingDelta(render_step_size / braking_time);
            driver.Initialize();

            // -------------------------------------
            // Create the vehicle Irrlicht interface
            // -------------------------------------
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("UAZBUS Demo :: CSharp");
            vis.SetChaseCamera(trackPoint, 6, 0.5);
            vis.Initialize();
            vis.AddLightDirectional();
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(uaz.GetVehicle());
            vis.AttachDriver(driver);

            // -----------------
            // Initialize output
            // -----------------
            // These are created whether povray was set to true or false
            try
            {
                // Check if the directory exists, if not, create it
                if (!Directory.Exists(out_dir))
                {
                    Directory.CreateDirectory(out_dir);
                }

                // Check for POV-Ray output directory, create it if it doesn't exist
                if (povray_output)
                {
                    if (!Directory.Exists(pov_dir))
                    {
                        Directory.CreateDirectory(pov_dir);
                    }
                }
            }
            catch (Exception ex) // Print error if the directories are unable to be created
            {
                Console.WriteLine("Error creating directory: " + ex.Message);
                Environment.Exit(1); // Exit code
            }

            // ---------------
            // Simulation loop
            // ---------------

            // Number of simulation steps between two 3D view render frames
            int render_steps = (int)Math.Ceiling(render_step_size / step_size);
            int render_frame = 0;
            // Initialize simulation frame counter
            int step_number = 0;

            // Prep the vehicle for simulation
            double maxKingpinAngle = 0.0;
            uaz.GetVehicle().LogSubsystemTypes();
            uaz.GetVehicle().EnableRealtime(true);
            ChRunningAverage RTF_Filter = new ChRunningAverage(50);

            // Simulation
            while (vis.Run())
            {
                double time = uaz.GetSystem().GetChTime();

                // Render scene
                if (step_number % render_steps == 0)
                {
                    vis.BeginScene();
                    vis.Render();
                    vis.EndScene();

                    // Povray output
                    if (povray_output && step_number % render_steps == 0)
                    {
                        string filename = $"{pov_dir}/data_{render_frame + 1:D3}.dat"; // Format the filename using string interpolation or string.Format
                        chrono.WriteVisualizationAssets(uaz.GetSystem(), filename);
                    }
                    render_frame++;
                }

                // Get driver inputs
                DriverInputs driver_inputs = driver.GetInputs();

                // Update modules (process inputs from other modules)

                driver.Synchronize(time);
                uaz.Synchronize(time, driver_inputs, terrain);
                terrain.Synchronize(time);
                vis.Synchronize(time, driver_inputs);

                // Test for validity of kingpin angles (max. allowed by UAZ: 27 deg)
                var suspF = CastToChToeBarLeafspringAxle(uaz.GetVehicle().GetSuspension(0));
                double leftAngle = suspF.GetKingpinAngleLeft() * 180.0 / chrono.CH_PI;
                double rightAngle = suspF.GetKingpinAngleRight() * 180.0 / chrono.CH_PI;
                if (Math.Abs(leftAngle) > maxKingpinAngle)
                {
                    maxKingpinAngle = Math.Abs(leftAngle);
                }
                if (Math.Abs(rightAngle) > maxKingpinAngle)
                {
                    maxKingpinAngle = Math.Abs(rightAngle);
                }
                
                // Advance simulation for one timestep for all modules
                driver.Advance(step_size);
                uaz.Advance(step_size);
                terrain.Advance(step_size);
                vis.Advance(step_size);

                // Increment frame number
                step_number++;
            }

            Console.WriteLine("Maximum Kingpin Angle = " + maxKingpinAngle + " deg");

        }

    }
}
