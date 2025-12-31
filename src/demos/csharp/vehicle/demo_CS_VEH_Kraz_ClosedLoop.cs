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
// Tractor-trailer acceleration test.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

using System;
using static ChronoGlobals;
using static chrono;
using static chrono_vehicle;
using static chrono_postprocess;

namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Copyright (c) 2017 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);

            // TODO: correct CHRONO_VERSION call
            // Console.WriteLine(chrono.GetLog() + "Copyright (c) 2017 projectchrono.org\nChrono version: " + CHRONO_VERSION + "\n\n");

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetVehicleDataPath(CHRONO_VEHICLE_DATA_DIR);
            
            // Rigid terrain dimensions
            double terrainLength = 300.0;  // size in X direction

            // Simulation step size
            double step_size = 1e-3;
            double tire_step_size = 1e-3;

            // Time interval between two render frames
            double render_step_size = 1.0 / 50;  // FPS = 50

            //------------------------------------------
            // Visualisation, Tracking and Vehicle Setup
            //------------------------------------------

            // Point on chassis tracked by the camera
            ChVector3d trackPoint = new ChVector3d(0.0, 0.0, 1.75);

            // Initial vehicle location and orientation
            ChVector3d initLoc = new ChVector3d(0, 0, 0.6);
            ChQuaterniond initRot = new ChQuaterniond(1, 0, 0, 0);

            // Create and configure the Kraz truck
            Kraz truck = new Kraz();
            truck.SetChassisFixed(false);
            truck.SetInitPosition(new ChCoordsysd(initLoc, initRot));
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
            RigidTerrain terrain = new RigidTerrain(truck.GetSystem());
            var patch_mat = new ChContactMaterialSMC();
            patch_mat.SetFriction(0.9f);
            patch_mat.SetRestitution(0.01f);
            patch_mat.SetYoungModulus(2e7f);
            patch_mat.SetPoissonRatio(0.3f);
            for (int i = 0; i < 3; i++)
            {
                var patch = terrain.AddPatch(patch_mat, new ChCoordsysd(new ChVector3d(terrainLength * i, 0, 0), chrono.QUNIT), terrainLength, 5);
                patch.SetColor(new ChColor(0.8f, 0.8f, 0.5f));
                patch.SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 5);
            }
            terrain.Initialize();

            // Create the vehicle Irrlicht interface
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Semi-trailer truck :: Follows Straight Line");
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 1.75), 6, 0.5);
            vis.Initialize();
            vis.AddLightDirectional();
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(truck.GetTractor());
            
            // Create the straight path and the driver system
            var path = StraightLinePath(new ChVector3d(-terrainLength / 2, 0, 0.5), new ChVector3d(10 * terrainLength / 2, 0, 0.5), 1);
            ChPathFollowerDriver driver = new ChPathFollowerDriver(truck.GetTractor(), path, "my_path", 1000.0);
            driver.GetSteeringController().SetLookAheadDistance(5.0);
            driver.GetSteeringController().SetGains(0.5, 0, 0);
            driver.GetSpeedController().SetGains(0.4, 0, 0);
            driver.Initialize();

#if (COMPONENTS == Postprocess)
            Console.WriteLine("Csharp Postprocessing Enabled. Will attempt Gnuplot of results");
#endif

            // ---------------
            // Simulation loop
            // ---------------

            // Running average of vehicle speed
            ChRunningAverage speed_filter = new ChRunningAverage(500);
            double last_speed = -1;

            // Record vehicle speed
            ChFunctionInterp speed_recorder = new ChFunctionInterp();

            // Initialize simulation frame counter and simulation time
            int step_number = 0;
            double time = 0;
            bool done = false;

            // Initialise the timer
            ChTimer timer = new ChTimer();
            timer.start();
            while (vis.Run())
            {
                time = truck.GetSystem().GetChTime();

                // C++ Method
                double speed = speed_filter.Add(truck.GetTractor().GetSpeed());
                if (!done)
                {
                    speed_recorder.AddPoint(time, speed);
                    // Check if changed in speed has plateaued
                    if (time > 6 && Math.Abs((speed - last_speed) / step_size) < 2e-4)
                    {
                        done = true;
                        timer.stop();
                        Console.WriteLine("Simulation time: " + timer.GetTimeSeconds());
                        Console.WriteLine("Maximum speed: " + speed);
#if (COMPONENTS == Postprocess)
                        ChGnuPlot gplot = new ChGnuPlot();
                        gplot.SetGrid();
                        gplot.SetLabelX("time (s)");
                        gplot.SetLabelY("speed (m/s)");
                        gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif
                    }

                }
                last_speed = speed;

                // End Simulation condition
                if (time >= 100)
                    break;

                vis.BeginScene();
                vis.Render();
        
                // Get driver inputs
                DriverInputs driver_inputs = driver.GetInputs();

                if (done)
                {
                    driver_inputs.m_throttle = 0.1;
                    driver_inputs.m_braking = 0.8;
                }

                // Update modules (process inputs from other modules)
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

                vis.EndScene();
            }
        }

    }
}
