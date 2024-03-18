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
// Gator acceleration test.
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

            // TODO: correct CHRONO_VERSION call
            //Console.WriteLine(chrono.GetLog() + "Copyright (c) 2017 projectchrono.org\nChrono version: " + CHRONO_VERSION + "\n\n");

            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);
            chrono_vehicle.SetDataPath(CHRONO_VEHICLE_DATA_DIR);
            
            // Simulation step size
            double step_size = 1e-3;

            // Initial vehicle location and orientation (m)
            ChVector3d initLoc = new ChVector3d(-40, 0, 0.5);
            ChQuaterniond initRot = new ChQuaterniond(1, 0, 0, 0);

            // Brake type (SIMPLE or SHAFTS)
            BrakeType brake_type = BrakeType.SHAFTS;

            // Terrain slope (radians)
            double slope = 20 * chrono.CH_DEG_TO_RAD;

            // Set speed (m/s)
            double target_speed = 4;

            //------------------------------------------
            // Visualisation and Vehicle Setup
            //------------------------------------------

            // Create and configure the vehicle
            Gator gator = new Gator();
            // Vehicle Collisions
            gator.SetContactMethod(ChContactMethod.NSC);
            gator.SetChassisCollisionType(CollisionType.NONE);
            // Configure vehicle specifics
            gator.SetInitPosition(new ChCoordsysd(initLoc, initRot));
            gator.SetChassisFixed(false);
            gator.SetBrakeType(brake_type);
            gator.SetTireType(TireModelType.TMEASY);
            gator.SetTireStepSize(step_size);
            gator.SetAerodynamicDrag(0.5, 5.0, 1.2);
            gator.EnableBrakeLocking(true);
            gator.Initialize();
            // Visualisation of vehicle
            gator.SetChassisVisualizationType(VisualizationType.MESH);
            gator.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
            gator.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES);
            gator.SetWheelVisualizationType(VisualizationType.MESH);
            gator.SetTireVisualizationType(VisualizationType.MESH);
            // Associate a collision system
            gator.GetSystem().SetCollisionSystemType(ChCollisionSystem.Type.BULLET);

            //------------------------------------------
            // Terrian Setup
            //------------------------------------------
            // Create the terrain
            RigidTerrain terrain = new RigidTerrain(gator.GetSystem());

            ChContactMaterialData minfo = new ChContactMaterialData
            {
                mu = 0.9f,
                cr = 0.01f,
                Y = 2e7f
            };
            var patch_mat = minfo.CreateMaterial(ChContactMethod.NSC);

            var patch1 = terrain.AddPatch(patch_mat, new ChCoordsysd(new ChVector3d(-25, 0, 0), chrono.QUNIT), 50.0, 20.0);
            patch1.SetTexture(GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
            patch1.SetColor(new ChColor(0.8f, 0.8f, 0.5f));

            double s = Math.Sin(slope);
            double c = Math.Cos(slope);
            var patch2 = terrain.AddPatch(patch_mat, new ChCoordsysd(new ChVector3d(100 * c, 0, 100 * s), chrono.QuatFromAngleY(-slope)), 200.0, 20.0);
            patch2.SetTexture(GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
            patch2.SetColor(new ChColor(0.8f, 0.5f, 0.8f));

            var patch3 = terrain.AddPatch(patch_mat, new ChCoordsysd(new ChVector3d(200 * c + 25, 0, 200 * s), chrono.QUNIT), 50.0, 20.0);
            patch3.SetTexture(GetDataFile("terrain/textures/tile4.jpg"), 200, 40);
            patch3.SetColor(new ChColor(0.8f, 0.8f, 0.5f));

            terrain.Initialize();

            //------------------------------------------
            // Driver and Path Following Setup
            //------------------------------------------
            // Create the straight path and the driver system
            var path = StraightLinePath(new ChVector3d(-50, 0, 0.5), new ChVector3d(300, 0, 0.5), 1);
            ChPathFollowerDriver driver = new ChPathFollowerDriver(gator.GetVehicle(), path, "my_path", target_speed);
            driver.GetSteeringController().SetLookAheadDistance(5.0);
            driver.GetSteeringController().SetGains(0.5, 0, 0);
            driver.GetSpeedController().SetGains(0.6, 0.4, 0.4);
            driver.Initialize();

            // Create the vehicle Irrlicht interface
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Gator Acceleration");
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
            vis.Initialize();
            vis.AddLightDirectional();
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(gator.GetVehicle());

            // ---------------
            // Simulation loop
            // ---------------
            gator.GetVehicle().LogSubsystemTypes();
            Console.WriteLine("Vehicle mass: " + gator.GetVehicle().GetMass());
            // Initialize simulation frame counters
            int step_number = 0;
            gator.GetVehicle().EnableRealtime(true);
            // Loop
            while (vis.Run())
            {
                double time = gator.GetSystem().GetChTime();
                
                vis.BeginScene();
                vis.Render();
                vis.EndScene();

                // Get driver inputs
                DriverInputs driver_inputs = driver.GetInputs();
                if (time > 13)
                {
                    driver_inputs.m_braking = 1;
                    driver_inputs.m_throttle = 0;
                }

                // Update modules (process inputs from other modules)
                driver.Synchronize(time);
                gator.Synchronize(time, driver_inputs, terrain);
                terrain.Synchronize(time);
                vis.Synchronize(time, driver_inputs);

                // Advance simulation for one timestep for all modules
                driver.Advance(step_size);
                gator.Advance(step_size);
                terrain.Advance(step_size);
                vis.Advance(step_size);

                // Increment frame number
                step_number++;
            }
        }
    }
}
