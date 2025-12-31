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
// Demonstration of using a RigidTerrain constructed from different patches.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

// TODO: The Json loads a problematic texture for one section. Requires further investigation. Non-Json terrain loads & textures fine.
// Use a JSON to build the terrain
#define USE_JSON

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
            chrono_vehicle.SetVehicleDataPath(CHRONO_VEHICLE_DATA_DIR);
            
            // Simulation step size
            double step_size = 3e-3;
            double tire_step_size = 1e-3;



            //------------------------------------------
            // Visualisation, Tracking and Vehicle Setup
            //------------------------------------------

            // Initial vehicle location and orientation
            ChVector3d initLoc = new ChVector3d(-10, 0, 1);

            // Create and configure the vehicle
            HMMWV_Reduced hmmwv = new HMMWV_Reduced();
            // Vehicle Collisions
            hmmwv.SetContactMethod(ChContactMethod.NSC);
            hmmwv.SetChassisCollisionType(CollisionType.NONE); // automatically enables collision for the chassis
            //hmmwv.SetCollisionSystemType(ChCollisionSystem.Type.BULLET); // TODO:: Currently has issues with SWIG wrapping. BULLET is presumed. May need to revisit if multicore module is wrapped.
            // Configure vehicle specifics
            hmmwv.SetChassisFixed(false);
            hmmwv.SetInitPosition(new ChCoordsysd(initLoc, chrono.QUNIT));
            hmmwv.SetEngineType(EngineModelType.SIMPLE);
            hmmwv.SetTransmissionType(TransmissionModelType.AUTOMATIC_SIMPLE_MAP);
            hmmwv.SetDriveType(DrivelineTypeWV.RWD);
            hmmwv.SetBrakeType(BrakeType.SHAFTS);
            hmmwv.SetTireType(TireModelType.TMEASY);
            hmmwv.SetTireStepSize(tire_step_size);
            hmmwv.Initialize();

            // Visualisation of vehicle
            hmmwv.SetChassisVisualizationType(VisualizationType.NONE);
            hmmwv.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
            hmmwv.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES);
            hmmwv.SetWheelVisualizationType(VisualizationType.MESH);
            hmmwv.SetTireVisualizationType(VisualizationType.MESH);

            //------------------------------------------
            // Terrain Setup
            //------------------------------------------
            RigidTerrain terrain;
            // Create the terrain
#if USE_JSON
            // Create the terrain from JSON specification file
            terrain = new RigidTerrain(hmmwv.GetSystem(), GetVehicleDataFile("terrain/RigidPatches.json"));
#else
            terrain = new RigidTerrain(hmmwv.GetSystem());
                if (true) {
                var patch1_mat = new ChMaterialSurfaceNSC();
                    patch1_mat.SetFriction(0.9f);
                    patch1_mat.SetRestitution(0.01f);
                    var patch1 = terrain.AddPatch(patch1_mat, new ChCoordsysD(new ChVector3d(-16, 0, 0), chrono.QUNIT), 32, 20);
                    patch1.SetColor(new ChColor(0.8f, 0.8f, 0.5f));
                    patch1.SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 20, 20);

                    var patch2_mat = new ChMaterialSurfaceNSC();
                patch2_mat.SetFriction(0.9f);
                    patch2_mat.SetRestitution(0.01f);
                    var patch2 = terrain.AddPatch(patch1_mat, new ChCoordsysD(new ChVector3d(16, 0, 0.08), chrono.QUNIT), 32, 20);
                    patch2.SetColor(new ChColor(1.0f, 0.5f, 0.5f));
                    patch2.SetTexture(GetVehicleDataFile("terrain/textures/concrete.jpg"), 20, 20);

                    var patch3_mat = new ChMaterialSurfaceNSC();
                    patch3_mat.SetFriction(0.9f);
                    patch3_mat.SetRestitution(0.01f);
                    var patch3 = terrain.AddPatch(patch3_mat, new ChCoordsysD(new ChVector3d(0, -42, 0), chrono.QUNIT),
                                                GetVehicleDataFile("terrain/meshes/bump.obj"));
                    patch3.SetColor(new ChColor(0.5f, 0.5f, 0.8f));
                    patch3.SetTexture(GetVehicleDataFile("terrain/textures/dirt.jpg"), 6.0f, 6.0f);

                    var patch4_mat = new ChMaterialSurfaceNSC();
                    patch4_mat.SetFriction(0.9f);
                    patch4_mat.SetRestitution(0.01f);
                    var patch4 = terrain.AddPatch(patch4_mat, new ChCoordsysD(new ChVector3d(0, 42, 0), chrono.QUNIT),
                                                GetVehicleDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0);
                    patch4.SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 6.0f, 6.0f);
                }

                if (false) {
                    var patch_mat = new ChMaterialSurfaceNSC();
                    patch_mat.SetFriction(0.9f);
                    patch_mat.SetRestitution(0.01f);
                    var patch = terrain.AddPatch(patch_mat, new ChCoordsysD(new ChVector3d(0, 0, 10), chrono.QUNIT),
                                                GetVehicleDataFile("terrain/multilayer/multilayer-terrain.obj"));
                }
#endif
            terrain.Initialize();

            //------------------------------------------
            // Driver Setup
            //------------------------------------------

            // Set the time response for steering and throttle keyboard inputs.
            double render_step_size = 1.0 / 50;  // FPS = 50
            double steering_time = 1.0;          // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 1.0;          // time to go from 0 to +1
            double braking_time = 0.3;           // time to go from 0 to +1

            ChInteractiveDriver driver = new ChInteractiveDriver(hmmwv.GetVehicle());
            driver.SetSteeringDelta(render_step_size / steering_time);
            driver.SetThrottleDelta(render_step_size / throttle_time);
            driver.SetBrakingDelta(render_step_size / braking_time);
            driver.Initialize();

            //------------------------------------------
            // Visualisation Setup
            //------------------------------------------

            // Create the vehicle Irrlicht interface
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Rigid Terrain Demo");
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 0.75), 6.0, 0.75);
            vis.Initialize();
            vis.AddLightDirectional();
            vis.AddSkyBox();
            vis.AddLogo();
            vis.AttachVehicle(hmmwv.GetVehicle());
            vis.AttachDriver(driver);

            // TODO: Fix wrapping for calc height
            /*
            ChWriterCSV csv_out = new ChWriterCSV(" ");
            for (int ix = 0; ix < 20; ix++) {
                double x = ix * 1.0;
                for (int iy = 0; iy < 100; iy++) {
                    //double z = terrain.CalcHeight(x, y);
                    //csv_out = (csv_out + x + y + z + "\n");
                }
            }
            csv_out.WriteToFile("terrain.out");
            */

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
        }
    }
}
