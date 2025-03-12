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

            // Create the vehicle system
            WheeledVehicle vehicle = new WheeledVehicle(GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json"), ChContactMethod.SMC);
            vehicle.Initialize(new ChCoordsysd(new ChVector3d(0, 0, 0.5), new ChQuaterniond(1, 0, 0, 0)));
            vehicle.GetChassis().SetFixed(false);
            vehicle.SetChassisVisualizationType(VisualizationType.MESH);
            vehicle.SetChassisRearVisualizationType(VisualizationType.PRIMITIVES);
            vehicle.SetSubchassisVisualizationType(VisualizationType.PRIMITIVES);
            vehicle.SetSuspensionVisualizationType(VisualizationType.PRIMITIVES);
            vehicle.SetSteeringVisualizationType(VisualizationType.PRIMITIVES);
            vehicle.SetWheelVisualizationType(VisualizationType.MESH);

            // Create and initialize the powertrain system
            ChEngine engine = ReadEngineJSON(GetDataFile("hmmwv/powertrain/HMMWV_EngineShafts.json"));
            ChTransmission transmission = ReadTransmissionJSON(GetDataFile("hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json"));
            ChPowertrainAssembly powertrain = new ChPowertrainAssembly(engine, transmission);
            vehicle.InitializePowertrain(powertrain);

            // Create and initialize the tires
            foreach (ChAxle axle in vehicle.GetAxles())
            {
                foreach (ChWheel wheel in axle.GetWheels())
                {
                    ChTire tire = ReadTireJSON(GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json"));
                    vehicle.InitializeTire(tire, wheel, VisualizationType.MESH);
                }
            }

            // Containing system
            ChSystem system = vehicle.GetSystem();

            // Associate a collision system
            system.SetCollisionSystemType(ChCollisionSystem.Type.BULLET);

            // Create the terrain
            RigidTerrain terrain = new RigidTerrain(system, GetDataFile("terrain/RigidPlane.json"));
            terrain.Initialize();
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();

            vis.SetWindowTitle("CSharp Vehicle Irrlicht Demo");
            vis.SetChaseCamera(new ChVector3d(0.0, 0.0, 1.75), 3, 1.5);
            vis.Initialize();
            
            vis.AddLightDirectional();
            vis.AddSkyBox();
            vis.AddLogo();
            
            vis.AttachVehicle(vehicle);

            // Create the interactive Irrlicht driver system
            ChInteractiveDriverIRR driver_irr = new ChInteractiveDriverIRR(vis);
            driver_irr.SetSteeringDelta(0.02);
            driver_irr.SetThrottleDelta(0.02);
            driver_irr.SetBrakingDelta(0.06);
            driver_irr.Initialize();        
            
            // Simulation loop
            double step_size = 2e-3;

            vehicle.EnableRealtime(true);
            while (vis.Run())
            {
                // Render scene
                vis.BeginScene();
                vis.Render();
                vis.EndScene();

                // Get driver inputs
                DriverInputs driver_inputs = driver_irr.GetInputs();

                // Update modules (process inputs from other modules)
                double time = vehicle.GetSystem().GetChTime();
                driver_irr.Synchronize(time);
                vehicle.Synchronize(time, driver_inputs, terrain);
                terrain.Synchronize(time);
                vis.Synchronize(time, driver_inputs);

                // Advance simulation for one timestep for all modules
                driver_irr.Advance(step_size);
                vehicle.Advance(step_size);
                terrain.Advance(step_size);
                vis.Advance(step_size);
            }
        }

    }
}
