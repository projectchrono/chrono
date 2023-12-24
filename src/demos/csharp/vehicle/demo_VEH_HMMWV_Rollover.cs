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

            // Initial vehicle location and orientation
            ChVectorD initLoc = new ChVectorD(0, 0, 0.5);

            // Create and configure the vehicle
            HMMWV_Full hmmwv = new HMMWV_Full();
            // Vehicle Collisions
            hmmwv.SetContactMethod(ChContactMethod.NSC);
            hmmwv.SetChassisCollisionType(CollisionType.HULLS); // automatically enables collision for the chassis
            //hmmwv.SetCollisionSystemType(ChCollisionSystem.Type.BULLET); // TODO:: Currently has issues with SWIG wrapping. BULLET is presumed. May need to revisit if multicore module is wrapped.
            // Configure vehicle specifics
            hmmwv.SetInitPosition(new ChCoordsysD(initLoc, chrono.QUNIT));
            hmmwv.SetEngineType(EngineModelType.SHAFTS);
            hmmwv.SetTransmissionType(TransmissionModelType.SHAFTS);
            hmmwv.SetDriveType(DrivelineTypeWV.AWD);
            hmmwv.UseTierodBodies(false);
            hmmwv.SetSteeringType(SteeringTypeWV.PITMAN_ARM);
            hmmwv.SetBrakeType(BrakeType.SHAFTS);
            hmmwv.SetTireType(TireModelType.TMEASY);
            hmmwv.SetTireStepSize(step_size);
            hmmwv.Initialize();

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
            // Ground patch
            var patch = terrain.AddPatch(terrain_mat, chrono.CSYSNORM, 100.0, 100.0);
            patch.SetTexture(GetDataFile("terrain/textures/dirt.jpg"), 20, 20);
            // Ramp patch
            var slope = chrono.Q_from_AngY(-15 * chrono.CH_C_DEG_TO_RAD);
            var ramp = terrain.AddPatch(terrain_mat, new ChCoordsysD(new ChVectorD(20, 3, 0), slope), 20, 6);
            ramp.SetTexture(GetDataFile("terrain/textures/concrete.jpg"), 2, 2);

            terrain.Initialize();

            // Create the vehicle Irrlicht interface
            ChWheeledVehicleVisualSystemIrrlicht vis = new ChWheeledVehicleVisualSystemIrrlicht();
            vis.SetWindowTitle("Rollover Demo");
            vis.SetChaseCamera(new ChVectorD(0.0, 0.0, 2.0), 5.0, 0.05);
            vis.Initialize();
            vis.AddLightDirectional(70, 20);
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
                if (chrono.Vdot(hmmwv.GetChassisBody().GetA().Get_A_Zaxis(), ChWorldFrame.Vertical()) < 0)
                {
                    var camera = vis.GetChaseCamera();
                    var camera_pos = vis.GetCameraPosition();
                    camera_pos.x = 20;
                    camera_pos.y = -12;
                    camera_pos.z = 2;
                    camera.SetCameraPos(camera_pos);
                    vis.SetChaseCameraState(ChChaseCamera.State.Free);
                    vis.SetChaseCameraAngle(chrono.CH_C_PI_2/2); // point camera towards the vehicle and ramp after freeing it
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
        }
    }
}
