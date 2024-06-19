# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demonstration of a steering path-follower and cruise control PID controlers. 
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chassis_vis_type)
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    hmmwv.SetSteeringVisualizationType(steering_vis_type)
    hmmwv.SetWheelVisualizationType(wheel_vis_type)
    hmmwv.SetTireVisualizationType(tire_vis_type)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain

    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             300, 50)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the path-follower, cruise-control driver
    # Use a parameterized ISO double lane change (to left)
    path = veh.DoubleLaneChangePath(initLoc, 13.5, 4.0, 11.0, 50.0, True)
    driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle(), path, "my_path", target_speed)
    driver.GetSteeringController().SetLookAheadDistance(5)
    driver.GetSteeringController().SetGains(0.8, 0, 0)
    driver.GetSpeedController().SetGains(0.4, 0, 0)
    driver.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV steering controller')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

   	# Visualization of controller points (sentinel & target)
    ballS = vis.GetSceneManager().addSphereSceneNode(0.1);
    ballT = vis.GetSceneManager().addSphereSceneNode(0.1);
    ballS.getMaterial(0).EmissiveColor = irr.SColor(0, 255, 0, 0);
    ballT.getMaterial(0).EmissiveColor = irr.SColor(0, 0, 255, 0);

    # Simulation loop
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        # End simulation
        if (time >= t_end):
            vis.Quit()

        # Update sentinel and target location markers for the path-follower controller.
        pS = driver.GetSteeringController().GetSentinelLocation()
        pT = driver.GetSteeringController().GetTargetLocation()
        ballS.setPosition(irr.vector3df(pS.x, pS.y, pS.z))
        ballT.setPosition(irr.vector3df(pT.x, pT.y, pT.z))

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location
initLoc = chrono.ChVector3d(-50, 0, 0.7)

# Vehicle target speed (cruise-control)
target_speed = 12

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_NONE
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Type of powertrain models (SHAFTS, SIMPLE)
engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineTypeWV_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 2e-3;
tire_step_size = 1e-3;

# Simulation end time
t_end = 100;


main()
