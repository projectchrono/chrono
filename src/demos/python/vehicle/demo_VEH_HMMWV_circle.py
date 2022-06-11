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
# Authors: Simone Benatti
# =============================================================================
#
# HMMWV constant radius turn.
# This program uses explicitly a ChPathSteeringControler.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
# =============================================================================

step_size = 2e-3

throttle_value = 0.3

# =============================================================================

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Create the HMMWV vehicle
my_hmmwv = veh.HMMWV_Full()
my_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
my_hmmwv.SetChassisFixed(False)
my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-75, 0, 0.5),chrono.QUNIT))
my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
my_hmmwv.SetDriveType(veh.DrivelineTypeWV_RWD)
my_hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
my_hmmwv.SetTireStepSize(step_size)
my_hmmwv.Initialize()

my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
my_hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Create the terrain
minfo = veh.MaterialInfo()
minfo.mu = 0.8
minfo.cr = 0.01
minfo.Y = 2e7
patch_mat = minfo.CreateMaterial(my_hmmwv.GetSystem().GetContactMethod())
terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 200)
patch.SetColor(chrono.ChColor(1, 1, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Left circle path
path = veh.CirclePath(chrono.ChVectorD(-75, 0, 0.6), 20, 40, True, 10)
npoints = path.getNumPoints()

path_asset = chrono.ChLineShape()
path_asset.SetLineGeometry(chrono.ChLineBezier(path))
path_asset.SetName("test path")
path_asset.SetNumRenderPoints(max(2 * npoints, 400))
patch.GetGroundBody().AddVisualShape(path_asset)

# Create the PID lateral controller
steeringPID = veh.ChPathSteeringController(path, False)
steeringPID.SetLookAheadDistance(5)
steeringPID.SetGains(0.8, 0, 0)
steeringPID.Reset(my_hmmwv.GetVehicle())

# Create the vehicle Irrlicht application
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
my_hmmwv.GetVehicle().SetVisualSystem(vis)
vis.SetWindowTitle('Constant radius test')
vis.SetWindowSize(1280, 1024)
vis.SetHUDLocation(500, 20)
vis.Initialize()
vis.AddLogo()
vis.AddTypicalLights()
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.AddSkyBox()

# Visualization of controller points (sentinel & target)
ballS = vis.GetSceneManager().addSphereSceneNode(0.1)
ballT = vis.GetSceneManager().addSphereSceneNode(0.1)
ballS.getMaterial(0).EmissiveColor = chronoirr.SColor(0, 255, 0, 0)
ballT.getMaterial(0).EmissiveColor = chronoirr.SColor(0, 0, 255, 0)

# ---------------
# Simulation loop
# ---------------

steeringPID_output = 0

while vis.Run() :
    time = my_hmmwv.GetSystem().GetChTime()
    
    # Driver inputs
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = np.clip(steeringPID_output, -1.0, +1.0)
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_braking = 0.0
    
    # Update sentinel and target location markers for the path-follower controller.
    pS = steeringPID.GetSentinelLocation()
    pT = steeringPID.GetTargetLocation()
    ballS.setPosition(chronoirr.vector3df(pS.x, pS.y, pS.z))
    ballT.setPosition(chronoirr.vector3df(pT.x, pT.y, pT.z))
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    my_hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize("", driver_inputs)
    
    # Advance simulation for one timestep for all modules
    steeringPID_output = steeringPID.Advance(my_hmmwv.GetVehicle(), step_size)
    terrain.Advance(step_size)
    my_hmmwv.Advance(step_size)
    vis.Advance(step_size)



