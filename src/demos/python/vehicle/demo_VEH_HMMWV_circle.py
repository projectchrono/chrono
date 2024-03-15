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
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-75, 0, 0.5),chrono.QUNIT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_RWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
minfo = chrono.ChContactMaterialData()
minfo.mu = 0.8
minfo.cr = 0.01
minfo.Y = 2e7
patch_mat = minfo.CreateMaterial(hmmwv.GetSystem().GetContactMethod())
terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
patch.SetColor(chrono.ChColor(1, 1, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Left circle path
path = veh.CirclePath(chrono.ChVector3d(-75, 0, 0.6), 20, 40, True, 10)
npoints = path.GetNumPoints()

path_asset = chrono.ChVisualShapeLine()
path_asset.SetLineGeometry(chrono.ChLineBezier(path))
path_asset.SetName("test path")
path_asset.SetNumRenderPoints(max(2 * npoints, 400))
patch.GetGroundBody().AddVisualShape(path_asset)

# Create the PID lateral controller
steeringPID = veh.ChPathSteeringController(path)
steeringPID.SetLookAheadDistance(5)
steeringPID.SetGains(0.8, 0, 0)

# Create the vehicle Irrlicht application
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Constant radius test')
vis.SetWindowSize(1280, 1024)
vis.SetHUDLocation(500, 20)
vis.Initialize()
vis.AddLogo()
vis.AddLightDirectional()
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())

# Visualization of controller points (sentinel & target)
ballS = vis.GetSceneManager().addSphereSceneNode(0.1)
ballT = vis.GetSceneManager().addSphereSceneNode(0.1)
ballS.getMaterial(0).EmissiveColor = chronoirr.SColor(0, 255, 0, 0)
ballT.getMaterial(0).EmissiveColor = chronoirr.SColor(0, 0, 255, 0)

# ---------------
# Simulation loop
# ---------------

steeringPID_output = 0

hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = hmmwv.GetSystem().GetChTime()
    
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
    vis.Render()
    vis.EndScene()
    
    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation for one timestep for all modules
    steeringPID_output = steeringPID.Advance(hmmwv.GetRefFrame(), time, step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)
