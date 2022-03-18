# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Rainer Gericke
# =============================================================================
#
# Demo program for UAZBUS simulation.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
# All units SI.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.4)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Type of tire model (RIGID, TMEASY, PAC02)
tire_model = veh.TireModelType_PAC02

# Poon chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 15

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = "./UAZBUS"


# =============================================================================


print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

# Create the vehicle, set parameters, and initialize
uaz = veh.UAZBUS()
uaz.SetContactMethod(chrono.ChContactMethod_NSC)
uaz.SetChassisFixed(False)
uaz.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
uaz.SetTireType(tire_model)
uaz.SetTireStepSize(tire_step_size)
uaz.SetInitFwdVel(0.0)
uaz.Initialize()

uaz.SetChassisVisualizationType(chassis_vis_type)
uaz.SetSuspensionVisualizationType(suspension_vis_type)
uaz.SetSteeringVisualizationType(steering_vis_type)
uaz.SetWheelVisualizationType(wheel_vis_type)
uaz.SetTireVisualizationType(tire_vis_type)

suspF = veh.CastToChToeBarLeafspringAxle(uaz.GetVehicle().GetSuspension(0))
leftAngle = suspF.GetKingpinAngleLeft()
rightAngle = suspF.GetKingpinAngleRight()

springFL = suspF.GetSpring(veh.LEFT)
shockFL = suspF.GetShock(veh.RIGHT)

print( "Spring rest length front: " + str(springFL.GetRestLength() ) + "\n")
print( "Shock rest length front:  " + str(shockFL.GetRestLength() ) + "\n")
suspR = veh.CastToChLeafspringAxle(uaz.GetVehicle().GetSuspension(1))
springRL = suspR.GetSpring(veh.LEFT)
shockRL = suspR.GetShock(veh.RIGHT)

print( "Spring rest length rear: " + str(springRL.GetRestLength() ) + "\n" )
print( "Shock rest length rear:  " + str(shockRL.GetRestLength() ) + "\n" )

print("Vehicle mass:               " + str( uaz.GetVehicle().GetVehicleMass() ) + "\n")
print("Vehicle mass (with tires):  " + str(uaz.GetTotalMass() ) + "\n")

# ------------------
# Create the terrain
# ------------------

terrain = veh.RigidTerrain(uaz.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 
                         600, 600)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 1200, 1200)
terrain.Initialize()

# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

app = veh.ChWheeledVehicleIrrApp(uaz.GetVehicle(), 'UAZBUS')
app.AddTypicalLights()

app.SetChaseCamera(trackPoint, 6.0, 0.5)
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(app)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# -----------------
# Initialize output
# -----------------
#if not os.path.isdir(out_dir):
#    os.makedirs(out_dir)
#assert os.path.isdir(out_dir),  "Error creating directory "

# ---------------
# Simulation loop
# ---------------

render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

maxKingpinAngle = 0.0

realtime_timer = chrono.ChRealtimeStepTimer()
while (app.GetDevice().run()) :
    time = uaz.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0) :
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()


    # Collect output data from modules (for inter-module communication)
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    uaz.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    # Test for validity of kingpin angles (max.allowed by UAZ: 27deg)
    suspF = veh.CastToChToeBarLeafspringAxle(uaz.GetVehicle().GetSuspension(0))
    leftAngle = suspF.GetKingpinAngleLeft() * 180.0 / chrono.CH_C_PI
    rightAngle = suspF.GetKingpinAngleRight() * 180.0 / chrono.CH_C_PI
    if abs(leftAngle) > maxKingpinAngle :
        maxKingpinAngle = abs(leftAngle)
    if abs(rightAngle) > maxKingpinAngle :
        maxKingpinAngle = abs(rightAngle)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    uaz.Advance(step_size)
    app.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

print( "Maximum Kingpin Angle = " + str(maxKingpinAngle ) + " deg \n" )