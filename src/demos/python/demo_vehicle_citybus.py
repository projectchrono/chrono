# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simone Benatti
# =============================================================================
#
# Main driver function for the City Bus full model.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath('../../../../Library/data/')
veh.SetDataPath('../../../../Library/data/Vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.ChassisCollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height (FLAT terrain only)
terrainLength = 200.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChMaterialSurface.SMC
contact_vis = False

# Simulation step sizes
step_size = 3e-3
tire_step_size = step_size

# Simulation end time
t_end = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = chrono.GetChronoOutputPath() + "CityBus"

# POV-Ray output
povray_output = False

# =============================================================================

#print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# --------------
# Create systems
# --------------

# Create the City Bus vehicle, set parameters, and initialize
my_bus = veh.CityBus()
my_bus.SetContactMethod(contact_method)
my_bus.SetChassisCollisionType(chassis_collision_type)
my_bus.SetChassisFixed(False)
my_bus.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
my_bus.SetTireType(tire_model)
my_bus.SetTireStepSize(tire_step_size)
my_bus.SetVehicleStepSize(step_size)
my_bus.Initialize()

tire_vis_type = veh.VisualizationType_MESH  # : VisualizationType::PRIMITIVES

my_bus.SetChassisVisualizationType(chassis_vis_type)
my_bus.SetSuspensionVisualizationType(suspension_vis_type)
my_bus.SetSteeringVisualizationType(steering_vis_type)
my_bus.SetWheelVisualizationType(wheel_vis_type)
my_bus.SetTireVisualizationType(tire_vis_type)

# Create the terrain
terrain = veh.RigidTerrain(my_bus.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight - 5), chrono.QUNIT),
                             chrono.ChVectorD(terrainLength, terrainWidth, 10))

patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
app = veh.ChWheeledVehicleIrrApp(my_bus.GetVehicle())
app.SetSkyBox()
app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
app.SetChaseCamera(trackPoint, 12.0, 0.5)
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()

# Create the driver system
driver = veh.ChIrrGuiDriver(app)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  my_bus.GetVehicle().GetVehicleMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter and simulation time
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
time = 0

while (app.GetDevice().run()) :
    time = my_bus.GetSystem().GetChTime()

    # End simulation
    if (time >= t_end):
        break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
        app.EndScene()
        render_frame += 1
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    my_bus.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    # Advance simulation for one timestep for all modules
    step = realtime_timer.SuggestSimulationStep(step_size)
    driver.Advance(step)
    terrain.Advance(step)
    my_bus.Advance(step)
    app.Advance(step)

    # Increment frame number
    step_number += 1

del app