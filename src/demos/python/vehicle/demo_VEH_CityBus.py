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

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height (FLAT terrain only)
terrainLength = 200.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 3e-3
tire_step_size = step_size

# Simulation end time
t_end = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

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
my_bus.Initialize()

tire_vis_type = veh.VisualizationType_MESH  # : VisualizationType::PRIMITIVES

my_bus.SetChassisVisualizationType(chassis_vis_type)
my_bus.SetSuspensionVisualizationType(suspension_vis_type)
my_bus.SetSteeringVisualizationType(steering_vis_type)
my_bus.SetWheelVisualizationType(wheel_vis_type)
my_bus.SetTireVisualizationType(tire_vis_type)

# Create the terrain
terrain = veh.RigidTerrain(my_bus.GetSystem())
if (contact_method == chrono.ChContactMethod_NSC):
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
elif (contact_method == chrono.ChContactMethod_SMC):
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
app = veh.ChWheeledVehicleIrrApp(my_bus.GetVehicle(), 'Citybus')
app.AddTypicalLights()
app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.SetChaseCamera(trackPoint, 15.0, 0.5)
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

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while (app.GetDevice().run()) :
    time = my_bus.GetSystem().GetChTime()

    # End simulation
    if (time >= t_end):
        break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
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
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_bus.Advance(step_size)
    app.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

del app
