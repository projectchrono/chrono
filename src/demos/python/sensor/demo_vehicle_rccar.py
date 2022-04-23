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

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath('../../../data/')
veh.SetDataPath('../../../data/vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(2, 2, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height (FLAT terrain only)
terrainLength = 200.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.2)

# Contact method
contact_method = chrono.ChMaterialSurface.NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
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

# Create the RCCar vehicle, set parameters, and initialize
my_rccar = veh.RCCar()
my_rccar.SetContactMethod(contact_method)
my_rccar.SetChassisCollisionType(chassis_collision_type)
my_rccar.SetChassisFixed(True)
my_rccar.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
my_rccar.SetTireType(tire_model)
my_rccar.SetTireStepSize(tire_step_size)
my_rccar.Initialize()

tire_vis_type = veh.VisualizationType_PRIMITIVES  # : VisualizationType::PRIMITIVES

my_rccar.SetChassisVisualizationType(chassis_vis_type)
my_rccar.SetSuspensionVisualizationType(suspension_vis_type)
my_rccar.SetSteeringVisualizationType(steering_vis_type)
my_rccar.SetWheelVisualizationType(wheel_vis_type)
my_rccar.SetTireVisualizationType(tire_vis_type)

# Create the terrain
terrain = veh.RigidTerrain(my_rccar.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight - 5), chrono.QUNIT), chrono.ChVectorD(terrainLength, terrainWidth, 10))

patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()
# terrain = veh.RigidTerrain(my_rccar.GetSystem())
# patch = terrain.AddPatch(chrono.CSYSNORM, chrono.GetChronoDataFile("sensor/textures/hallway.obj"), "mesh", 0.01, False)
#
# vis_mesh = chrono.ChTriangleMeshConnected()
# vis_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/textures/hallway.obj"), True, True)
#
# trimesh_shape = chrono.ChTriangleMeshShape()
# trimesh_shape.SetMesh(vis_mesh)
# trimesh_shape.SetName("mesh_name")
# trimesh_shape.SetMutable(False)
#
# patch.GetGroundBody().AddVisualShape(trimesh_shape)
#
# patch.SetContactFrictionCoefficient(0.9)
# patch.SetContactRestitutionCoefficient(0.01)
# patch.SetContactMaterialProperties(2e7, 0.3)
# # patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
#
# terrain.Initialize()

# Create the vehicle Irrlicht interface
#vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
#my_rccar.GetVehicle().SetVisualSystem(vis)
#vis.SetWindowTitle('RCcar')
#vis.SetWindowSize(1280, 1024)
#vis.SetChaseCamera(trackPoint, 1.5, 0.5)
#vis.Initialize()
#vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
#vis.AddTypicalLights()


# Create the driver system
# driver = veh.ChIrrGuiDriver(vis)
driver = veh.ChDriver(my_rccar.GetVehicle())

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
# driver.SetSteeringDelta(render_step_size / steering_time)
# driver.SetThrottleDelta(render_step_size / throttle_time)
# driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Create sensor manager
manager = sens.ChSensorManager(my_rccar.GetSystem())
f = 3
for i in range(8):
    manager.scene.AddPointLight(chrono.ChVectorF(f,1.25,2.3),chrono.ChColor(1,1,1),5)
    manager.scene.AddPointLight(chrono.ChVectorF(f,3.75,2.3),chrono.ChColor(1,1,1),5)
    f += 3

factor = 2
cam = sens.ChCameraSensor(
    my_rccar.GetChassisBody(),                                                          # body lidar is attached to
    30,                                                                 # scanning rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, .5), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),  # offset pose
    1920*factor,                                                               # number of horizontal samples
    1080*factor,                                                                 # number of vertical channels
    chrono.CH_C_PI / 4,                                                            # horizontal field of view
    (720/1280)*chrono.CH_C_PI / 4.                                                        # vertical field of view
)
cam.SetName("Camera Sensor")
# cam.FilterList().append(sens.ChFilterImgAlias(factor))
# cam.FilterList().append(sens.ChFilterVisualize(1920, 1080, "Third Person Camera"))
# cam.FilterList().append(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)

cam2 = sens.ChCameraSensor(
    my_rccar.GetChassisBody(),                                                          # body lidar is attached to
    30,                                                                 # scanning rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),  # offset pose
    1920,                                                               # number of horizontal samples
    1080,                                                                 # number of vertical channels
    chrono.CH_C_PI / 4,                                                            # horizontal field of view
    (720/1280)*chrono.CH_C_PI / 4.                                                        # vertical field of view
)
cam2.SetName("Camera Sensor")
# cam2.FilterList().append(sens.ChFilterVisualize("Birds Eye Camera"))
cam2.FilterList().append(sens.ChFilterRGBA8Access())
manager.AddSensor(cam2)

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  my_rccar.GetVehicle().GetVehicleMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while True :
    print('test')
    # if not vis.Run():
    #     break
    time = my_rccar.GetSystem().GetChTime()

    # End simulation
    if (time >= t_end):
        break

    # Render scene and output POV-Ray data
    # if (step_number % render_steps == 0) :
    #     vis.BeginScene()
    #     vis.DrawAll()
    #     vis.EndScene()
    #     render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    my_rccar.Synchronize(time, driver_inputs, terrain)
    # vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_rccar.Advance(step_size)
    # vis.Advance(step_size)
    my_rccar.GetSystem().DoStepDynamics(step_size)

    manager.Update()

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

