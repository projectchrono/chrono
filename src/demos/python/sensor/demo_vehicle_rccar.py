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
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

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
contact_method = chrono.ChContactMethod_NSC
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
my_rccar.SetChassisFixed(False)
my_rccar.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
my_rccar.SetTireType(tire_model)
my_rccar.SetTireStepSize(tire_step_size)
my_rccar.SetMaxMotorVoltageRatio(0.16)
my_rccar.SetStallTorque(0.3)
my_rccar.SetTireRollingResistance(0.06)
my_rccar.SetMotorResistanceCoefficients(0.02, 1e-4)


my_rccar.Initialize()

tire_vis_type = veh.VisualizationType_PRIMITIVES  # : VisualizationType::PRIMITIVES

my_rccar.SetChassisVisualizationType(chassis_vis_type)
my_rccar.SetSuspensionVisualizationType(suspension_vis_type)
my_rccar.SetSteeringVisualizationType(steering_vis_type)
my_rccar.SetWheelVisualizationType(wheel_vis_type)
my_rccar.SetTireVisualizationType(tire_vis_type)

# Create the terrain
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(my_rccar.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight - 5), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# -------------------------------------
# Create the vehicle Irrlicht interface
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('dart')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(my_rccar.GetVehicle())


driver_data = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0), 
                                veh.DataDriverEntry(0.1, 1.0, 0.0, 0.0),
                                veh.DataDriverEntry(0.5, 1.0, 0.7, 0.0),
                                 ])
driver = veh.ChDataDriver(my_rccar.GetVehicle(), driver_data)
driver.Initialize()

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
    my_rccar.GetChassisBody(),                                          # body lidar is attached to
    30,                                                                 # scanning rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, .5), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),  # offset pose
    1920*factor,                                                        # number of horizontal samples
    1080*factor,                                                        # number of vertical channels
    chrono.CH_C_PI / 4                                                  # horizontal field of view
)
cam.SetName("Camera Sensor")
# cam.FilterList().append(sens.ChFilterImgAlias(factor))
# cam.FilterList().append(sens.ChFilterVisualize(1920, 1080, "Third Person Camera"))
# cam.FilterList().append(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)

cam2 = sens.ChCameraSensor(
    my_rccar.GetChassisBody(),                                          # body lidar is attached to
    30,                                                                 # scanning rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))),  # offset pose
    1920,                                                               # number of horizontal samples
    1080,                                                               # number of vertical channels
    chrono.CH_C_PI / 4                                                  # horizontal field of view
)
cam2.SetName("Camera Sensor")
# cam2.FilterList().append(sens.ChFilterVisualize("Birds Eye Camera"))
cam2.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam2)

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print( "VEHICLE MASS: ",  my_rccar.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while True :
    time = my_rccar.GetSystem().GetChTime()

    # End simulation
    if (time >= t_end):
        break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

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

