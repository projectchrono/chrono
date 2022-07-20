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

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
# import cv2

from control_utilities.track import RandomTrack, Track

import numpy as np
# import cv2


save_data = False
visualize = True

"""
!!!! Set this path before running the demo!
"""
# chrono.SetChronoDataPath('../../../data/')
veh.SetDataPath(chrono.GetChronoDataPath()+"vehicle/")

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
# terrain_model = veh.RigidTerrainType_BOX
terrainHeight = .5      # terrain height (FLAT terrain only)
terrainLength = 400.0  # size in X direction
terrainWidth = 400.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.2)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
t_end = 200

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50
control_step_size = 1.0/50  # 50 Hz


# f = plt.figure()

# PID controller
prev_s = 0
prev_t = 0
prev_b = 0

ks = 0.75
kt = 0.5
kb = 0.5


def ControlCar(steering, velocity, img, lidar_depth, pt_cloud):
    if(img.shape[0] < 2 or pt_cloud.shape[0] < 2):
        return 0, 0, 0

    z_ground_param = -.2
    side_margin = 0.1
    distance_param = 5
    intensity_param = 0.01

    # indices = pt_cloud[:,:,3]>intensity_param
    ptc_x = (pt_cloud[:, :, 0][pt_cloud[:, :, 3] > intensity_param]).flatten()
    ptc_y = (pt_cloud[:, :, 1][pt_cloud[:, :, 3] > intensity_param]).flatten()
    ptc_z = (pt_cloud[:, :, 2][pt_cloud[:, :, 3] > intensity_param]).flatten()

    # print("shapes:",ptc_x.shape,ptc_y.shape,ptc_z.shape)

    # remove points on ground
    ptc_x = ptc_x[ptc_z > z_ground_param]
    ptc_y = ptc_y[ptc_z > z_ground_param]

    # remove points too far in distance
    ptc_y = ptc_y[ptc_x < distance_param]

    left_pc_points = ptc_y[ptc_y < -side_margin]
    right_pc_points = ptc_y[ptc_y > side_margin]
    if(left_pc_points.shape[0] == 0):
        left_pc_points = -np.ones((1, 1))

    if(right_pc_points.shape[0] == 0):
        right_pc_points = np.ones((1, 1))

    print("Left pts:", left_pc_points.shape)
    print("Right pts:", right_pc_points.shape)

    print("Mean left:", np.mean(left_pc_points))
    print("Mean right:", np.mean(right_pc_points))

    mid_pt = (np.mean(left_pc_points) + np.mean(right_pc_points)) / 2

    # print("image dimensions:",img.shape)
    # cv2.imshow("",img)
    # cv2.waitKey(0)

    x = mid_pt

    target_s = np.clip(x, -1, 1)
    target_b = 0
    target_v = 5

    # PID control to target
    s = (target_s - steering) * ks
    t = (target_v - velocity) * kt
    # b = (target_b - prev_b) * kb

    s = np.clip(s, -1, 1)
    t = np.clip(t, 0, 1)

    # print("X=",x,", Target s=",target_s,", Command s=",s)

    return s, t, 0
    # return 0,0,0

# track = RandomTrack(width=1,x_max=50,y_max=50)
# track.generateTrack(seed=np.random.randint(0,10000))


points = [
    [49.8, 132.9],
    [60.3, 129.3],
    [75.6, 129.0],
    [87.9, 131.7],
    [96.9, 129.6],
    [111.0, 120.0],
    [115.2, 110.7],
    [120.6, 96.9],
    [127.8, 88.5],
    [135.9, 77.4],
    [135.9, 65.1],
    [133.2, 51.3],
    [128.4, 43.2],
    [119.7, 36.3],
    [105.0, 35.7],
    [90.0, 36.3],
    [82.5, 46.2],
    [82.5, 63.6],
    [83.4, 82.2],
    [77.1, 93.9],
    [61.2, 88.5],
    [55.5, 73.5],
    [57.9, 54.6],
    [66.6, 45.0],
    [75.9, 36.3],
    [79.2, 25.5],
    [78.0, 13.2],
    [65.1, 6.0],
    [50.7, 6.0],
    [36.6, 11.7],
    [29.1, 21.3],
    [24.0, 36.9],
    [24.0, 56.1],
    [29.1, 70.8],
    [24.9, 77.7],
    [13.5, 77.7],
    [6.3, 81.6],
    [5.7, 92.7],
    [6.3, 107.7],
    [8.7, 118.2],
    [15.3, 122.7],
    [24.3, 125.4],
    [31.2, 126.0],
    [40.8, 129.6],
    [49.8, 132.9]
]
track = Track(points, width=1)
track.generateTrack()


l1 = track.left.points[0]
r1 = track.right.points[0]

v1 = track.right.points[1]-track.right.points[0]
v2 = chrono.ChVectorD(1, 0, 0)
ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
if (chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0):
    ang *= -1
q = chrono.Q_from_AngZ(ang)

initLoc = chrono.ChVectorD((l1 + r1) / 2) + chrono.ChVectorD(0, 0, .20)
initRot = q

# Create the RCCar vehicle, set parameters, and initialize
my_rccar = veh.RCCar()
my_rccar.SetContactMethod(contact_method)
my_rccar.SetChassisCollisionType(chassis_collision_type)
my_rccar.SetChassisFixed(False)
my_rccar.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
my_rccar.SetTireType(tire_model)
my_rccar.SetTireStepSize(tire_step_size)
my_rccar.Initialize()


tire_vis_type = veh.VisualizationType_MESH  # : VisualizationType::PRIMITIVES

my_rccar.SetChassisVisualizationType(chassis_vis_type)
my_rccar.SetSuspensionVisualizationType(suspension_vis_type)
my_rccar.SetSteeringVisualizationType(steering_vis_type)
my_rccar.SetWheelVisualizationType(wheel_vis_type)
my_rccar.SetTireVisualizationType(tire_vis_type)

if my_rccar.GetChassisBody().GetVisualShape(0):
    vis_mat = chrono.ChVisualMaterial()
    vis_mat.SetAmbientColor(chrono.ChColor(0, 0, 0))
    vis_mat.SetDiffuseColor(chrono.ChColor(.5, .5, .5))
    vis_mat.SetSpecularColor(chrono.ChColor(1, 1, 1))
    vis_mat.SetFresnelMin(0)
    vis_mat.SetFresnelMax(1)
    my_rccar.GetChassisBody().GetVisualShape(0).SetMaterial(0, vis_mat)

# Create the terrain
terrain = veh.RigidTerrain(my_rccar.GetSystem())
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

vis_mat = chrono.ChVisualMaterial()
# vis_mat.SetAmbientColor(chrono.ChColor(0, 0, 0))
vis_mat.SetKdTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"))
vis_mat.SetSpecularColor(chrono.ChColor(.0, .0, .0))
vis_mat.SetFresnelMin(0)
vis_mat.SetFresnelMax(.1)
patch.GetGroundBody().GetVisualShape(0).SetMaterial(0, vis_mat)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('RC car')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 1.5, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AttachVehicle(my_rccar.GetVehicle())

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

# ---------------
# Simulation loop
# ---------------

for points in (track.left.points, track.right.points):
    for i in range(len(points)-1):
        p1 = points[i]+chrono.ChVectorD(0, 0, .5)
        p2 = points[i+1]
        box = chrono.ChBodyEasyBox(
            (p2-p1).Length()/10, .2, .8, 1000, True, True)
        box.SetPos(p1)

        v1 = p2-p1
        v2 = chrono.ChVectorD(1, 0, 0)
        ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
        if (chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0):
            ang *= -1
        q = chrono.Q_from_AngZ(ang)
        box.SetRot(q)
        box.SetBodyFixed(True)

        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetAmbientColor(chrono.ChColor(0, 0, 0))

        if i % 2 == 0:
            vis_mat.SetDiffuseColor(chrono.ChColor(.8, 0, 0))
        else:
            vis_mat.SetDiffuseColor(chrono.ChColor(.8, .8, .8))
        vis_mat.SetSpecularColor(chrono.ChColor(.2, .2, .2))
        vis_mat.SetFresnelMin(0)
        vis_mat.SetFresnelMax(.7)

        bos.GetVisualShape(0).SetMaterial(0, vis_mat)

        my_rccar.GetSystem().Add(box)
        print("Added box to world")


vis.BindAll()

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

control_steps = math.ceil(control_step_size / step_size)

# Initialize simulation frame counter s
# realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


manager = sens.ChSensorManager(my_rccar.GetSystem())
manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVectorF(-100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)


# field of view:
fov = 1.408
camera_3rd = sens.ChCameraSensor(
    # body lidar is attached to
    my_rccar.GetChassisBody(),
    # scanning rate in Hz
    60,
    chrono.ChFrameD(chrono.ChVectorD(-2, 0, 1),
                    chrono.Q_from_AngAxis(.3, chrono.ChVectorD(0, 1, 0))),  # offset pose
    # number of horizontal samples
    1920*2,
    # number of vertical channels
    1080*2,
    # horizontal field of view                                                     # vertical field of view
    chrono.CH_C_PI / 4
)
camera_3rd.SetName("Camera Sensor")
if(visualize):
    camera_3rd.PushFilter(sens.ChFilterVisualize(
        1280, 720, "Third Person Camera"))
if(save_data):
    camera_3rd.PushFilter(sens.ChFilterSave(
        1280, 720, "output/iros/third_person_camera/"))
manager.AddSensor(camera_3rd)

camera_front = sens.ChCameraSensor(
    # body lidar is attached to
    my_rccar.GetChassisBody(),
    # scanning rate in Hz
    60,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, .2),
                    chrono.Q_from_AngAxis(0, chrono.ChVectorD(1, 0, 0))),  # offset pose
    # number of horizontal samples
    210*2,
    # number of vertical channels
    160*2,
    # camera's horizontal field of view
    fov
)
camera_front.SetName("Camera Sensor")

camera_front.PushFilter(sens.ChFilterImgAlias(2))
if(visualize):
    camera_front.PushFilter(sens.ChFilterVisualize(210, 160, "Front Camera"))
if(save_data):
    camera_front.PushFilter(sens.ChFilterSave("output/iros/front_camera/"))
camera_front.PushFilter(sens.ChFilterRGBA8Access())
camera_front.PushFilter(sens.ChFilterGrayscale())
camera_front.PushFilter(sens.ChFilterR8Access())
manager.AddSensor(camera_front)


lidar = sens.ChLidarSensor(
    # body lidar is attached to
    my_rccar.GetChassisBody(),
    # scanning rate in Hz
    30,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, .3), chrono.Q_from_AngAxis(
        0, chrono.ChVectorD(0, 1, 0))),  # offset pose
    # number of horizontal samples
    1000,
    # number of vertical channels
    200,
    # horizontal field of view
    chrono.CH_C_PI / 2,
    chrono.CH_C_PI / 24.,
    # vertical field of view
    -chrono.CH_C_PI / 24.,
    100  # max lidar range
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
if(visualize):
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0))
if(save_data):
    lidar.PushFilter(sens.ChFilterSavePtCloud("output/iros/lidar/"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


camera_data_RGBA8 = np.zeros(1)
camera_data_R8 = np.zeros(1)
lidar_data_XYZI = np.zeros(1)
lidar_data_DI = np.zeros(1)

steering = 0
throttle = 0
braking = 0

while vis.Run():
    t = my_rccar.GetSystem().GetChTime()

    manager.Update()

    # End simulation
    if (t >= t_end):
        break

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    if (step_number % control_steps == 0):
        lidar_buffer_XYZI = lidar.GetMostRecentXYZIBuffer()
        lidar_buffer_DI = lidar.GetMostRecentDIBuffer()
        camera_buffer_RGBA8 = camera_front.GetMostRecentRGBA8Buffer()
        camera_buffer_R8 = camera_front.GetMostRecentR8Buffer()

        if camera_buffer_R8.HasData():
            camera_data_R8 = camera_buffer_R8.GetChar8Data()  # .astype(np.float32)

            # print("R8: Camera datatype:",camera_data_R8.dtype)
            # print("R8: Max:",np.max(camera_data_R8))
            # print("R8: Min:",np.min(camera_data_R8))
            #
            # print("R8: Number of nonzeros:",np.count_nonzero(camera_data_R8[:,:]))
            # print("R8: Number of nonzero rows:",np.count_nonzero(camera_data_R8[:,:])/1280)
            #
            # plt.imshow((camera_data_R8)[:,:,0])
            # plt.show()

        if camera_buffer_RGBA8.HasData():
            camera_data_RGBA8 = camera_buffer_RGBA8.GetRGBA8Data()  # .astype(np.float32)

            # print("Camera datatype:",camera_data_RGBA8.dtype)
            # print("Max:",np.max(camera_data_RGBA8))
            # print("Min:",np.min(camera_data_RGBA8))
            #
            # print("Number of nonzeros:",np.count_nonzero(camera_data_RGBA8[:,:,0]))
            # print("Number of nonzero rows:",np.count_nonzero(camera_data_RGBA8[:,:,0])/1280)
            #
            # plt.imshow((camera_data_RGBA8)[:,:,0:3])
            # plt.show()

        if lidar_buffer_XYZI.HasData():
            lidar_data_XYZI = np.copy(lidar_buffer_XYZI.GetXYZIData())
            print("\n\nUpdate frame:", lidar_buffer_XYZI.LaunchedCount)

        if lidar_buffer_DI.HasData():

            lidar_data_DI = lidar_buffer_DI.GetDIData()
            # print("lidar datatype:",lidar_data_DI.dtype)
            # print("Max:",np.max(lidar_data_DI))
            # print("Min:",np.min(lidar_data_DI))
            # plt.imshow((lidar_data_DI/200.0)[:,:,0])
            # plt.show()

        velocity = my_rccar.GetVehicle().GetVehicleSpeed()
        steering, throttle, braking = ControlCar(
            steering, velocity, camera_data_RGBA8, lidar_data_DI, np.copy(lidar_data_XYZI))

    # driver.SetSteering(steering)
    # driver.SetThrottle(thottle)
    # driver.SetBraking(braking)

    # driver_inputs = driver.GetInputs()
    driver_inputs = veh.Inputs()  # steering,thottle,braking)
    driver_inputs.m_steering = steering
    driver_inputs.m_throttle = throttle
    driver_inputs.m_braking = braking

    # Update modules (process inputs from other modules)
    # driver.Synchronize(t)
    terrain.Synchronize(t)
    my_rccar.Synchronize(t, driver_inputs, terrain)
    vis.Synchronize("lidar driver", driver_inputs)

    # Advance simulation for one timestep for all modules
    # driver.Advance(step_size)
    terrain.Advance(step_size)
    my_rccar.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    # realtime_timer.Spin(step_size)
