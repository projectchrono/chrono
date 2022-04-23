# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Asher Elmquist
# =============================================================================
#
# Chrono demonstration of a multiple sensors
# Creates a few bodies to be sensed
#
# =============================================================================

import pychrono.core as chrono
import pychrono.sensor as sens

import time
import math
import numpy as np

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.Set_G_acc(chrono.ChVectorD(0,0,-9.81))

    # ----------------------------------------
    # add a floor, box and sphere to the scene
    # ----------------------------------------
    phys_mat = chrono.ChMaterialSurfaceNSC()
    phys_mat.SetFriction(0.5)
    phys_mat.SetDampingF(0.00000)
    phys_mat.SetCompliance (1e-9)
    phys_mat.SetComplianceT(1e-9)

    floor = chrono.ChBodyEasyBox(10,10,1,1000,True,True,phys_mat)
    floor.SetPos(chrono.ChVectorD(0,0,-1))
    floor.SetBodyFixed(True)
    mphysicalSystem.Add(floor)

    box = chrono.ChBodyEasyBox(1,1,1,1000,True,True,phys_mat)
    box.SetPos(chrono.ChVectorD(0,0,5))
    box.SetRot(chrono.Q_from_AngAxis(.2,chrono.ChVectorD(1,0,0)))
    mphysicalSystem.Add(box)

    sphere = chrono.ChBodyEasySphere(.5,1000,True,True,phys_mat)
    sphere.SetPos(chrono.ChVectorD(0,0,8))
    sphere.SetRot(chrono.Q_from_AngAxis(.2,chrono.ChVectorD(1,0,0)))
    mphysicalSystem.Add(sphere)

    vis_mat = chrono.ChVisualMaterial()
    vis_mat.SetAmbientColor(chrono.ChColor(0, 0, 0))
    vis_mat.SetDiffuseColor(chrono.ChColor(.2,.2,.9))
    vis_mat.SetSpecularColor(chrono.ChColor(.9,.9,.9))
    sphere.GetVisualShape(0).SetMaterial(0, vis_mat)


    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.scene.AddPointLight(chrono.ChVectorF(100,100,100),chrono.ChColor(1,1,1),1000.0)
    manager.scene.AddPointLight(chrono.ChVectorF(-100,-100,100),chrono.ChColor(1,1,1),1000.0)


    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    cam = sens.ChCameraSensor(
        floor,                  # body camera is attached to
        cam_update_rate,        # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # number of horizontal samples
        image_height,           # number of vertical channels
        cam_fov                # vertical field of view
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(cam_lag)
    cam.SetCollectionWindow(cam_collection_time)

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------------------------
    # Visualizes the image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "RGB Image"))

    # Save the current image to a png file at the specified path
    if (save):
        cam.PushFilter(sens.ChFilterSave(out_dir + "/rgb/"))

    # Provides the host access to this RGBA8 buffer
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Filter the sensor to grayscale
    cam.PushFilter(sens.ChFilterGrayscale());

    # Render the buffer again to see the new grayscaled image
    if (vis):
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save the grayscaled image at the specified path
    if (save):
        cam.PushFilter(sens.ChFilterSave(out_dir + "/gray/"))

    # Access the grayscaled buffer as R8 pixels
    cam.PushFilter(sens.ChFilterR8Access())

    # Add a camera to a sensor manager
    manager.AddSensor(cam)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        floor,                  # body lidar is attached to
        lidar_update_rate,      # scanning rate in Hz
        offset_pose,            # offset pose
        horizontal_samples,     # number of horizontal samples
        vertical_samples,       # number of vertical channels
        horizontal_fov,         # horizontal field of view
        max_vert_angle,
        min_vert_angle,           # vertical field of view
        100 #max lidar range
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lidar_lag)
    lidar.SetCollectionWindow(lidar_collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if vis:
        # Randers the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Provides the host access to the Depth,Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth,Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # ----------------------------------------------
    # Create an IMU sensor and add it to the manager
    # ----------------------------------------------
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    acc = sens.ChAccelerometerSensor(box,                     # body imu is attached to
                           imu_update_rate,         # update rate in Hz
                           offset_pose,             # offset pose
                           noise_none,          # noise model
                           )
    acc.SetName("Accelerometer")
    acc.SetLag(imu_lag)
    acc.SetCollectionWindow(imu_collection_time)

    # Provides the host access to the imu data
    acc.PushFilter(sens.ChFilterAccelAccess())

    # Add the imu to the sensor manager
    manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(box,                     # body imu is attached to
                           imu_update_rate,         # update rate in Hz
                           offset_pose,             # offset pose
                           noise_none,          # noise model
                           )
    gyro.SetName("Gyroscope")
    gyro.SetLag(imu_lag)
    gyro.SetCollectionWindow(imu_collection_time)

    # Provides the host access to the imu data
    gyro.PushFilter(sens.ChFilterGyroAccess())

    # Add the imu to the sensor manager
    manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(box,                     # body imu is attached to
                           imu_update_rate,         # update rate in Hz
                           offset_pose,             # offset pose
                           noise_none,          # noise model
                           gps_reference
                           )
    mag.SetName("Magnetometer")
    mag.SetLag(imu_lag)
    mag.SetCollectionWindow(imu_collection_time)

    # Provides the host access to the imu data
    mag.PushFilter(sens.ChFilterMagnetAccess())

    # Add the imu to the sensor manager
    manager.AddSensor(mag)


    # ----------------------------------------------
    # Create an GPS sensor and add it to the manager
    # ----------------------------------------------
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    gps = sens.ChGPSSensor(box,                     # body imu is attached to
                           gps_update_rate,       # update rate in Hz
                           offset_pose,             # offset pose
                           gps_reference,
                           noise_none          # noise model
                           )
    gps.SetName("GPS Sensor")
    gps.SetLag(gps_lag)
    gps.SetCollectionWindow(gps_collection_time)

    # Provides the host access to the gps data
    gps.PushFilter(sens.ChFilterGPSAccess())

    # Add the gps to the sensor manager
    manager.AddSensor(gps)

    # ---------------
    # Simulate system
    # ---------------
    t1 = time.time()
    ch_time = 0
    while (ch_time < end_time):

        # Access the sensor data
        camera_data_RGBA8 = cam.GetMostRecentRGBA8Buffer()
        camera_data_R8 = cam.GetMostRecentR8Buffer()
        lidar_data_DI = lidar.GetMostRecentDIBuffer()
        lidar_data_XYZI = lidar.GetMostRecentXYZIBuffer()
        gps_data = gps.GetMostRecentGPSBuffer()
        acc_data = acc.GetMostRecentAccelBuffer()
        gyro_data = gyro.GetMostRecentGyroBuffer()
        mag_data = mag.GetMostRecentMagnetBuffer()

        # Check data is present
        # If so, print out the max value
        if camera_data_RGBA8.HasData():
            print("Camera RGBA8:",camera_data_RGBA8.GetRGBA8Data().shape,"max:",np.max(camera_data_RGBA8.GetRGBA8Data()))
        if camera_data_R8.HasData():
            print("Camera R8:",camera_data_R8.GetChar8Data().shape,"max:",np.max(camera_data_R8.GetChar8Data()))
        if lidar_data_DI.HasData():
            print("Lidar DI:",lidar_data_DI.GetDIData().shape,"max:",np.max(lidar_data_DI.GetDIData()))
        if lidar_data_XYZI.HasData():
            print("Lidar XYZI:",lidar_data_XYZI.GetXYZIData().shape,"max:",np.max(lidar_data_XYZI.GetXYZIData()))
        if gps_data.HasData():
            print("GPS:",gps_data.GetGPSData().shape,"max:",np.max(gps_data.GetGPSData()))
        if acc_data.HasData():
            print("Accelerometer:",acc_data.GetAccelData().shape,"max:",np.max(acc_data.GetAccelData()))
        if gyro_data.HasData():
            print("Gyroscope:",gyro_data.GetGyroData().shape,"max:",np.max(gyro_data.GetGyroData()))
        if mag_data.HasData():
            print("Magnetometer:",mag_data.GetMagnetData().shape,"max:",np.max(mag_data.GetMagnetData()))

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:",end_time,"Wall time:",time.time()-t1)

# -----------------
# Sensor parameters
# -----------------

# Update rate of each sensor in Hz
cam_update_rate = 5
lidar_update_rate = 5
imu_update_rate = 200
gps_update_rate = 2

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
cam_fov = 1.408

# Lidar horizontal and vertical samples
horizontal_samples = 4500
vertical_samples = 32

# Lidar horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees
max_vert_angle = chrono.CH_C_PI / 12.
min_vert_angle = -chrono.CH_C_PI / 6.


# Lag time for each sensor
cam_lag = 0
lidar_lag = 0
imu_lag = 0
gps_lag = 0

# Collection window for each sensor
# Typically 1 / update rate
cam_collection_time =  1. / float(cam_update_rate)
lidar_collection_time = 1. / float(lidar_update_rate)
imu_collection_time = 0 # instant
gps_collection_time = 0 # instant

# GPS reference point
# Located in Madison, WI
gps_reference = chrono.ChVectorD(-89.400, 43.070, 260.0)

# IMU and GPS noise models
# Setting to none (does not affect the data)
noise_none = sens.ChNoiseNone()

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 20.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/SENSORS_PY"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()
