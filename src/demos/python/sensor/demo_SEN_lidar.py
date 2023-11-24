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
# Chrono demonstration of a lidar sensor
# Simple demonstration of certain filters and the visualization of a static mesh
#
# =============================================================================

import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():

    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # add a mesh to be sensed by a lidar
    # ----------------------------------
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    # scale to a different size
    mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(2))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetBodyFixed(True)
    mphysicalSystem.Add(mesh_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        mesh_body,              # body lidar is attached to
        update_rate,            # scanning rate in Hz
        offset_pose,            # offset pose
        horizontal_samples,     # number of horizontal samples
        vertical_samples,       # number of vertical channels
        horizontal_fov,         # horizontal field of view
        max_vert_angle,         # vertical field of view
        min_vert_angle,
        100.0,  # max lidar range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,          # sample radius
        divergence_angle,       # divergence angle
        divergence_angle,       # divergence angle
        return_mode            # return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(
            horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Provides the host access to the Depth,Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth,Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(
            640, 480, 1.0, "Lidar Point Cloud"))

    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 5
    orbit_rate = 0.2
    ch_time = 0.0

    render_time = 0

    t1 = time.time()

    while (ch_time < end_time):
        lidar.SetOffsetPose(chrono.ChFrameD(
            chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate), -
                             orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))))

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer recieved from lidar. Lidar resolution: {0}x{1}'
                  .format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time()-t1)

# -----------------
# Lidar parameters
# -----------------


# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model="CONST_NORMAL_XYZI"   # Gaussian noise with constant mean and standard deviation
noise_model = "NONE"                  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 4500
vertical_samples = 32

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample,2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

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
out_dir = "SENSOR_OUTPUT/"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()
