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
# Chrono demonstration of a camera sensor.
# Generates a mesh object and rotates camera sensor around the mesh.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------------------
    # add a mesh to be sensed by a camera
    # -----------------------------------
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    # scale to a different size
    mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(2))

    trimesh_shape = chrono.ChTriangleMeshShape()
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

    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-5, 0, 2), chrono.Q_from_AngAxis(2, chrono.ChVectorD(0, 1, 0)))
    cam = sens.ChCameraSensor(
        mesh_body,              # body camera is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                    # camera's horizontal field of view
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    # ------------------------------------------------------------------
    # Create a filter graph for post-processing the data from the camera
    # ------------------------------------------------------------------
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    # Renders the image at current point in the filter graph
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "Before Grayscale Filter"))

    # Provides the host access to this RGBA8 buffer
    cam.PushFilter(sens.ChFilterRGBA8Access())

    # Save the current image to a png file at the specified path
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Filter the sensor to grayscale
    cam.PushFilter(sens.ChFilterGrayscale())

    # Render the buffer again to see the new grayscaled image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    # Save the grayscaled image at the specified path
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resizes the image to the provided width and height
    cam.PushFilter(sens.ChFilterImageResize(
        int(image_width / 2), int(image_height / 2)))

    # Access the grayscaled buffer as R8 pixels
    cam.PushFilter(sens.ChFilterR8Access())

    # add sensor to manager
    manager.AddSensor(cam)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.5
    ch_time = 0.0

    t1 = time.time()

    while (ch_time < end_time):
        cam.SetOffsetPose(chrono.ChFrameD(
            chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate), -
                             orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))))

        # Access the RGBA8 buffer from the camera
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if (rgba8_buffer.HasData()):
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer recieved from cam. Camera resolution: {0}x{1}'
                  .format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Camera parameters
# -----------------


# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model="CONST_NORMAL"      # Gaussian noise with constant mean and standard deviation
# noise_model="PIXEL_DEPENDENT"   # Pixel dependent gaussian noise
# noise_model="RESPONSE_FUNCTION" # Noise model based on camera's response and parameters
noise_model = "NONE"              # No noise model

# Camera lens model
# Either PINHOLE or FOV_LENS
lens_model = sens.PINHOLE

# Update rate in Hz
update_rate = 30

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = 1.408

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0

# Exposure (in seconds) of each image
exposure_time = 0

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
