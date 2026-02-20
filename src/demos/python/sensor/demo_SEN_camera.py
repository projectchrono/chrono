# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Bocheng Zou, Asher Elmquist
# =============================================================================
#
# Chrono demonstration of a camera sensor with different types of lights.
# Generates a mesh object and rotates camera sensor around the mesh.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time
import argparse
import sys
import os
from enum import Enum

# -----------------------------------------------------------------------------
# Camera parameters
# -----------------------------------------------------------------------------

# Noise model attached to the sensor
class NoiseModel(Enum):
    CONST_NORMAL = 1 # Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT = 2 # Pixel dependent gaussian noise
    NONE = 3 # No noise model

noise_model = NoiseModel.NONE

# Camera lens model
# Either PINHOLE or FOV_LENS
lens_model = sens.PINHOLE

# Update rate in Hz
update_rate = 30.0

# Image width and height
image_width = 1280
image_height = 720

# Camera's horizontal field of view
fov = chrono.CH_PI_3 # [rad]

# Lag (in seconds) between sensing and when data becomes accessible
lag = 0.05

# Exposure (in seconds) of each image for generating motion blur effect
exposure_time = 0.02

alias_factor = 2

use_diffuse_1 = False   # whether Camera 1 consinders diffuse reflection
use_diffuse_2 = False   # whether Camera 2 consinders diffuse reflection
use_denoiser_1 = False  # whether Camera 1 uses the OptiX denoiser
use_denoiser_2 = False  # whether Camera 2 uses the OptiX denoiser

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------

# Simulation step size
step_size = 1e-2 # [sec]

# Simulation end time
end_time = 200.0 # [sec]

# Save camera images
save = False

# Render camera images
vis = True

# Verbose terminal output
verbose = False

# Output directory
out_dir = "SENSOR_OUTPUT/CAM_DEMO/"

def main():
    print(f"Copyright (c) 2026 projectchrono.org")

    # Address arguments
    parser = argparse.ArgumentParser(description="Chrono demonstration of a camera sensor with different types of lights.")
    parser.add_argument('spp', type=float, help="Number of samples per pixel")
    parser.add_argument('light_type', type=str, choices=['point', 'spot', 'directional', 'environment'], help="Light type to illuminate the scene")
    parser.add_argument('--use_diffuse_1', action='store_true', help="Path Camera considers diffuse reflection")
    parser.add_argument('--use_diffuse_2', action='store_true', help="Legacy Camera considers diffuse reflection")
    parser.add_argument('--use_denoiser_1', action='store_true', help="Path Camera uses OptiX denoiser")
    parser.add_argument('--use_denoiser_2', action='store_true', help="Legacy Camera uses OptiX denoiser")

    args = parser.parse_args()

    alias_factor = int(math.sqrt(args.spp))
    light_type_str = args.light_type
    use_diffuse_1 = args.use_diffuse_1
    use_diffuse_2 = args.use_diffuse_2
    use_denoiser_1 = args.use_denoiser_1
    use_denoiser_2 = args.use_denoiser_2

    if use_diffuse_1: print("\nPath Camera considers diffuse reflection\n")
    if use_diffuse_2: print("\nLegacy Camera considers diffuse reflection\n")
    if use_denoiser_1: print("\nPath Camera uses OptiX denoiser\n")
    if use_denoiser_2: print("\nLegacy Camera uses OptiX denoiser\n")

    # -----------------
    # Create the system
    # -----------------
    sys = chrono.ChSystemNSC()
    sys.SetGravityY()

    # ---------------------------------------
    # add a mesh to be visualized by a camera
    # ---------------------------------------
    mmesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(chrono.GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                   True, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1)) # scale to a different size

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Audi Chassis Mesh")

    # Create a dielectric and diffuse visual material
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(-6, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    if mesh_body.GetVisualShape(0).GetNumMaterials() > 0:
        mesh_body.GetVisualShape(0).GetMaterial(0).SetRoughness(1.0)
        mesh_body.GetVisualShape(0).GetMaterial(0).SetMetallic(0.0)
    mesh_body.SetFixed(True)
    sys.Add(mesh_body)

    # Create a dielectric and diffuse visual material
    vis_mat3 = chrono.ChVisualMaterial()
    vis_mat3.SetAmbientColor(chrono.ChColor(0.0, 0.0, 0.0))
    vis_mat3.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))
    vis_mat3.SetSpecularColor(chrono.ChColor(0.0, 0.0, 0.0))
    vis_mat3.SetOpacity(1.0)
    vis_mat3.SetRoughness(1.0)
    vis_mat3.SetMetallic(0.0)
    vis_mat3.SetUseSpecularWorkflow(True)
    vis_mat3.SetClassID(30000)
    vis_mat3.SetInstanceID(30000)

    floor = chrono.ChBodyEasyBox(20, 20, 0.1, 1000, True, False)
    floor.SetPos(chrono.ChVector3d(0, 0, -0.6))
    floor.SetFixed(True)
    shape = floor.GetVisualModel().GetShapeInstances()[0].shape
    if shape.GetNumMaterials() == 0:
        shape.AddMaterial(vis_mat3)
    else:
        shape.GetMaterials()[0] = vis_mat3
    sys.Add(floor)

    vis_mat = chrono.ChVisualMaterial()
    vis_mat.SetAmbientColor(chrono.ChColor(0.0, 0.0, 0.0))
    vis_mat.SetDiffuseColor(chrono.ChColor(0.0, 1.0, 0.0))
    vis_mat.SetSpecularColor(chrono.ChColor(1.0, 1.0, 1.0))
    vis_mat.SetOpacity(1.0)
    vis_mat.SetUseSpecularWorkflow(True)
    vis_mat.SetRoughness(0.5)
    vis_mat.SetClassID(30000)
    vis_mat.SetInstanceID(50000)

    box_body = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, False)
    box_body.SetPos(chrono.ChVector3d(0, -2, 0))
    box_body.SetFixed(True)
    sys.Add(box_body)
    shape = box_body.GetVisualModel().GetShapeInstances()[0].shape
    if shape.GetNumMaterials() == 0:
        shape.AddMaterial(vis_mat)
    else:
        shape.GetMaterials()[0] = vis_mat

    # Create a metallic and specular visual material
    vis_mat2 = chrono.ChVisualMaterial()
    vis_mat2.SetBSDF(chrono.BSDFType_SPECULAR)
    vis_mat2.SetAmbientColor(chrono.ChColor(0.0, 0.0, 0.0))
    vis_mat2.SetDiffuseColor(chrono.ChColor(1.0, 1.0, 1.0))
    vis_mat2.SetSpecularColor(chrono.ChColor(1.0, 1.0, 1.0))
    vis_mat2.SetOpacity(1.0)
    vis_mat2.SetUseSpecularWorkflow(True)
    vis_mat2.SetRoughness(0.0)
    vis_mat2.SetMetallic(1.0)
    vis_mat2.SetClassID(30000)
    vis_mat2.SetInstanceID(20000)

    sphere_body = chrono.ChBodyEasySphere(0.5, 1000, True, False)
    sphere_body.SetPos(chrono.ChVector3d(0, 0, 0))
    sphere_body.SetFixed(True)
    sys.Add(sphere_body)
    shape = sphere_body.GetVisualModel().GetShapeInstances()[0].shape
    if shape.GetNumMaterials() == 0:
        shape.AddMaterial(vis_mat2)
    else:
        shape.GetMaterials()[0] = vis_mat2

    vis_mat4 = chrono.ChVisualMaterial()
    vis_mat4.SetAmbientColor(chrono.ChColor(0.0, 0.0, 0.0))
    vis_mat4.SetDiffuseColor(chrono.ChColor(0.0, 0.0, 1.0))
    vis_mat4.SetSpecularColor(chrono.ChColor(0.0, 0.0, 0.0))
    vis_mat4.SetOpacity(1.0)
    vis_mat4.SetUseSpecularWorkflow(True)
    vis_mat4.SetRoughness(0.5)
    vis_mat4.SetClassID(30000)
    vis_mat4.SetInstanceID(1000)

    cyl_body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.25, 1, 1000, True, False)
    cyl_body.SetPos(chrono.ChVector3d(0, 2, 0))
    cyl_body.SetFixed(True)
    sys.Add(cyl_body)
    shape = cyl_body.GetVisualModel().GetShapeInstances()[0].shape
    if shape.GetNumMaterials() == 0:
        shape.AddMaterial(vis_mat4)
    else:
        shape.GetMaterials()[0] = vis_mat4

    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(True)
    sys.Add(ground_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(sys)
    manager.SetRayRecursions(6)
    manager.SetVerbose(verbose)
    manager.scene.SetAmbientLight(chrono.ChVector3f(0.0, 0.0, 0.0))
    b = sens.Background()

    intensity = 1.0
    if "point" in light_type_str:
        manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 3), chrono.ChColor(intensity, intensity, intensity), 500.0, True)
    elif "directional" in light_type_str:
        light_elevation = 15.0
        light_azimuth = 45.0
        manager.scene.AddDirectionalLight(chrono.ChColor(intensity, intensity, intensity), light_elevation * chrono.CH_DEG_TO_RAD, light_azimuth * chrono.CH_DEG_TO_RAD)
    elif "spot" in light_type_str:
        intensity = 6.0
        manager.scene.AddSpotLight(chrono.ChVector3f(-5.0, 5.0, 5.0), chrono.ChColor(intensity, intensity, intensity), 8.67, chrono.ChVector3f(1.0, -1.0, -1.0), 60.0 * chrono.CH_DEG_TO_RAD, 90.0 * chrono.CH_DEG_TO_RAD, False)
    elif "environment" in light_type_str:
        b.mode = sens.BackgroundMode_ENVIRONMENT_MAP
        env_light_scale = 0
        
        b.env_tex = chrono.GetChronoDataFile("sensor/textures/quarry_01_4k.hdr")
        # b.env_tex = chrono.GetChronoDataFile("sensor/textures/dreifaltigkeitsberg_2k.hdr")
        # b.env_tex = chrono.GetChronoDataFile("sensor/textures/UVChecker_byValle_4K.png")
        env_light_scale = 1.0

        # b.env_tex = chrono.GetChronoDataFile("sensor/textures/envmap_sun_at_270_045.hdr")
        # env_light_scale = 1000.0

        manager.scene.SetBackground(b)
        manager.Update()
        manager.scene.AddEnvironmentLight(b.env_tex, env_light_scale)
    else:
        print("\nUnsupported type of light in this scene ...\n")
        sys.exit(1)

    # Set up the background
    if "environment" not in light_type_str:
        b.mode = sens.BackgroundMode_SOLID_COLOR
        b.color_zenith = chrono.ChVector3f(0.0, 0.0, 0.0)
        manager.scene.SetBackground(b)

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    offset_pose1 = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
    cam1 = sens.ChCameraSensor(
        ground_body,             # body camera is attached to
        update_rate,             # update rate in Hz
        offset_pose1,            # offset pose
        image_width,             # image width
        image_height,            # image height
        fov,                     # camera's horizontal field of view
        alias_factor,            # super sampling factor
        lens_model,              # lens model type
        use_diffuse_1,           # whether consider diffuse reflection
        use_denoiser_1,          # whether use OptiX denoiser
        sens.Integrator_PATH,    # integrator algorithm for rendering
        2.2                      # gamma correction
    )
    cam1.SetName("Camera Sensor")
    cam1.SetLag(lag)
    cam1.SetCollectionWindow(exposure_time)

    # --------------------------------------------------------------------
    # Create a filter graph for post-processing the images from the camera
    # --------------------------------------------------------------------
    if noise_model == NoiseModel.CONST_NORMAL:
        cam1.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.0004))
    elif noise_model == NoiseModel.PIXEL_DEPENDENT:
        cam1.PushFilter(sens.ChFilterCameraNoisePixDep(0.0004, 0.0004))
    elif noise_model == NoiseModel.NONE:
        # Don't add any noise models
        pass

    # Renders the image at current point in the filter graph
    if vis:
        cam1.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Path Integrator Denoised" if use_denoiser_1 else "Path Integrator"))

    # Provides the host access to this RGBA8 buffer
    cam1.PushFilter(sens.ChFilterRGBA8Access())

    if save:
        # Save the current image to a png file at the specified path
        cam1.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    # Filter the sensor to grayscale
    cam1.PushFilter(sens.ChFilterGrayscale())

    # Render the buffer again to see the new grayscaled image
    if vis:
        cam1.PushFilter(sens.ChFilterVisualize(640, 360, "Final Visualization"))

    # Save the grayscaled image at the specified path
    if save:
        cam1.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    # Resizes the image to the provided width and height
    cam1.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    # Access the grayscaled buffer as R8 pixels
    cam1.PushFilter(sens.ChFilterR8Access())

    # add sensor to the manager
    manager.AddSensor(cam1)

    # -------------------------------------------------------
    # Create a second camera and add it to the sensor manager
    # -------------------------------------------------------
    offset_pose2 = chrono.ChFramed(chrono.ChVector3d(5, 0, 0), chrono.QuatFromAngleAxis(chrono.CH_PI, chrono.ChVector3d(0, 0, 1)))
    cam2 = sens.ChCameraSensor(
        ground_body,             # body camera is attached to
        update_rate,             # update rate in Hz
        offset_pose2,            # offset pose
        image_width,             # image width
        image_height,            # image height
        fov,                     # camera's horizontal field of view
        alias_factor,            # super sampling factor
        lens_model,              # lens model type
        use_diffuse_2,           # whether consider diffuse reflection
        use_denoiser_2,          # whether use OptiX denoiser
        sens.Integrator_LEGACY,  # integrator algorithm for rendering
        2.2                      # gamma correction
    )
    cam2.SetName("Antialiasing Camera Sensor")
    cam2.SetLag(lag)
    cam2.SetCollectionWindow(exposure_time)

    # Render the antialiased image
    if vis:
        cam2.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Legacy Integrator Denoised" if use_denoiser_2 else "Legacy Integrator"))

    # Save the antialiased image
    if save:
        cam2.PushFilter(sens.ChFilterSave(out_dir + "antialiased/"))

    # Provide the host access to the RGBA8 buffer
    cam2.PushFilter(sens.ChFilterRGBA8Access())

    # Add the second camera to the sensor manager
    manager.AddSensor(cam2)

    # -------------------------------------------------------
    # Create a depth camera that shadows camera2
    # -------------------------------------------------------
    depth = sens.ChDepthCamera(
        ground_body, update_rate, offset_pose2, image_width, image_height, fov, 1000, lens_model)
    depth.SetName("Depth Camera")
    depth.SetLag(lag)
    depth.SetCollectionWindow(exposure_time)

    # Render the depth map
    if vis:
        depth.PushFilter(sens.ChFilterVisualize(640, 360, "Depth Camera"))
    
    # Set max depth of the depth camera
    depth.SetMaxDepth(30.0) # meters

    # Note: With Depth camera, an access filter is already added to the filter graph internally. DO NOT add another.

    # Add the depth camera to the sensor manager
    manager.AddSensor(depth)

    # -------------------------------------------------------
    # Create a normal camera that shadows camera2
    # -------------------------------------------------------
    normal = sens.ChNormalCamera(
        ground_body, update_rate, offset_pose2, image_width, image_height, fov, lens_model)
    normal.SetName("Normal Camera")
    normal.SetLag(lag)
    normal.SetCollectionWindow(exposure_time)

    # Render the normal map
    if vis:
        normal.PushFilter(sens.ChFilterVisualize(640, 360, "Normal Camera"))

    # Save the normal map
    if save:
        normal.PushFilter(sens.ChFilterSave(out_dir + "normal/"))

    # Note: Within Normal Camera, an access filter is already added to the filter graph internally. DO NOT add another.

    # Add the depth camera to the sensor manager
    manager.AddSensor(normal)

    # -------------------------------------------------------
    # Create a semantic segmentation camera that shadows camera2
    # -------------------------------------------------------
    seg = sens.ChSegmentationCamera(
        ground_body, update_rate, offset_pose2, image_width, image_height, fov, lens_model)
    seg.SetName("Semantic Segmentation Camera")
    seg.SetLag(lag)
    seg.SetCollectionWindow(exposure_time)

    # Render the semantic mask
    if vis:
        seg.PushFilter(sens.ChFilterVisualize(640, 360, "Semantic Segmentation"))

    # Save the semantic mask
    if save:
        seg.PushFilter(sens.ChFilterSave(out_dir + "segmentation/"))

    # Provide the host access to the RGBA8 buffer
    seg.PushFilter(sens.ChFilterSemanticAccess())

    # Add the second camera to the sensor manager
    manager.AddSensor(seg)

    manager.Update()

    for v in trimesh_shape.GetMaterials():
        v.SetClassID(200)
        v.SetInstanceID(200)

    # ---------------
    # Simulate system
    # ---------------
    # Demonstration shows cameras panning around a stationary mesh.
    # Each camera begins on opposite sides of the object, but rotate at the same speed
    orbit_radius = 13.0 # [m]
    orbit_rate = 0.1    # [rad/sec]
    orbit_start = 0.0 * chrono.CH_PI / 180.0 # [rad]
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        # Rotate the cameras around the mesh at a fixed rate
        orbit_angle = ch_time * orbit_rate + orbit_start
        pose = chrono.ChFramed(
            chrono.ChVector3d(orbit_radius * math.cos(orbit_angle), orbit_radius * math.sin(orbit_angle), 2),
            chrono.QuatFromAngleAxis(orbit_angle + chrono.CH_PI, chrono.ChVector3d(0, 0, 1)))

        cam1.SetOffsetPose(pose)
        cam2.SetOffsetPose(pose)
        seg.SetOffsetPose(pose)
        depth.SetOffsetPose(pose)
        normal.SetOffsetPose(pose)

        # Access the depth buffer from depth camera
        depth_buffer = depth.GetMostRecentDepthBuffer()
        if verbose and depth_buffer.HasData():
            # Print max depth values
            # float min_depth = depth_ptr->Buffer[0].depth;
            # float max_depth = depth_ptr->Buffer[0].depth;
            # for (int i = 0; i < depth_ptr->Height * depth_ptr->Width; i++) {
            #   max_depth = std::max(max_depth, depth_ptr->Buffer[i].depth);
            # }
            depth_data = depth_buffer.GetDepthData()
            h, w = depth_buffer.Height, depth_buffer.Width
            d = depth_data[h//2, w//2]
            print(f"Depth buffer recieved from depth camera. Camera resolution: {w}x{h}, frame= {depth_buffer.LaunchedCount}, t={depth_buffer.TimeStamp}, depth [{h * w // 2}] ={d}m")

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        sys.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = sys.GetChTime()

    t2 = time.time()
    print(f"Simulation time: {ch_time}s, wall time: {t2 - t1}s.")

if __name__ == "__main__":
    main()
