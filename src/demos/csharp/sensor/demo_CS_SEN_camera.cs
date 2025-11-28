// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono demonstration of a camera sensor.
// Generates a mesh object and rotates camera sensor around the mesh.
//
// =============================================================================

using System;
using static ChronoGlobals;
using static chrono_sensor;


namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Copyright (c) 2025 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);


            // Set the path to the Chrono data files and Chrono::Vehicle data files
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            // -----------------------------------------------------------------------------
            // Camera parameters
            // -----------------------------------------------------------------------------

            // Camera lens model
            // Either PINHOLE or FOV_LENS
            CameraLensModelType lens_model = CameraLensModelType.PINHOLE;

            // Update rate in Hz
            float update_rate = 30;

            // Image width and height
            uint image_width = 1280;
            uint image_height = 720;

            // Camera's horizontal field of view
            float fov = (float)chrono.CH_PI_3;

            // Lag (in seconds) between sensing and when data becomes accessible
            float lag = 0.05F;

            // Exposure (in seconds) of each image
            float exposure_time = 0.02F;

            uint alias_factor = 2;

            bool use_gi = true;  // whether cameras should use global illumination


            // -----------------------------------------------------------------------------
            // Simulation parameters
            // -----------------------------------------------------------------------------

            // Simulation step size
            double step_size = 1e-2;

            // Simulation end time
            float end_time = 2000F;

            // Render camera images
            bool vis = true;

            // Verbose terminal output
            bool verbose = false;

            // ------------------------------------------------------------------------------
            // Create the system
            // ------------------------------------------------------------------------------

            ChSystemNSC sys = new ChSystemNSC();

            // add a mesh to be visualized by a camera
            ChTriangleMeshConnected mmesh = ChTriangleMeshConnected.CreateFromWavefrontFile(chrono.GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                  false, true);
            mmesh.Transform(new ChVector3d(0, 0, 0), new ChMatrix33d(1));  // scale to a different size

            ChVisualShapeTriangleMesh trimesh_shape = new ChVisualShapeTriangleMesh();
            trimesh_shape.SetMesh(mmesh);
            trimesh_shape.SetName("Audi Chassis Mesh");
            trimesh_shape.SetMutable(false);

            ChBody mesh_body = new ChBody();
            mesh_body.SetPos(new ChVector3d(-6, 0, 0));
            mesh_body.AddVisualShape(trimesh_shape, new ChFramed());
            mesh_body.SetFixed(true);
            sys.Add(mesh_body);

            ChVisualMaterial vis_mat3 = new ChVisualMaterial();
            vis_mat3.SetAmbientColor(new ChColor(0F, 0F, 0F));
            vis_mat3.SetDiffuseColor(new ChColor(0.5F, 0.5F, 0.5F));
            vis_mat3.SetSpecularColor(new ChColor(0F, 0F, 0F));
            vis_mat3.SetUseSpecularWorkflow(true);
            vis_mat3.SetClassID(30000);
            vis_mat3.SetInstanceID(30000);


            ChBodyEasyBox floor = new ChBodyEasyBox(20, 20, 0.1, 1000, true, false);
            floor.SetPos(new ChVector3d(0, 0, -1));
            floor.SetFixed(true);
            sys.Add(floor);
            {
                ChVisualShape shape = floor.GetVisualModel().GetShapeInstances()[0].shape;
                if (shape.GetNumMaterials() == 0)
                {
                    shape.AddMaterial(vis_mat3);
                }
                else
                {
                    shape.GetMaterials()[0] = vis_mat3;
                }
            }

            ChVisualMaterial vis_mat = new ChVisualMaterial();
            vis_mat.SetAmbientColor(new ChColor(0F, 0F, 0F));
            vis_mat.SetDiffuseColor(new ChColor(0F, 1F, 0F));
            vis_mat.SetSpecularColor(new ChColor(1F, 1F, 1F));
            vis_mat.SetUseSpecularWorkflow(true);
            vis_mat.SetRoughness(0.5F);
            vis_mat.SetClassID(30000);
            vis_mat.SetInstanceID(50000);

            ChBodyEasyBox box_body = new ChBodyEasyBox(1.0, 1.0, 1.0, 1000, true, false);
            box_body.SetPos(new ChVector3d(0, -2, 0));
            box_body.SetFixed(true);
            sys.Add(box_body);
            {
                ChVisualShape shape = box_body.GetVisualModel().GetShapeInstances()[0].shape;
                if (shape.GetNumMaterials() == 0)
                {
                    shape.AddMaterial(vis_mat);
                }
                else
                {
                    shape.GetMaterials()[0] = vis_mat;
                }
            }

            ChVisualMaterial vis_mat2 = new ChVisualMaterial();
            vis_mat2.SetAmbientColor(new ChColor(0F, 0F, 0F));
            vis_mat2.SetDiffuseColor(new ChColor(1F, 0F, 0F));
            vis_mat2.SetSpecularColor(new ChColor(0F, 0F, 0F));
            vis_mat2.SetUseSpecularWorkflow(true);
            vis_mat2.SetRoughness(0.5f);
            vis_mat2.SetClassID(30000);
            vis_mat2.SetInstanceID(20000);

            ChBodyEasySphere sphere_body = new ChBodyEasySphere(0.5, 1000, true, false);
            sphere_body.SetPos(new ChVector3d(0, 0, 0));
            sphere_body.SetFixed(true);
            sys.Add(sphere_body);
            {
                ChVisualShape shape = sphere_body.GetVisualModel().GetShapeInstances()[0].shape;
                if (shape.GetNumMaterials() == 0)
                {
                    shape.AddMaterial(vis_mat2);
                }
                else
                {
                    shape.GetMaterials()[0] = vis_mat2;
                }
            }

            ChVisualMaterial vis_mat4 = new ChVisualMaterial();
            vis_mat4.SetAmbientColor(new ChColor(0F, 0F, 0F));
            vis_mat4.SetDiffuseColor(new ChColor(0F, 0F, 1F));
            vis_mat4.SetSpecularColor(new ChColor(0F, 0F, 0F));
            vis_mat4.SetUseSpecularWorkflow(true);
            vis_mat4.SetRoughness(0.5f);
            vis_mat4.SetClassID(30000);
            vis_mat4.SetInstanceID(1000);

            ChBodyEasyCylinder cyl_body = new ChBodyEasyCylinder(ChAxis.Y, 0.25, 1, 1000, true, false);
            cyl_body.SetPos(new ChVector3d(0, 2, 0));
            cyl_body.SetFixed(true);
            sys.Add(cyl_body);
            {
                ChVisualShape shape = cyl_body.GetVisualModel().GetShapeInstances()[0].shape;
                if (shape.GetNumMaterials() == 0)
                {
                    shape.AddMaterial(vis_mat4);
                }
                else
                {
                    shape.GetMaterials()[0] = vis_mat4;
                }
            }

            ChBodyEasyBox ground_body = new ChBodyEasyBox(1, 1, 1, 1000, false, false);
            ground_body.SetPos(new ChVector3d(0, 0, 0));
            ground_body.SetFixed(true);
            sys.Add(ground_body);

            // -----------------------
            // Create a sensor manager
            // -----------------------

            ChSensorManager manager = new ChSensorManager(sys);
            manager.SetVerbose(verbose);

            float intensity = 1F;
            manager.scene.AddPointLight(new ChVector3f(100F, 100F, 100F), new ChColor(intensity, intensity, intensity), 500F);
            manager.scene.SetAmbientLight(new ChVector3f(0.1F, 0.1F, 0.1F));

            Background b = new Background();
            b.mode = BackgroundMode.ENVIRONMENT_MAP;
            b.env_tex = chrono.GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
            manager.scene.SetBackground(b);

            // ------------------------------------------------
            // Create a camera and add it to the sensor manager
            // ------------------------------------------------

            ChFramed offset_pose1 = new ChFramed(new ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0.2, new ChVector3d(0, 1, 0)));
            ChCameraSensor cam = new ChCameraSensor(ground_body,   // body camera is attached to
                                                    update_rate,   // update rate in Hz
                                                    offset_pose1,  // offset pose
                                                    image_width,   // image width
                                                    image_height,  // image height
                                                    fov,           // camera's horizontal field of view
                                                    alias_factor,  // super sampling factor
                                                    lens_model,    // lens model type
                                                    use_gi, 2.2F);
            cam.SetName("Camera Sensor");
            cam.SetLag(lag);
            cam.SetCollectionWindow(exposure_time);

            // --------------------------------------------------------------------
            // Create a filter graph for post-processing the images from the camera
            // --------------------------------------------------------------------

            // Renders the image at current point in the filter graph
            if (vis)
                cam.PushFilter(new ChFilterVisualize(640, 360, "Global Illumination"));

            // Provides the host access to this RGBA8 buffer
            cam.PushFilter(new ChFilterRGBA8Access());

            // Filter the sensor to grayscale
            cam.PushFilter(new ChFilterGrayscale());

            // Render the buffer again to see the new grayscaled image
            if (vis)
                cam.PushFilter(new ChFilterVisualize(640, 360, "Final Visualization"));

            // Resizes the image to the provided width and height
            cam.PushFilter(new ChFilterImageResize((int)image_width / 2, (int)image_height / 2));

            // Access the grayscaled buffer as R8 pixels
            cam.PushFilter(new ChFilterR8Access());

            // add sensor to the manager
            manager.AddSensor(cam);

            // -------------------------------------------------------
            // Create a second camera and add it to the sensor manager
            // -------------------------------------------------------

            ChFramed offset_pose2 = new ChFramed(new ChVector3d(5, 0, 0), chrono.QuatFromAngleAxis(chrono.CH_PI, new ChVector3d(0, 0, 1)));
            ChCameraSensor cam2 = new ChCameraSensor(ground_body,   // body camera is attached to
                                                     update_rate,   // update rate in Hz
                                                     offset_pose2,  // offset pose
                                                     image_width,   // image width
                                                     image_height,  // image height
                                                     fov,           // camera's horizontal field of view
                                                     alias_factor,  // supersample factor for antialiasing
                                                     lens_model, false, 2.2F);  // FOV
            cam2.SetName("Antialiasing Camera Sensor");
            cam2.SetLag(lag);
            cam2.SetCollectionWindow(exposure_time);

            // Render the antialiased image
            if (vis)
                cam2.PushFilter(new ChFilterVisualize(640, 360, "Whitted Ray Tracing"));


            // Provide the host access to the RGBA8 buffer
            cam2.PushFilter(new ChFilterRGBA8Access());

            // Add the second camera to the sensor manager
            manager.AddSensor(cam2);

            // -------------------------------------------------------
            // Create a depth camera that shadows camera2
            // -------------------------------------------------------
            ChDepthCamera depth = new ChDepthCamera(ground_body,   // body camera is attached to
                                                    update_rate,   // update rate in Hz
                                                    offset_pose2,  // offset pose
                                                    image_width,   // image width
                                                    image_height,  // image height
                                                    fov,           // camera's horizontal field of view
                                                    1000F,
                                                    lens_model);   // FOV
            depth.SetName("Depth Camera");
            depth.SetLag(lag);
            depth.SetCollectionWindow(exposure_time);

            // Render the semantic mask
            if (vis)
                depth.PushFilter(new ChFilterVisualize(640, 360, "Depth Camera"));

            // Set max depth of the depth camera
            depth.SetMaxDepth(30F); // meters
                                    // Note: With Depth camera, an access filter is already added to the filter graph internally. DO NOT add another.
                                    // Add the depth camera to the sensor manager
            manager.AddSensor(depth);

            // -------------------------------------------------------
            // Create a semantic segmentation camera that shadows camera2
            // -------------------------------------------------------
            ChSegmentationCamera seg = new ChSegmentationCamera(ground_body,   // body camera is attached to
                                                                update_rate,   // update rate in Hz
                                                                offset_pose2,  // offset pose
                                                                image_width,   // image width
                                                                image_height,  // image height
                                                                fov,           // camera's horizontal field of view
                                                                lens_model);   // FOV
            seg.SetName("Semantic Segmentation Camera");
            seg.SetLag(lag);
            seg.SetCollectionWindow(exposure_time);

            // Render the semantic mask
            if (vis)
                seg.PushFilter(new ChFilterVisualize(640, 360, "Semantic Segmentation"));

            // Provide the host access to the RGBA8 buffer
            seg.PushFilter(new ChFilterSemanticAccess());

            // Add the second camera to the sensor manager
            // manager.AddSensor(seg);

            manager.Update();

            ////if (std::shared_ptr < ChVisualShape > visual_asset = std::dynamic_pointer_cast<ChVisualShape>(trimesh_shape))
            ////{
            ////    for (v : visual_asset.GetMaterials()) {
            ////        v.SetClassID(200);
            ////        v.SetInstanceID(200);
            ////    }
            ////}

            // ---------------
            // Simulate system
            // ---------------

            // Demonstration shows cameras panning around a stationary mesh.
            // Each camera begins on opposite sides of the object, but rotate at the same speed
            float orbit_radius = 10F;
            float orbit_rate = 0.1f;
            float ch_time = 0F;

            ChVector3d vecZ = new ChVector3d(0, 0, 1);

            while (ch_time < end_time)
            {
                // Rotate the cameras around the mesh at a fixed rate
                cam.SetOffsetPose(new ChFramed(
                    new ChVector3d(orbit_radius * Math.Cos(ch_time * orbit_rate), orbit_radius * Math.Sin(ch_time * orbit_rate), 2),
                    chrono.QuatFromAngleAxis(ch_time * orbit_rate + chrono.CH_PI, vecZ)));

                //cam2.SetOffsetPose(new ChFramed(
                //    new ChVector3d(orbit_radius * Math.Cos(ch_time * orbit_rate), orbit_radius * Math.Sin(ch_time * orbit_rate), 2),
                //    chrono.QuatFromAngleAxis(ch_time * orbit_rate + chrono.CH_PI, vecZ)));

                //seg.SetOffsetPose(new ChFramed(
                //    new ChVector3d(orbit_radius * Math.Cos(ch_time * orbit_rate), orbit_radius * Math.Sin(ch_time * orbit_rate), 2),
                //    chrono.QuatFromAngleAxis(ch_time * orbit_rate + chrono.CH_PI, vecZ)));

                //depth.SetOffsetPose(new ChFramed(
                //    new ChVector3d(orbit_radius * Math.Cos(ch_time * orbit_rate), orbit_radius * Math.Sin(ch_time * orbit_rate), 2),
                //    chrono.QuatFromAngleAxis(ch_time * orbit_rate + chrono.CH_PI, vecZ)));


                // Update sensor manager
                // Will render/save/filter automatically
                manager.Update();

                // Perform step of dynamics
                sys.DoStepDynamics(step_size);

                // Get the current time of the simulation
                ch_time = (float)sys.GetChTime();
            }

        }
    }
}
