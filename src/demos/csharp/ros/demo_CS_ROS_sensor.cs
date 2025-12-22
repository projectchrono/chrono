// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bocheng Zou, Aaron Young (C++ original)
// =============================================================================
//
// Demo to show the use of Chrono::Sensor with ROS
//
// =============================================================================

using System;
using static ChronoGlobals;
using static chrono;
using static chrono_sensor;

namespace ChronoDemo
{
    internal class Program
    {
        static void Main(string[] args)
        {
            chrono.SetChronoDataPath(CHRONO_DATA_DIR);

            Console.WriteLine("Copyright (c) 2025 projectchrono.org");
            Console.WriteLine("Chrono version: " + CHRONO_VERSION);
            Console.WriteLine();

            // Create the system
            ChSystemNSC sys = new ChSystemNSC();

            // Add a mesh object to make the scene interesting
            var mmesh = ChTriangleMeshConnected.CreateFromWavefrontFile(
                chrono.GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                false,
                true
            );

            mmesh.Transform(new ChVector3d(0, 0, 0), new ChMatrix33d(1));

            var trimesh_shape = new ChVisualShapeTriangleMesh();
            trimesh_shape.SetMesh(mmesh);
            trimesh_shape.SetName("Audi Chassis Mesh");
            trimesh_shape.SetMutable(false);

            var mesh_body = new ChBody();
            mesh_body.SetPos(new ChVector3d(0, 0, 0));
            mesh_body.AddVisualShape(trimesh_shape, new ChFramed(new ChVector3d(0, 0, 0)));
            mesh_body.SetFixed(true);
            sys.Add(mesh_body);

            // This is the body we'll attach the sensors to
            var ground_body = new ChBodyEasyBox(1, 1, 1, 1000, false, false);
            ground_body.SetPos(new ChVector3d(0, 0, 0));
            ground_body.SetFixed(false);
            ground_body.SetMass(0);
            sys.Add(ground_body);

            // -----------------------
            var noise_none = new ChNoiseNone();
            ChFramed offset_pose = new ChFramed(
                new ChVector3d(-8, 0, 2),
                chrono.QuatFromAngleAxis(0.2, new ChVector3d(0, 1, 0))
            );

            // Create the sensor system
            ChSensorManager sensor_manager = new ChSensorManager(sys);
            sensor_manager.scene.AddPointLight(
                new ChVector3f(100, 100, 100),
                new ChColor(2, 2, 2),
                500.0f
            );
            sensor_manager.scene.SetAmbientLight(new ChVector3f(0.1f, 0.1f, 0.1f));

            // Set the background to an environment map
            Background b = new Background();
            b.mode = BackgroundMode.ENVIRONMENT_MAP;
            b.env_tex = chrono.GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
            sensor_manager.scene.SetBackground(b);

            // Create a camera that's placed on the hood
            ChCameraSensor cam = new ChCameraSensor(
                ground_body,
                30.0f,
                offset_pose,
                1280,
                720,
                (float)(chrono.CH_PI / 3.0)
            );
            cam.PushFilter(new ChFilterRGBA8Access());
            cam.PushFilter(new ChFilterVisualize(1280, 720));
            sensor_manager.AddSensor(cam);

            // Create a lidar and add it to the sensor manager
            ChLidarSensor lidar = new ChLidarSensor(
                ground_body,
                5.0f,
                offset_pose,
                900,
                30,
                (float)(2.0 * chrono.CH_PI),
                (float)(chrono.CH_PI / 12.0),
                (float)(-chrono.CH_PI / 6.0),
                100.0f
            );
            lidar.PushFilter(new ChFilterDIAccess());
            lidar.PushFilter(new ChFilterPCfromDepth());
            lidar.PushFilter(new ChFilterXYZIAccess());
            lidar.PushFilter(new ChFilterVisualizePointCloud(640, 480, 0.50f, "3D Lidar"));
            sensor_manager.AddSensor(lidar);

            // Create a 2d lidar and add it to the sensor manager
            ChLidarSensor lidar_2d = new ChLidarSensor(
                ground_body,
                5.0f,
                offset_pose,
                480,
                1,
                (float)(2.0 * chrono.CH_PI),
                0.0f,
                0.0f,
                100.0f
            );
            lidar_2d.PushFilter(new ChFilterDIAccess());
            lidar_2d.PushFilter(new ChFilterVisualize(640, 480, "2D Lidar"));
            sensor_manager.AddSensor(lidar_2d);

            // add an accelerometer, gyroscope, and magnetometer
            ChVector3d gps_reference = new ChVector3d(-89.400, 43.070, 260.0);

            var acc = new ChAccelerometerSensor(ground_body, 100.0f, offset_pose, noise_none);
            acc.PushFilter(new ChFilterAccelAccess());
            sensor_manager.AddSensor(acc);

            var gyro = new ChGyroscopeSensor(ground_body, 100.0f, offset_pose, noise_none);
            gyro.PushFilter(new ChFilterGyroAccess());
            sensor_manager.AddSensor(gyro);

            var mag = new ChMagnetometerSensor(ground_body, 100.0f, offset_pose, noise_none, gps_reference);
            mag.PushFilter(new ChFilterMagnetAccess());
            sensor_manager.AddSensor(mag);

            // add a GPS sensor
            var gps = new ChGPSSensor(ground_body, 5.0f, offset_pose, gps_reference, noise_none);
            gps.PushFilter(new ChFilterGPSAccess());
            sensor_manager.AddSensor(gps);

            sensor_manager.Update();

            // ------------

            // Create ROS manager
            ChROSManager ros_manager = new ChROSManager();

            // /clock
            var clock_handler = new ChROSClockHandler();
            ros_manager.RegisterHandler(clock_handler);

            // Camera publisher at half the sensor update rate
            float camera_rate = cam.GetUpdateRate() / 2.0f;
            string camera_topic_name = "~/output/camera/data/image";
            var camera_handler = new ChROSCameraHandler(camera_rate, cam, camera_topic_name);
            ros_manager.RegisterHandler(camera_handler);

            // 3D lidar publisher
            string lidar_topic_name = "~/output/lidar/data/pointcloud";
            var lidar_handler = new ChROSLidarHandler(lidar, lidar_topic_name);
            ros_manager.RegisterHandler(lidar_handler);

            // 2D lidar publisher as LaserScan
            string lidar_2d_topic_name = "~/output/lidar_2d/data/laser_scan";
            var lidar_2d_handler = new ChROSLidarHandler(
                lidar_2d,
                lidar_2d_topic_name,
                ChROSLidarHandlerMessageType.LASER_SCAN
            );
            ros_manager.RegisterHandler(lidar_2d_handler);

            // Accelerometer
            float acc_rate = acc.GetUpdateRate();
            string acc_topic_name = "~/output/accelerometer/data";
            var acc_handler = new ChROSAccelerometerHandler(acc_rate, acc, acc_topic_name);
            ros_manager.RegisterHandler(acc_handler);

            // Gyroscope
            string gyro_topic_name = "~/output/gyroscope/data";
            var gyro_handler = new ChROSGyroscopeHandler(gyro, gyro_topic_name);
            ros_manager.RegisterHandler(gyro_handler);

            // Magnetometer
            string mag_topic_name = "~/output/magnetometer/data";
            var mag_handler = new ChROSMagnetometerHandler(mag, mag_topic_name);
            ros_manager.RegisterHandler(mag_handler);

            // IMU handler at half accel rate
            // NOTE: Do NOT set IMU handler update rate higher than any sub-handlers.
            string imu_topic_name = "~/output/imu/data";
            float imu_rate = acc.GetUpdateRate() / 2.0f;
            var imu_handler = new ChROSIMUHandler(imu_rate, imu_topic_name);
            imu_handler.SetAccelerometerHandler(acc_handler);
            imu_handler.SetGyroscopeHandler(gyro_handler);
            imu_handler.SetMagnetometerHandler(mag_handler);
            ros_manager.RegisterHandler(imu_handler);

            // GPS publisher
            string gps_topic_name = "~/output/gps/data";
            var gps_handler = new ChROSGPSHandler(gps, gps_topic_name);
            ros_manager.RegisterHandler(gps_handler);

            // TF handler (one handler, multiple sensors)
            var tf_handler = new ChROSTFHandler(100);
            tf_handler.AddSensor(cam, ground_body.GetName(), "cam");
            tf_handler.AddSensor(lidar, ground_body.GetName(), "lidar");
            tf_handler.AddSensor(lidar_2d, ground_body.GetName(), "lidar_2d");
            tf_handler.AddSensor(acc, ground_body.GetName(), "acc");
            tf_handler.AddSensor(gyro, ground_body.GetName(), "gyro");
            tf_handler.AddSensor(mag, ground_body.GetName(), "mag");
            tf_handler.AddSensor(gps, ground_body.GetName(), "gps");

            // Important: register TF handler too
            ros_manager.RegisterHandler(tf_handler);

            // Initialize ROS manager
            ros_manager.Initialize();

            // -------------------------
            // Simulation
            // -------------------------
            double time = 0.0;
            double step_size = 2e-3;
            double time_end = 1000.0;

            // Give the ground body some rotational velocity so sensors appear to move.
            // C++: ground_body->SetAngVelParent({0,0,0.1});
            ground_body.SetAngVelParent(new ChVector3d(0, 0, 0.1));

            while (time < time_end)
            {
                time = sys.GetChTime();

                sensor_manager.Update();

                if (!ros_manager.Update(time, step_size))
                    break;

                sys.DoStepDynamics(step_size);
            }
        }
    }
}
