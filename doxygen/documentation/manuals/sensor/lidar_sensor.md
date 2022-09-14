Lidar Sensor Model {#lidar_sensor}
=================================

\tableofcontents

In chrono:sensor:ChLidarSensor, the synthetic data is generated via GPU-based ray-tracing. By leveraging hardware accelerated support and the headless rendering capabilities provided by NVIDIA Optix Library. For each lidar beam, a group of rays are traced that sample that lidar beam. The number of samples, along with beam divergence angle, are set by the user. The entire frame/scan of the lidar is processed in a single render step. To account for the time difference of rays across the scan, keyframes and motion blur techniques are used. With these keyframes, each beam in the scan traces the scene at a specific time, reproducing the motion of objects and the lidar. The intensity returned by the lidar beams is based on diffuse reflectance.

#### Creating a Lidar
~~~{.cpp}
auto lidar = chrono_types::make_shared<ChLidarSensor>(
	               parent_body,             // body lidar is attached to
                   update_rate,             // scanning rate in Hz
                   offset_pose,             // offset pose
                   horizontal_samples,      // number of horizontal samples
                   vertical_channels,       // number of vertical channels
                   horizontal_fov,          // horizontal field of view
                   max_vert_angle,          // high vertical extent
                   min_vert_angle,          // low vertical extent
                   max_distance,            // maximum range
                   beam_shape,              // set the beam shape to rectangular or elliptical
                   sample_radius,           // configures the number of samples to use per beam
                   vert_divergence_angle,   // vertical beam divergence angle
                   hori_divergence_angle,   // horizontal beam divergence angle
                   return_mode,             // return mode for the lidar when multiple samples used
                   clip_near                // near clipping distance to ensure housing geometry not seen
                   );               
lidar->SetName("Lidar Sensor");
lidar->SetLag(lag);
lidar->SetCollectionWindow(collection_time); // typically time to spin 360 degrees
~~~

<br>

#### Lidar Filter Graph
~~~{.cpp}
// Access lidar data in raw format (range and intensity)
lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());

// Generate point cloud from raw data
lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());

// Add noise based on angle, angle, range, intensity
lidar->PushFilter(chrono_types::make_shared<ChFilterLidarNoiseXYZI>(0.01f, 0.001f, 0.001f, 0.01f));

// Access lidar data in point cloud format
lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());

// visualize point cloud (<height, width, zoom, name> of visual window)
lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 2, "Lidar Point Cloud"));

// Add sensor to manager
manager->AddSensor(lidar);
~~~

<br>

#### Lidar Data Access
~~~{.cpp}
UserXYZIBufferPtr xyzi_ptr;
while () {
    xyzi_ptr=lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
    if(xyzi_ptr->Buffer) {
        // Retrieve and print the first point in the point cloud
        PixelXYZI first_point= xyzi_ptr->Buffer[0];
        std::cout<<"First Point: [ "<<unsigned(first_point.x) <<", "<<
        unsigned(first_point.y) <<", “ <<unsigned(first_point.z) <<", "<<
        unsigned(first_point.intensity) <<" ]"<<std::endl;
    }
}
~~~
