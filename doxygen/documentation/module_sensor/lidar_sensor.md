Lidar Sensor Model {#lidar_sensor}
=================================

\tableofcontents

Details of the Lidar sensor implemented in Chrono::Sensor.

In Chrono:Sensor:ChCameraSensor, the synthetic data is generated via GPU-based ray-tracing. By leveraging hardware acclereated support and the headless rendering capablities provided by Nvidia Optix Library.

#### Creating a Lidar
~~~(.cpp)
autolidar = chrono_types::make_shared<ChLidarSensor>(my_body,        // body lidar is attached to
update_rate,     // scanning rate in Hz
offset_pose,     // offset pose
horizontal_samples,// number of horizontal samples
vertical_channels,// number of vertical channels
horizontal_fov,// horizontal field of view
max_vert_angle,// high vertical extent
min_vert_angle, // low vertical extent
100 );  // maximum range

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

// Addsensorto manager
manager->AddSensor(cam);
~~~

<br>

#### Lidar Data Access
~~~{.cpp}
UserXYZIBufferPtr xyzi_ptr;
while(){
    xyzi_ptr=lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
    if(xyzi_ptr->Buffer) {
        // Retrieve and print the first point in the point cloud
        PixelXYZIfirst_point= xyzi_ptr->Buffer[0];
        std::cout<<"First Point: [ "<<unsigned(first_point.x) <<", "<<
        unsigned(first_point.y) <<", “ <<unsigned(first_point.z) <<", "<<
        unsigned(first_point.intensity) <<" ]"<<std::endl;
    }
}
~~~