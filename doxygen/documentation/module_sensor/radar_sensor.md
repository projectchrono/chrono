Radasr Sensor Model {#radar_sensor}
=================================

\tableofcontents

In Chrono:Sensor:ChRadarSensor, the synthetic data is generated via GPU-based ray-tracing. By leveraging hardware acclereated support and the headless rendering capablities provided by Nvidia Optix Library. Rays are For each lidar beam, a group of rays are traced that sample that lidar beam. The number of samples, along with beam divergence angle, are set by the user. The entire frame/scan of the lidar is processed in a single render step. To account for the time difference of rays across the scan, keyframes and motion blur techniques are used. With these keyframes, each beam in the scan traces the scene at a specific time, reproducing the motion of objects and the lidar. The intensity returned by the lidar beams is based on diffuse reflectance.

#### Creating a Lidar
~~~{.cpp}
auto radar = chrono_types::make_shared<ChRadarSensor>(
	               my_body,             // body lidar is attached to
                   update_rate,         // scanning rate in Hz
                   offset_pose,         // offset pose
                   horizontal_samples,  // number of horizontal rays
                   vertical_samples,   // number of vertical rays
                   horizontal_fov,      // horizontal field of view
                   vertical_fov,      // low vertical extent
                   100);                // maximum range

radar->SetName("Radar Sensor");
radar->SetLag(lag);
radar->SetCollectionWindow(collection_time); // typically time to spin 360 degrees
~~~

<br>

#### Radar Filter Graph
~~~{.cpp}
// Access radar data in raw format (range, azimuth, and elevation)
radar->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());

// Generate point cloud from raw data
radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>());

// Access lidar data in point cloud format
radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZAccess>());

// visualize point cloud (<height, width, zoom, name> of visual window)
radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(640, 480, 2, "Radar Point Cloud"));

// Add sensor to manager
manager->AddSensor(radar);
~~~

<br>

#### Ridar Data Access
~~~{.cpp}
UserRadarXYZBufferPtr data_ptr;
while () {
    data_ptr=Radar->GetMostRecentBuffer<UserRadarXYZBufferPtr>();
    if(data_ptr->Buffer) {
        // Retrieve and print the first point in the point cloud
        RadarXYZReturn first_point= data_ptr->Buffer[0];
        std::cout<<"First Point: [ "<<unsigned(first_point.x) <<", "<<
        unsigned(first_point.y) <<", “ <<unsigned(first_point.z) <<", "<<
        unsigned(first_point.vel_x) <<", "<<unsigned(first_point.vel_y)<<", "
        unsigned(first_point.vel_z) <<", "<<
        unsigned(first_point.amplitude) <<" ]"<<std::endl;
    }
}
~~~
