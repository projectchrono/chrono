Radar Sensor Model {#radar_sensor}
=================================

\tableofcontents

In chrono:sensor:ChRadarSensor, the synthetic data is generated via GPU-based ray-tracing. By leveraging hardware acclereated support and the headless rendering capabilities provided by NVIDIA Optix Library. The field of view and maximum distance of the radar define the space in which objects may be detected. To model this, the space is partitioned and sampled using rays. The rays trace the environment from the sensors out into the environment. If a ray collision is detected, the ray is endowed with the velocity of the detection, the intensity of return, object ID, and distance to detection. The sampled says are then used to approximate the full radar return. Since radar is transmitted as a single, continuous wave, the movement of the scene during a scan is negligible in contrast to lidar. The intensity returned by the radar samples is based on a diffuse reflectance model.

#### Creating a Radar
~~~{.cpp}
auto radar = chrono_types::make_shared<ChRadarSensor>(
	               parent_body,         // body radar is attached to
                   update_rate,         // scanning rate in Hz
                   offset_pose,         // offset pose
                   horizontal_samples,  // number of horizontal rays
                   vertical_samples,    // number of vertical rays
                   horizontal_fov,      // horizontal field of view
                   vertical_fov,        // low vertical extent
                   100);                // maximum range

radar->SetName("Radar Sensor");
radar->SetLag(lag);
~~~

<br>

#### Radar Filter Graph
~~~{.cpp}
// Access radar data in raw format (range, azimuth, and elevation)
radar->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());

// Generate point cloud from raw data
radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>());

// Access radar data in point cloud format
radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZAccess>());

// visualize point cloud (<height, width, zoom, name> of visual window)
radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(640, 480, 2, "Radar Point Cloud"));

// Add sensor to manager
manager->AddSensor(radar);
~~~

<br>

#### Radar Data Access
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
