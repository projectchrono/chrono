Overview of Chrono::Sensor {#sensor_overview}
=================================

\tableofcontents

The Chrono::Sensor module provides support for simulating RGB cameras, lidar, GPS, and IMU within a Chrono simulation. Sensors are objects that are attached to Chrono Bodies(ChBody). Chrono:::Sensor is currently compatible with the core rigid body simulation in Chrono including Chrono::Vehicle.

## Detailed overview of Chrono::Sensor.

 ##### How the sensor process is setup (more details in sensor demos)
~~~{.cpp} 
import ..

// Chrono 
ChSystemNSC mphysicalSystem

// ...

// Setup and initialize sensors and sensor system (manager and environment) 
auto manager = chrono_types::make_shared<ChSensor>
manager->scene->AddPointLight({x,y,z}, {intensity, intensity, intensity}, distance)

// Simulation loop 
while(){ 

  // update the sensor manager
  manager->Update();

  // perform step of dynamics
  mphysicalSystem.DoStepDynamics(step_size);
}
~~~
<br>

 ##### Dynamic chrono simulation has a higher update frequency than sensor simulation (dynamics: order 1kHz; sensors: 10-100Hz), the sensor framework uses a separate thread to manage the data curation. 

  - <img src="http://www.projectchrono.org/assets/manual/sensor/processing.png" width="600" />

<br>

 ##### Chrono::Sensor can leverage multiple render threads each managing a separate GPU for simulating a subset of sensors. This is particularly useful for scenarios with with multiple agents and numerous sensors that operate at various update frequencies. 
  - <img src = "http://www.projectchrono.org/assets/manual/sensor/multigpu.png" width ="600"/>

<br>

 ##### Sensors have a filter graph by which users can add processing modules
   - <img src="http://www.projectchrono.org/assets/manual/sensor/filter_graph_general.png" width="300" />

<br>

 ##### How key frames are used
  - TODO

<br>

##### Load sensor from JSON Files
~~~{.cpp}
auto cam = Sensor::CreateFromJSON(
  GetChronoDataFile("sensor/json/generic/Camera.json"),    // path to json file
  my_body,    // body to which the sensor is attached
  ChFrame<>({-5, 0, 0}, Q_from_AngZ(0)));    // attachment pose for the sensor
  
  // add camera to the manager
  manager->AddSensor(cam);
~~~

## Reference Frames and Relative Attachment Positions

Each Chrono sensor defaults to Z-up, X-forwad, and Y-left to match a vehicle ISO reference frame. For an RGB camera, this means that the z-axis points vertically in the image plane, the y-axis points left in the image plane, and the x-axis points into the image plane. For lidar, the x-axis point along rays with zero vertical angle and zero horizontal angle
