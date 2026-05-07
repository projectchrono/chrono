Overview of Chrono::Sensor {#sensor_overview}
=================================

\tableofcontents

The Chrono::Sensor module provides support for simulating RGB cameras, lidar, radar, GPS, and accelerometer, gyroscope, and magnetometer within a Chrono simulation. Sensors are objects that are attached to Chrono Bodies(ChBody). Chrono::Sensor is currently compatible with the core rigid body simulation in Chrono including Chrono::Vehicle. It's also compatible with the SCMTerrain, but it's not yet compatible with FSI problems. A full list of supported capabilities are listed.

## Supported Sensors
 - RGB monocular camera
 - Physics-based RGB monocular camera
 - Depth camera
 - Segmentation camera
 - Normal-map camera
 - Lidar
 - GPS
 - IMU

## Current Capabilities
 - Scene Rendering
	 - Lights
		 - Point light
		 - Spot light
		 - Directional light
		 - Rectangle area light
		 - Disk area light
		 - Environment light
		 - Shadows
	 - Materials
		 - Reflection based on material reflectance
		 - Fresnel effect
		 - Mesh support based on Wavefront OBJ+MTL format
		 - Programmatic material creation
		 - Legacy integrator supports partial transparency without refractance
	 - Objects
		 - Box primitives
		 - Sphere primitives
		 - Cylinder primitives
		 - Triangle Mesh
 - Camera sensor
	 - Ground-truth ray-traced camera rendering
	 - Filter-based sensor model for user defined sensor model
 - Filters
  	 - Greyscale kernel
  	 - Visualization using GLFW
  	 - Copy-back filter for data access from CPU
  	 - Save images to file at a specific path
  	 - Convert lidar measurements to point cloud
 - Lidar Sensor
     - Single ray and multi-ray data generation
 - GPS Sensor
 - IMU Sensor
     - Accelerometer and Gyroscope

## Detailed overview of Chrono::Sensor.

 ##### How the sensor system is setup (more examples can be found in the sensor demos)
~~~{.cpp}
import ..

// Chrono
ChSystemNSC mphysicalSystem

// ...

// Setup and initialize sensors and sensor system (manager and environment)
auto manager = chrono_types::make_shared<ChSensor>();

// Setup and customize the scene
manager->scene->AddPointLight({x,y,z}, {intensity, intensity, intensity}, distance);
manager->scene->SetAmbientLight({0.1, 0.1, 0.1});

// Set sky gradient
Background b;
b.mode = BackgroundMode::GRADIENT;
b.color_horizon = {.6, .7, .8};
b.color_zenith = {.4, .5, .6};
manager->scene->SetBackground(b);

// Add some sensors
// see sensor specific pages for adding sensors to manager

// Simulation loop
while(){

  // update the sensor manager
  manager->Update();

  // perform step of dynamics
  mphysicalSystem.DoStepDynamics(step_size);
}
~~~
<br>

##### Chrono::Sensor design considerations

Since dynamic chrono simulations typically have a higher update frequency than sensors (dynamics: order 1kHz; sensors: 10-100Hz), the sensor framework uses a separate thread to manage the data curation.

  - <img src="http://www.projectchrono.org/assets/manual/sensor/processing.png" width="600" />

<br>

Chrono::Sensor can leverage multiple render threads each managing a separate GPU for simulating a group of sensors. This is particularly useful for scenarios with multiple agents and numerous sensors that operate at various update frequencies.
  - <img src = "http://www.projectchrono.org/assets/manual/sensor/multigpu.png" width ="600"/>

<br>

Each sensor has a filter graph which users can extend to customize the computation pipeline for modeling specific sensor attributes or configuring specific data formats.
   - <img src="http://www.projectchrono.org/assets/manual/sensor/filter_graph_general.png" width="300" />

<br>


##### Loading sensor models from JSON Files
~~~{.cpp}
auto cam = Sensor::CreateFromJSON(
  GetChronoDataFile("sensor/json/generic/Camera.json"),   // path to json file
  my_body,                                                // body to which the sensor is attached
  ChFramed(ChVector3d(-5, 0, 0), QUNIT));                 // attachment pose for the sensor

  // add camera to the manager
  manager->AddSensor(cam);
~~~

## Reference Frames and Relative Attachment Positions

Each Chrono sensor defaults to Z-up, X-forwad, and Y-left to match a vehicle ISO reference frame. For an RGB camera, this means that the z-axis points vertically in the image plane, the y-axis points left in the image plane, and the x-axis points into the image plane. For lidar, the x-axis point along rays with zero vertical angle and zero horizontal angle
