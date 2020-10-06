Overview of Chrono::Sensor {#sensor_overview}
=================================

\tableofcontents


The Chrono::Sensor module provides support for simulating RGB cameras, lidar, GPS, and IMU within a Chrono simulation. Chrono:::Sensor is currently compatible with the core rigid body simulation in Chrono including Chrono::Vehicle.



Detailed overview of Chrono::Sensor.

Important Overview Points
 - How the sensor process is setup
 - How rendering steps are performed
 - How the filter graph works
 - How key frames are used

##Simulation Setup
Important Setup Points
  - order of how a user should setup a simulation
  - how a user should make calls for creating sensors
  - how parameter setting works and how the environment is setup

##Simulation Loop


Important Sim Loop Points
 - How the user should make calls in the sim loop
 - Where/how updating the scene should be taken place

##Reference Frames and Relative Attachment Positions

Important Reference Frame Points
 - The reference frames for the sensors
 - Mounting pose relative to the parent object

Each Chrono sensor defaults to Z-up, X-forwad, and Y-left to match a vehicle ISO reference frame. For an RGB camera, this means that the z-axis points vertically in the image plane, the y-axis points left in the image plane, and the x-axis points into the image plane. For lidar, the x-axis point along rays with zero vertical angle and zero horizontal angle
