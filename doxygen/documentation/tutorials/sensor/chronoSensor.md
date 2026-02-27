Chrono SENSOR module tutorials {#tutorial_table_of_content_chrono_sensor}
===============================

The Chrono distribution contains several demos for modeling and simulating sensors for robots and autonomous vehicles with the [SENSOR module](@ref sensor).

Chrono::Sensor provides support for:
 - RGB mono Camera 
 - Lidar
 - Radar
 - GPS
 - IMU (accelerometer, gyroscope, magnetometer)
 - Tachometer  

These are parameterized models of sensors commonly used in robotic and autonomous vehicle systems.

In addition to the main library, the SENSOR module also creates a library of pre-defined [SENSOR models](@ref sensor_overview) which currently contains:

- Generic Camera
- Lidar
  - Generic lidar
  - Velodyne VLP-16 (Puck)
  - Velodyne HDL-32E
- Generic GPS
- Generic IMUs

Select sensor demos:

* Basic Sensor Demos
  * demo_SEN_camera - example camera sensors with custom filters
  * demo_SEN_lidar - example lidar sensor with custom filters
  * demo_SEN_GPSIMU - example using a GPS and IMU on a pendulum
  * demo_SEN_tachometer - example use of a tachometer to measure body's rotational speed
  * demo_SEN_JSON - example use of sensors through JSON interface
  * demo_SEN_demo_SEN_phys_cam - example of a camera sensor
  * demo_SEN_cornell_box - demonstration of a camera sensor with different types of lights inside a Cornell box for benchmarking and observing color bleeding

* Vehicle and Sensors (require Chrono::Vehicle and Chrono::Irrlicht)
  * demo_SEN_Gator - example vehicle equipped with sensors
  * demo_SEN_HMMWV - example HMMWV equipped with sensors
  * demo_SEN_deformableSoil - example sensing on deformable terrain
