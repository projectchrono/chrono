Chrono SENSOR module tutorials {#tutorial_table_of_content_chrono_sensor}
===============================

The Chrono distribution contains several demos for modeling and simulating sensors for robots and autonomous vehicles with the [SENSOR module](@ref sensor).

Chrono::Sensor provides support for Camera, Lidar, GPS, and IMU. These are parameterized models of sensors commonly used in robotic and autonomous vehicle systems.

In addition to the main library, the SENSOR module also creates a library of pre-defined [SENSOR models](@ref sensor_models) which currently contains:

- Camera Sensor
- Lidar Sensor
  - Velodyne VLP-16 (Puck)
  - Velodyne HDL-32E
- GPS Sensor
- IMU Sensor

Selected sensor demos:

* Basic Sensor Demos
  * demo_SEN_Camera - example camera sensors with custom filters
  * demo_SEN_Lidar - example lidar sensor with custom filters
  * demo_SEN_GPSIMU - example using a GPS and IMU on a pendulum
  * demo_SEN_JSON - example use of sensors through JSON interface

* Vehicle and Sensors (require Chrono::Vehicle and Chrono::Irrlicht)
  * demo_SEN_Gator - example vehicle equipped with sensors
  * demo_SEN_HMMWV - example HMMWV equipped with sensors
  * demo_SEN_rccar - another example vehicle with sensors
  * demo_SEN_deformableSoil - example sensing on deformable terrain

* Sensors with TensorRT (requies USE_TENSOR_RT=ON in cmake)
  * demo_SEN_NNCamera - example for using a camera with a filter based on a neural net with inference through TensorRT
