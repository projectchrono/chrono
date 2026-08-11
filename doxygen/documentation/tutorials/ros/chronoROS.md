Chrono ROS module tutorials {#tutorial_table_of_content_chrono_ros}
===============================

The Chrono distribution contains several demos for integrating autonomy stacks with the Robot Operating System (ROS) using the [ROS module](@ref ros). In addition to the C++ API, Chrono::ROS is also available through PyChrono via the module `pychrono.ros`.

Chrono::ROS exchanges information between Chrono's datatypes and ROS messages. To add custom logic, extend the base `ChROSHandler` with your own publishers and subscribers (see [Custom Handlers](@ref custom_handlers)).

Select ROS C++ demos:

* demo_ROS_custom_handler - writing a custom handler, alongside the built-in clock, body, and TF handlers
* demo_ROS_sensor - ROS integration with Chrono::Sensor (camera, lidar, IMU, GPS)
* demo_ROS_urdf - ROS integration with a URDF model (robot description and transforms)
* demo_ROS_vehicle - a Chrono::Vehicle driven from ROS
* demo_ROS_viper - ROS integration with the Viper rover model
* demo_ROS_two_managers - using multiple `ChROSManager` instances
* demo_ROS_hydraulic_crane - publishing telemetry from a multibody co-simulation

Select ROS Python demos:

* demo_ROS_custom_handler.py - writing a custom handler in Python
* demo_ROS_sensor.py - ROS integration with Chrono::Sensor
* demo_ROS_numpy_camera.py - publishing images from a numpy array via the buffer protocol
* demo_ROS_urdf.py - ROS integration with a URDF model
* demo_ROS_vehicle.py - a Chrono::Vehicle driven from ROS
* demo_ROS_viper.py - ROS integration with the Viper rover model
* demo_ROS_two_managers.py - using multiple `ChROSManager` instances

In addition to the above demos, for a more complex example, please see the [Autonomy Research Testbed](https://github.com/uwsbel/autonomy-research-testbed). In this repository is an effort conducted by the [Simulation Based Engineering Lab](https://sbel.wisc.edu) at the University of Wisconsin-Madison to develop autonomy stacks for a scale vehicle. Chrono (and Chrono::ROS) is used as the development platform for this effort.