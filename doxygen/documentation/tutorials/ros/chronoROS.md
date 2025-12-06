Chrono ROS module tutorials {#tutorial_table_of_content_chrono_ros}
===============================

The Chrono distribution contains several demos for integrating autonomy stacks with the Robot Operating System (ROS) using the [ROS module](@ref ros). In addition to the C++ API, Chrono::ROS is also available through PyChrono via the module `pychrono.ros`.

Chrono::ROS provides direct integration into the ROS ecosystem using builtin ROS entities within rclcpp. Chrono::ROS simply aids in exchanging information between the builtin ROS datatypes and Chrono's datatypes. One can extend the base `ChROSHandler` to develop custom logic with ROS publishers/subscribers/etc.

Select ROS C++ demos:

* demo_ROS_sensor - demo of ROS integration with Chrono::Sensor
* demo_ROS_two_managers - demo of using multiple `ChROSManager` instances
* demo_ROS_urdf - demo of ROS integration with a URDF model
* demo_ROS_vehicle - demo of ROS integration with a Chrono::Vehicle with optional support of sensors
* demo_ROS_viper - demo of ROS integration with a Viper rover model

Select ROS Python demos:

* demo_ROS_sensor.py - demo of ROS integration with Chrono::Sensor
* demo_ROS_two_managers.py - demo of using multiple `ChROSManager` instances
* demo_ROS_urdf.py - demo of ROS integration with a URDF model
* demo_ROS_vehicle.py - demo of ROS integration with a Chrono::Vehicle with optional support of sensors
* demo_ROS_viper.py - demo of ROS integration with a Viper rover model

In addition to the above demos, for a more complex example, please see the [Autonomy Research Testbed](https://github.com/uwsbel/autonomy-research-testbed). In this repository is an effort conducted by the [Simulation Based Engineering Lab](https://sbel.wisc.edu) at the University of Wisconsin-Madison to develop autonomy stacks for a scale vehicle. Chrono (and Chrono::ROS) is used as the development platform for this effort.