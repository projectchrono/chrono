Chrono::ROS {#module_ros_overview}
==============

## Project Overview

Chrono::ROS is a module developed by the Simulation-Based Engineering Lab ([SBEL](https://sbel.wisc.edu)) at the University of Wisconsin-Madison. The goal of the project is to provide a simple and extensible interface between Project Chrono and the Robot Operating System ([ROS](docs.ros.org)). Chrono::ROS allows individuals to directly incorporate Chrono into the development of ROS autonomy stacks. Built on top of ROS 2's existing network capabilities, Chrono::ROS uses existing ROS concepts (publishers, subscribers, etc.) and interacts directly with ROS topics.

## Support for Other Chrono Modules {#syn_modules}

Chrono::ROS is not necessarily tied to any one Chrono module; however, currently, only limited handlers have been implemented for existing Chrono functionality. For instance, [Chrono::Vehicle](@ref manual_vehicle) and [Chrono::Sensor](@ref manual_sensor) have general purpose handlers implemented for rapid prototyping.

If other modules are used, custom handlers can be implemented that handle this functionality. 