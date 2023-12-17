# Install the ROS module {#module_ros_installation}

[TOC]

This is an optional module that enables direct integration of Chrono with the Robot Operating System (ROS).

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.

## Features

The **ROS module** allows users to interface a ROS autonomy stack with autonomous agents in Chrono.

For more detail, read the [Chrono::ROS](@ref manual_ros) section of the reference manual.

## Required Dependencies

- To build and run applications based on this module, the following are required:
  - ROS 2 Humble (see [docs.ros.org](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions). Docker is recommended. A Chrono::ROS Docker image is available [here](https://hub.docker.com/r/uwsbel/projectchrono).
    - The following ROS 2 packages are required to build Chrono::ROS. All come with the base ROS 2 installation.
      - `rclcpp`
      - `rcl_interfaces`
      - `std_msgs`
      - `geometry_msgs`
      - `sensor_msgs`
      - `tf2_ros`
      - `tf2_msgs`

## Optional Dependencies

  - Some features (detailed in the [reference manual](@ref manual_ros)) are conditionally built based on some optional dependencies. These dependencies include:
    - [chrono_ros_interfaces](https://github.com/projectchrono/chrono_ros_interfaces)
  - To build URDF support for Chrono::ROS, you will also need to enable the [Chrono::Parsers](@ref module_parsers_installation) module.

<div class="ce-info">
NOTE: If you enable Chrono::Parsers with URDF support and you're using a ROS 2 distribution new than Iron, ensure the `urdfdom_DIR` is set <i>not</i> to the ROS 2 installation. The `ChParserURDF` class uses a newer version of urdfdom.
</div>

## Building instructions

1. To build Chrono::ROS, after installing the above dependencies, ensure you have sourced your ROS 2 installation (e.g. `source /opt/ros/humble/setup.bash`).
2. Please repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:
  - Set the `ENABLE_MODULE_ROS` as 'on', then press 'Configure' (to refresh the variable list)
  - Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.
3. As mentioned above, to enable URDF support, you must also enable the [Chrono::Parsers](@ref module_parsers_installation) module.

## How to use it

- Consult the [reference manual](@ref manual_ros).

- Look at the [API section](@ref ros) of this module for documentation about classes and functions.

- Look at the C++ and Python source of [demos](@ref tutorial_table_of_content_chrono_ros) to learn how to use the functions of this module.
