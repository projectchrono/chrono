# Install the ROS module {#module_ros_installation}

[TOC]

This is an optional module that enables direct integration of Chrono with the Robot Operating System (ROS).

Read [the introduction to modules](modularity.html) for a technical
background on the modularity of the Chrono project.

## Features

The **ROS module** allows users to interface a ROS autonomy stack with autonomous agents in Chrono.

For more detail, read the [Chrono::ROS](@ref manual_ros) section of the reference manual.

## Required Dependencies

- To build and run applications based on this module, the following are required:
  - ROS Humble (see [docs.ros.org](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions). Docker is recommended.
  - Have [chrono_ros_interfaces](https://github.com/AaronYoung5/chrono_ros_interfaces) locally.

## Building instructions

1. To build Chrono::ROS, you must first build [chrono_ros_interfaces](https://github.com/AaronYoung5/chrono_ros_interfaces) and have sourced the setup file. To build a ROS workspace, please [see the official ROS documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:

3. Set the `ENABLE_MODULE_ROS` as 'on', then press 'Configure' (to refresh the variable list)

4. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

## How to use it

- Consult the [reference manual](@ref manual_ros).

- Look at the [API section](@ref ros) of this module for documentation about classes and functions.

- Look at the C++ and Python source of [demos](@ref tutorial_table_of_content_chrono_ros) to learn how to use the functions of this module.
