# Install the ROS module {#module_ros_installation}

[TOC]

Chrono::ROS is an optional module that enables direct integration of Chrono with the [Robot Operating System (ROS)](https://ros.org/).

## Features

The **ROS module** allows users to interface a ROS 2-based autonomy stack with autonomous agents in Chrono.

For more detail, read the [Chrono::ROS](@ref manual_ros) section of the reference manual.

## Requirements

- To build and run applications based on this module, the following are required:
  - Linux OS (Windows OS support will be validated soon).
  - Available shared memory (`/dev/shm`) sized for the bridge's channel.
    - The simulation and the ROS subprocess communicate over a shared-memory channel. Its default capacity is 128 MiB for simulation-to-ROS (publishing) traffic plus 32 MiB for ROS-to-simulation (received-command) traffic.
    - Sizing: peak `sim->node` usage is roughly (number of actively-subscribed large topics) × (per-message size). Handlers skip publishing a topic that has no subscriber, and the subprocess drains the channel continuously, so frames rarely accumulate beyond one or two per topic. A 4K RGBA8 image is ~31.6 MiB, so the 128 MiB default holds about four such frames in flight — ample in practice. If `ChROSBridge::GetDroppedOutboundCount()` grows during a run, a subscriber was too slow to keep up and the channel filled momentarily.
    - Override the capacities with `ChROSManager::SetChannelCapacity(sim_to_node_bytes, node_to_sim_bytes)` — both directions are set together, and it must be called **before** `Initialize()`. Raise them for many simultaneous high-bandwidth streams or very large custom topics; lower them on memory-constrained systems. `/dev/shm` must exceed the configured total plus some slack.
  - All current ROS 2 distributions and middleware (rmw) variants are supported; ROS 1 is not.
  - A ROS 2 installation (see [docs.ros.org](https://docs.ros.org/) for instructions). Docker is recommended; a Docker image with Chrono::ROS built is available [here](https://hub.docker.com/r/uwsbel/projectchrono).
    - When using Docker, override the default `shm_size` (often 64 MB) so it exceeds the configured channel total; `shm_size: 256mb` covers the default capacity.
  - NOTE: All ROS 2 packages required by Chrono::ROS are included in the base ROS 2 installation.

## Optional Dependencies

  - Some features (detailed in the [reference manual](@ref manual_ros)) are conditionally built based on some optional dependencies. These dependencies include:
    - [chrono_ros_interfaces](https://github.com/projectchrono/chrono_ros_interfaces)
  - To build URDF support for Chrono::ROS, you will also need to enable the [Chrono::Parsers](@ref module_parsers_installation) module.

<div class="ce-info">
NOTE: If you enable Chrono::Parsers with URDF support and you're using a ROS 2 distribution newer than Iron, ensure the `urdfdom_DIR` is set <i>not</i> to the ROS 2 installation. The `ChParserURDF` class uses a newer version of urdfdom.
</div>

## Building instructions

1. To build Chrono::ROS, after installing the above dependencies, ensure you have sourced your ROS 2 installation (e.g. `source /opt/ros/humble/setup.bash`).
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
3. Set `CH_ENABLE_MODULE_ROS` to 'on' in CMake config arguments or the GUI equivalent.
4. As mentioned above, to enable URDF support, you must also enable the [Chrono::Parsers](@ref module_parsers_installation) module.
5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

## How to use it

- Consult the [reference manual](@ref manual_ros).

- Look at the [API section](@ref ros) of this module for documentation about classes and functions.

- Look at the C++ and Python source of [demos](@ref tutorial_table_of_content_chrono_ros) to learn how to use the functions of this module.
