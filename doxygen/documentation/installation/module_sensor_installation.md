Install the SENSOR module   {#module_sensor_installation}
===============================

[TOC]

Chrono::Sensor is an optional module that enables the modeling and simulation of sensors for the purpose of simulating robotics and autonomous agents within Chrono.


## Features

The **SENSOR module** allows users to model and simulate sensors for robotics and autonomous agents.

For more detail, read the [Chrono::Sensor](@ref manual_sensor) section of the reference manual.


## Required Dependencies

- To **run** applications based on this module, the following are required:
  * NVIDIA GPU, Turing or newer - capable of running OptiX
  * NVIDIA Graphics driver R580 or newer

- To **build** applications based on this module, the following are required:
  * [CUDA](https://developer.nvidia.com/cuda-downloads)
  * [OptiX](https://developer.nvidia.com/designworks/optix/download) - version 9.0 or newer (will **NOT** work with older versions)
  * [GLFW](https://www.glfw.org/) - version 3.0 or newer
  * [GLEW](http://glew.sourceforge.net/) - version 1.0 or newer
  * [OpenGL](https://www.opengl.org/)

<div class="ce-warning">
OptiX support in Chrono::Sensor is optional. If OptiX is not available, sensor models that require ray-tracing (e.g., camera, lidar, radar) will not be included in the Chrono::Sensor library. Availability of OptiX support is indicated via the macro `CHRONO_HAS_OPTIX` in the configuration header `ChConfigSensor.h`.
</div>

## Building instructions

1. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see the CMake window, you must add the following steps:

2. Set the `CH_ENABLE_MODULE_SENSOR` as 'on', then press 'Configure' (to refresh the variable list)

3. Set the `OptiX_INSTALL_DIR` and `OptiX_ROOT_DIR` variables to the root of the OptiX directory installed on the system (directory that includes `include/`) and press 'Configure' to refresh the variable list. If a refresh does not correctly set the following variable: `OptiX_Include` manually set it accordingly (e.g. `OptiX_INSTALL_DIR/include`).

5. Set all the values for `GLEW_...`, `GLFW_...` to proper directory or file values if not automatically found by cmake.

6. Optionally set `CH_USE_SENSOR_NVRTC` to 'on' to enable runtime compilation of the Optix RT Kernels. Press 'Configure' to refresh the variable list. If set to 'off', the RT Kernels will be compiled at runtime. Depending on the system, you may need to set `CMAKE_CUDA_ARCHITECTURES` to the specific target architecture since this will result in RT Kernels being compiled to PTX.

7. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

**NOTE**: if linking to Chrono::Sensor install from an external project, make sure to set the directory of the install location where the    shader code (compiled ptx code or shaders/*.cu files) is located. This should be set at the top of any external code that will use    Chrono::Sensor from an install location.

  ```cpp
  //function to set the shader location (include ChOptixUtils.h)
  chrono::sensor::SetSensorShaderDir("path/to/sensor/shaders");
  
  //if USE_CUDA_NVRTC is enabled, use
  chrono::sensor::SetSensorShaderDir("path/to/install/include/chrono_sensor/optix/shaders/");
  
  //if USE_CUDA_NVRTC is disabled, use
  chrono::sensor::SetSensorShaderDir("path/to/install/lib/sensor_ptx/");
 ```

## How to use it

- Consult the [reference manual](@ref manual_sensor).

- Look at the [API section](@ref sensor) of this module for documentation about classes and functions.

- Look at the C++ and Python source of [demos](@ref tutorial_table_of_content_chrono_sensor) to learn how to use the functions of this module.
