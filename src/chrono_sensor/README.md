#### For information about the pychrono.sensor module including reasons/motives leveraged in the interfacing of sensor+python, see the sensor_readme.md in src/chrono_python/

# Chrono Sensor Module
Current Tested Systems:
 - Arch Linux:
 - Ubuntu 20.04: GCC 9.3, CUDA 10.2
 - Windows 10: VS 2019, CUDA 10.2

## Supported Sensors
 - RGB Mono Camera
 - Lidar
 - GPS
 - IMU

## Dependencies
 - NVIDIA GPU (required)
	 - tested on Maxwell and later
 - OptiX (required)
	 - 6.5.0
 - CUDA (required)
	 - tested with CUDA 10.2
 - GLFW >= 3.0 (required)
 - GLEW >= 1.0 (required)
 - openGL (required)
 - TensorRT (optional)
     - tested with TensorRT 7.0.0
     - need to explicitly enable TensorRT in cmake by setting CH_USE_TENSOR_RT=ON (default: USE_TENSOR_RT=OFF)

## CMake and Build Notes
 - consult the Chrono documentation for build instructions.

## Getting started with the demos
 - consult the Chrono documentation and reference manual for information on the Chrono::Sensor demos.

## Current Capabilities
 - Scene Rendering
	 - lights
		 - simple point light
		 - shadows
	 - Materials
		 - reflection based on material reflectance
		 - fresnel effect
		 - mesh support based on Wavefront OBJ+MTL format
		 - programmatic material creation
		 - partial transparency without refractance
	 - Objects
		 - Box primitives
		 - Sphere primitives
		 - cylinder primitives
		 - Triangle Mesh
 - Camera sensor
	 - ground-truth ray-traced camera rendering
	 - filter-based sensor model for user defined sensor model
 - Filters
  	 - Greyscale kernel
  	 - visualization using GLFW
  	 - copy-back filter for data access from CPU
  	 - save images to file at a specific path
  	 - convert lidar measurements to point cloud
  	 - image augmentation with pretrained neural nets
 - Lidar Sensor
    - single ray and multiray data generation
 - GPS Sensor
 - IMU Sensor
  - Accelerometer and Gyroscope

## Capabilities in Progress
 - expanded TensorRT model parsing
 - development of image augmentation networks
 - extending render support (lights, materials, objects, etc)
 - expanding mesh file support