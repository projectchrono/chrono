#### For information about the pychrono.sensor module including reasons/motives leveraged in the interfacing of sensor+python, see the sensor_readme.md in src/chrono_python/

# Chrono Sensor Module
Current Tested Systems:
 - Arch Linux: <font color="red">[Can be removed]</font>
 - Ubuntu 20.04: GCC 9.3, CUDA 10.2 <font color="red">[Need to update]</font>
 - Windows 10: VS 2019, CUDA 10.2 <font color="red">[Need to update]</font>
 - MacOS? <font color="red">[Need to update]</font>

## Supported Sensors
 - RGB monocular camera
 - Physics-based RGB monocular camera
 - Depth camera
 - Segmentation camera
 - Normal-map camera
 - Lidar
 - GPS
 - IMU

## Dependencies
 - NVIDIA GPU (required)
	 - tested on Maxwell and later <font color="red">[Need to update]</font>
 - OptiX (required)
	 - 9.0.0
 - CUDA (required)
	 - tested with CUDA 10.2 <font color="red">[Need to update]</font>
 - GLFW >= 3.0 (required) <font color="red">[Need to update]</font>
 - GLEW >= 1.0 (required) <font color="red">[Need to update]</font>
 - openGL (required) <font color="red">[Need to update]</font>
 - TensorRT (optional) <font color="red">[Can be removed]</font>
     - tested with TensorRT 7.0.0
     - need to explicitly enable TensorRT in cmake by setting CH_USE_SENSOR_TENSORRT=ON (default: OFF)

## CMake and Build Notes
 - consult the Chrono documentation for build instructions.

## Getting started with the demos
 - consult the Chrono documentation and reference manual for information on the Chrono::Sensor demos.

## Current Capabilities
 - Scene Rendering
	 - Lights
		 - Point light
		 - Spot light
		 - Directional light
		 - Rectangle area light
		 - Disk area light
		 - Environment light
		 - Shadows
	 - Materials
		 - Reflection based on material reflectance
		 - Fresnel effect
		 - Mesh support based on Wavefront OBJ+MTL format
		 - Programmatic material creation
		 - Legacy integrator supports partial transparency without refractance
	 - Objects
		 - Box primitives
		 - Sphere primitives
		 - Cylinder primitives
		 - Triangle Mesh
 - Camera sensor
	 - Ground-truth ray-traced camera rendering
	 - Filter-based sensor model for user defined sensor model
 - Filters
  	 - Greyscale kernel
  	 - Visualization using GLFW
  	 - Copy-back filter for data access from CPU
  	 - Save images to file at a specific path
  	 - Convert lidar measurements to point cloud
 - Lidar Sensor
     - Single ray and multi-ray data generation
 - GPS Sensor
 - IMU Sensor
     - Accelerometer and Gyroscope

## Capabilities in Progress