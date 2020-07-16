#### For information about the pychrono.sensor module including reasons/motives leveraged in the interfacing of sensor+python, see the sensor_readme.md in src/chrono_python/

# Chrono Sensor Module
Latest Tested Commit:
 - Arch Linux:
 - Ubuntu 20.04:
 - Windows 10:

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
     - need to explicitly enable TensorRT in cmake by setting USE_TENSOR_RT=ON (default: USE_TENSOR_RT=OFF)

## CMake and Build Notes
 - Recommended to build and NOT install. Use the build directory rather than installing to a system directory
 - set OptiX install path to root of where OptiX libraries are located
	 - Required OptiX paths (set manually if not found automatically by CMake)
   - liboptix library
	 - liboptixu library
	 - liboptix_prime library
	 - OptiX include path (path to included directory that contains files such as optix.h)
 - USE_CUDA_NVRTC (default=OFF)
   - Set USE_CUDA_NVRTC=ON to allow runtime generation of ray tracing (RT) programs
 - USE_TENSOR_RT (default=OFF)
   - Set USE_TENSOR_RT=ON to allow use of TensorRT in sensor module
   - set TensorRT libaries and include directory accordingly if enabled

## Getting started with the demos
 - demo_SEN_buildtest
	 - builds if Chrono_Sensor and Build_Demos are enabled
	 - includes falling objects, camera and lidar sensors. A filter graph is added to the camera(s) that displays the original render, converts to greyscale, then displays greyscale image
 - demo_SEN_HMWMV
	 - only builds if Chrono_Sensor, Build_Demos, Chrono_Vehicle, and Chrono_Irrlicht are enabled
	 - runs a chrono vehicle with multiple sensor attached to the vehicle. This is the starting point for a typical autonomous vehicle simulation.
 - demo_SEN_lidar
     - shows how to create a lidar and attach it to a chrono body. It also demonstrates a few lidar filters for turning the raw data into a point cloud and accessing both the raw data and point cloud
 - demo_SEN_camera
     - shows how to create a camera and attach it to a chrono body. It also demonstrates a few camera filters for manipulating the data and accessing the output data
 - demo_SEN_GPSIMU
     - demo that shows how to setup a GPS and an IMU and access their data streams
 - demo_SEN_NNcamera
     - demonstrates how to use a pretrained neural network as a filter for a camera


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
  -
 - IMU Sensor
  - Accelerometer and Gyroscope
  -

## Capabilities in Progress
 - expanded TensorRT model parsing
 - development of image augmentation networks
 - extending render support (lights, materials, objects, etc)
 - expanding mesh file support
 -
