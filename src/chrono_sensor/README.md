#### For information about the pychrono.sensor module including reasons/motives leveraged in the interfacing of sensor+python, see the sensor_readme.md in src/chrono_python/

# Chrono Sensor Module
Tested Systems:
 - Arch Linux: 2/17/2019
 - Windows 10: 7/28/2019

## Supported Sensors
 - RGB Mono Camera
 - Lidar
 - GPS
 - IMU

## Dependencies
 - NVIDIA GPU (required)
	 - tested on Maxwell and later
 - OptiX (required)
	 - 6.0.0 or 6.5.0
 - Cuda (required)
	 - 10.2 (tested on Linux and Windows)
	 - 10.1?
 - GLFW >= 3.? (required)
 - GLEW >= ?? (required)
 - openGL >= ?? (required)
 - TensorRT (optional)
     - tested with TensorRT 7.0.0
     - using TensorRT 7.0.0 requires use of OptiX 6.5
     - need to explicitly enable TensorRT in cmake by setting USE_TENSOR_RT=ON (default: USE_TENSOR_RT=OFF)

## CMake and Build Notes
 - Recommended to build and NOT install. Use the build directory rather than installing to a system directory
 - OptiX cmake paths that must be set
	 - liboptix.so
	 - liboptixu.so
	 - liboptix_prime.so
	 - optix include path: path to included directory that contains files such as optix.h

## Getting started with the demos
 - demo_SEN_buildtest
	 - builds if Chrono_Sensor and Build_Demos are enabled
	 - includes falling objects and camera sensor. A filter graph is added to the camera(s) that displays the original render, converts to greyscale, then displays greyscale image
 - demo_SEN_HMWMV
	 - only builds if Chrono_Sensor, Build_Demos, Chrono_Vehicle, and Chrono_Irrlicht are enabled
	 - runs a chrono vehicle with multiple sensor attached to the vehicle. This is the starting point for a typical autonomous vehicle simulation.
 - demo_SEN_sedan
	 - sedan demo for running a simulation along Park St. in Madison, WI. Mesh from Asher Elmquist is needed to run in simulation properly
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
		 - basic point light
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
	 -

## Capabilities in Progress
 - expanded TensorRT model parsing
 - development of image augmentation networks
 - extending render support (lights, materials, objects, etc)
 - improving mesh file support via tinyobjloader
 -
