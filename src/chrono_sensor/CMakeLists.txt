#=============================================================================
# CMake configuration file for Chrono Sensor module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_SENSOR "Enable the Chrono Sensor module" OFF)

if(NOT CH_ENABLE_MODULE_SENSOR)
  mark_as_advanced(FORCE CH_USE_SENSOR_OPTIX)
  mark_as_advanced(FORCE CH_USE_SENSOR_VULKAN_RT)
  mark_as_advanced(FORCE CH_USE_SENSOR_METAL_RT)

  mark_as_advanced(FORCE CH_USE_SENSOR_NVDB)
  mark_as_advanced(FORCE CH_USE_SENSOR_NVRTC)

  mark_as_advanced(FORCE GLM_INCLUDE_DIR)
  mark_as_advanced(FORCE GLEW_DIR)
  mark_as_advanced(FORCE glfw3_DIR)

  set(CH_USE_SENSOR_OPTIX FALSE)
  set(CH_USE_SENSOR_OPTIX ${CH_USE_SENSOR_OPTIX} PARENT_SCOPE)
  set(CH_USE_SENSOR_VULKAN_RT FALSE)
  set(CH_USE_SENSOR_VULKAN_RT ${CH_USE_SENSOR_VULKAN_RT} PARENT_SCOPE)
  set(CH_USE_SENSOR_METAL_RT FALSE)
  set(CH_USE_SENSOR_METAL_RT ${CH_USE_SENSOR_METAL_RT} PARENT_SCOPE)

  return()
endif()

message(STATUS "\n==== Chrono Sensor module ====\n")

mark_as_advanced(CLEAR CH_USE_SENSOR_OPTIX)
mark_as_advanced(CLEAR CH_USE_SENSOR_VULKAN_RT)
if(APPLE)
  mark_as_advanced(CLEAR CH_USE_SENSOR_METAL_RT)
else()
  # Metal exists only on Apple platforms; do not offer the switch anywhere else.
  mark_as_advanced(FORCE CH_USE_SENSOR_METAL_RT)
endif()

if(CH_USE_SENSOR_OPTIX)
  mark_as_advanced(CLEAR CH_USE_SENSOR_NVDB)
  mark_as_advanced(CLEAR CH_USE_SENSOR_NVRTC)
endif()

mark_as_advanced(CLEAR GLM_INCLUDE_DIR)
mark_as_advanced(CLEAR GLEW_DIR)
mark_as_advanced(CLEAR glfw3_DIR)

#-----------------------------------------------------------------------------
# Select rendered-sensor backend
#-----------------------------------------------------------------------------

set(CH_USE_SENSOR_OPTIX OFF CACHE BOOL "Enable legacy OptiX-dependent Chrono::Sensor features")
if(APPLE)
  # Apple has no CUDA/OptiX, and MoltenVK does not implement Vulkan ray tracing.
  # The native path on Apple Silicon is Metal ray tracing.
  set(CH_USE_SENSOR_VULKAN_RT OFF CACHE BOOL "Enable Vulkan ray-tracing Chrono::Sensor features")
  set(CH_USE_SENSOR_METAL_RT ON CACHE BOOL "Enable Metal ray-tracing Chrono::Sensor features")
else()
  set(CH_USE_SENSOR_VULKAN_RT ON CACHE BOOL "Enable Vulkan ray-tracing Chrono::Sensor features")
  set(CH_USE_SENSOR_METAL_RT OFF CACHE BOOL "Enable Metal ray-tracing Chrono::Sensor features")
endif()

# GPU requirements are declared PER FEATURE here, not per module.
#
# Chrono::Sensor itself requires no GPU backend at all: its default renderer is
# Vulkan RT, which is vendor-neutral and builds fine on AMD. Only the optional
# OptiX renderer is CUDA-and-NVIDIA-only. Attaching "requires CUDA" to the
# module rather than to the feature would wrongly disable the entire module on
# AMD hardware that can run most of it.
if(CH_USE_SENSOR_OPTIX)

  chrono_select_gpu_backend(CHRONO_SENSOR_OPTIX_BACKEND
                            FEATURE  "Chrono::Sensor OptiX renderer"
                            NAME     SENSOR_OPTIX
                            REQUIRES CUDA)

  if(CHRONO_SENSOR_OPTIX_BACKEND STREQUAL "NONE")

    chrono_gpu_feature_unavailable(FEATURE  "Chrono::Sensor OptiX renderer"
                                   REQUIRES "CUDA"
                                   OPTION   CH_USE_SENSOR_OPTIX
                                   CLASS    EXPLICIT)

  else()

    message(STATUS "Chrono CUDA architectures: ${CHRONO_CUDA_ARCHITECTURES}")

    message(STATUS "Looking for OptiX")
    find_package(OptiX)

    if(OptiX_INCLUDE)
      message(STATUS "  OptiX include directory: ${OptiX_INCLUDE}")
    else()
      message(WARNING "OptiX was not found; disabling OptiX-dependent Chrono::Sensor features")
      set(CH_USE_SENSOR_OPTIX FALSE CACHE BOOL "Enable OptiX-dependent Chrono::Sensor features" FORCE)
    endif()

  endif()

endif()

#-----------------------------------------------------------------------------
# Check Vulkan RT
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_VULKAN_RT)
  option(CH_USE_SENSOR_VULKAN_RT_GPU "Compile and use the Vulkan RT GPU renderer; CPU fallback is used only when no Vulkan RT GPU is available" ON)

  message(STATUS "Looking for Vulkan ray tracing support")
  find_package(Vulkan QUIET)
  find_package(Threads REQUIRED)
  if(Vulkan_FOUND AND TARGET Vulkan::Vulkan)
    message(STATUS "  Vulkan include directory: ${Vulkan_INCLUDE_DIRS}")
    if(CH_USE_SENSOR_VULKAN_RT_GPU)
      if(Vulkan_GLSLANG_VALIDATOR_EXECUTABLE)
        set(CH_VULKAN_GLSLANG_VALIDATOR ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE})
      else()
        find_program(CH_VULKAN_GLSLANG_VALIDATOR
                     NAMES glslangValidator
                     HINTS ENV VULKAN_SDK
                     PATH_SUFFIXES Bin bin)
      endif()
      if(NOT CH_VULKAN_GLSLANG_VALIDATOR)
        message(FATAL_ERROR "CH_USE_SENSOR_VULKAN_RT_GPU is ON but glslangValidator was not found. Install the Vulkan SDK or set CH_USE_SENSOR_VULKAN_RT_GPU=OFF for the explicit CPU-only fallback build.")
      endif()
      message(STATUS "  Vulkan RT shader compiler: ${CH_VULKAN_GLSLANG_VALIDATOR}")
    endif()
  else()
    message(STATUS "WARNING: Vulkan was not found; disabling Vulkan RT Chrono::Sensor features")
    set(CH_USE_SENSOR_VULKAN_RT FALSE CACHE BOOL "Enable Vulkan ray-tracing Chrono::Sensor features" FORCE)
  endif()
endif()

#-----------------------------------------------------------------------------
# Check Metal RT (Apple)
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_METAL_RT)
  if(APPLE)
    message(STATUS "Looking for Apple Metal ray tracing support")
    find_library(CH_METAL_FRAMEWORK Metal REQUIRED)
    find_library(CH_FOUNDATION_FRAMEWORK Foundation REQUIRED)
    find_library(CH_QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
    find_library(CH_COREGRAPHICS_FRAMEWORK CoreGraphics REQUIRED)
    find_library(CH_IMAGEIO_FRAMEWORK ImageIO REQUIRED)
    message(STATUS "  Metal framework: ${CH_METAL_FRAMEWORK}")
  else()
    message(STATUS "WARNING: Metal RT is only available on Apple platforms; disabling Metal RT Chrono::Sensor features")
    set(CH_USE_SENSOR_METAL_RT FALSE CACHE BOOL "Enable Metal ray-tracing Chrono::Sensor features" FORCE)
  endif()
endif()

if(CH_USE_SENSOR_OPTIX)

  message(STATUS "Building Chrono::Sensor with OptiX support")

  # Optional settings
  option(CH_USE_SENSOR_NVDB "Enable Chrono::FSI deformable terrain rendering in Chrono::Sensor" OFF)
  option(CH_USE_SENSOR_NVRTC "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX" ON)
  
  mark_as_advanced(CLEAR CH_USE_SENSOR_NVDB)
  mark_as_advanced(CLEAR CH_USE_SENSOR_NVRTC)

  # Replacement variable for configuration header
  set(CHRONO_HAS_OPTIX "#define CHRONO_HAS_OPTIX")

else()

  message(STATUS "Building Chrono::Sensor with NO OptiX support")

  mark_as_advanced(FORCE CH_USE_SENSOR_NVDB)
  mark_as_advanced(FORCE CH_USE_SENSOR_NVRTC)

  set(CHRONO_HAS_OPTIX "#undef CHRONO_HAS_OPTIX")

endif()

if(CH_USE_SENSOR_VULKAN_RT)
  message(STATUS "Building Chrono::Sensor with Vulkan RT support")
  set(CHRONO_HAS_VULKAN_RT "#define CHRONO_HAS_VULKAN_RT")
else()
  set(CHRONO_HAS_VULKAN_RT "#undef CHRONO_HAS_VULKAN_RT")
endif()

if(CH_USE_SENSOR_METAL_RT)
  message(STATUS "Building Chrono::Sensor with Metal RT support")
  set(CHRONO_HAS_METAL_RT "#define CHRONO_HAS_METAL_RT")
else()
  set(CHRONO_HAS_METAL_RT "#undef CHRONO_HAS_METAL_RT")
endif()

# Whether ANY ray-tracing backend was selected. Chrono::Sensor builds without one, and
# a build like that still has the GPS, IMU and tachometer sensors, but it has no rendered
# sensors and none of the camera, lidar or radar buffer types: those are declared only
# inside the per-backend blocks of ChSensorBuffer.h. Demos and tests that touch them have
# to be gated on this rather than on any single backend.
#
# Cached as INTERNAL so that src/demos and src/tests can read it, since a plain set() here
# would not reach a sibling directory, and so it never appears as a user-facing option: it
# is derived from the three backend switches, not chosen.
if(CH_USE_SENSOR_OPTIX OR CH_USE_SENSOR_VULKAN_RT OR CH_USE_SENSOR_METAL_RT)
  set(CH_SENSOR_HAS_RENDER_BACKEND TRUE CACHE INTERNAL "A Chrono::Sensor ray-tracing backend is enabled")
  set(CHRONO_HAS_SENSOR_RENDER "#define CHRONO_HAS_SENSOR_RENDER")
else()
  set(CH_SENSOR_HAS_RENDER_BACKEND FALSE CACHE INTERNAL "A Chrono::Sensor ray-tracing backend is enabled")
  set(CHRONO_HAS_SENSOR_RENDER "#undef CHRONO_HAS_SENSOR_RENDER")
endif()

#-----------------------------------------------------------------------------
# LIST CUDA FILES USED FOR RT PROGRAMS - TO BE COMPILED TO PTX SHADERS
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_OPTIX)

  set(Chrono_sensor_RT_SOURCES
      optix/shaders/box.cu
      optix/shaders/sphere.cu
      optix/shaders/cylinder.cu
      optix/shaders/miss.cu

      optix/shaders/material_shaders.cu
      
      optix/shaders/camera_raygen.cu
      optix/shaders/depth_cam_raygen.cu
      optix/shaders/lidar_raygen.cu
      optix/shaders/normal_cam_raygen.cu
      optix/shaders/phys_cam_raygen.cu
      optix/shaders/radar_raygen.cu
      optix/shaders/segment_cam_raygen.cu
      # ---- Register Your Customized Sensor Raygen Here (Cmake compiling) ---- #
  )

  if(CH_USE_SENSOR_NVDB)
    list(APPEND Chrono_sensor_RT_SOURCES
      optix/shaders/nvdb_vol_intersect.cu
    )
  endif()

  set(Chrono_sensor_RT_HEADERS
      optix/shaders/ChOptixLightStructs.h

      optix/shaders/camera_utils.cuh
      optix/shaders/device_utils.cuh
      optix/shaders/shader_utils.cuh
      
      optix/shaders/camera_hapke_shader.cuh
      optix/shaders/camera_volumetric_shader.cuh
      optix/shaders/camera_legacy_shader.cuh
      optix/shaders/camera_path_shader.cuh
      optix/shaders/camera_shader.cuh
      optix/shaders/depth_cam_shader.cuh
      optix/shaders/lidar_shader.cuh
      optix/shaders/normal_cam_shader.cuh
      optix/shaders/radar_shader.cuh
      optix/shaders/segment_cam_shader.cuh
      optix/shaders/shadow_shader.cuh
      # ---- Register Your Customized Sensor Shader Here (Cmake compiling) ---- #

      optix/shaders/ChOptixLightHubs.cuh
      optix/shaders/ChOptixPointLight.cuh
      optix/shaders/ChOptixSpotLight.cuh
      optix/shaders/ChOptixDirectionalLight.cuh
      optix/shaders/ChOptixRectangleLight.cuh
      optix/shaders/ChOptixDiskLight.cuh
      optix/shaders/ChOptixEnvironmentLight.cuh
      # ---- Register Your Customized Light Type Here (Cmake compiling) ---- #

  )

  source_group("RT Programs" FILES ${Chrono_sensor_RT_SOURCES} ${Chrono_sensor_RT_HEADERS})

else()

  set(Chrono_sensor_RT_SOURCES "")
  set(Chrono_sensor_RT_HEADERS "")

endif()

#-----------------------------------------------------------------------------
# LIST CUDA FILES THAT ARE TO BE COMPILED AS SOURCE
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_OPTIX)

  set(Chrono_sensor_CUDA_SOURCES
      cuda/grayscale.cu
      cuda/pointcloud.cu
      cuda/lidar_reduce.cu
      cuda/camera_noise.cu
      cuda/lidar_noise.cu
      cuda/curand_utils.cu
      cuda/image_ops.cu
      cuda/nn_prep.cu
      cuda/lidar_clip.cu
      cuda/radarprocess.cu
      cuda/cuda_utils.cu
      cuda/phys_cam_ops.cu
  )

  set(Chrono_sensor_CUDA_HEADERS
      cuda/grayscale.cuh
      cuda/grayscale.cuh
      cuda/lidar_reduce.cuh
      cuda/camera_noise.cuh
      cuda/lidar_noise.cuh
      cuda/curand_utils.cuh
      cuda/image_ops.cuh
      cuda/nn_prep.cuh
      cuda/lidar_clip.cuh
      cuda/radarprocess.cuh
      cuda/cuda_utils.cuh
      cuda/phys_cam_ops.cuh
  )

  if(CH_ENABLE_MODULE_FSI_SPH)
    list(APPEND Chrono_sensor_CUDA_SOURCES
      cuda/fsi_sph_render.cu
    )
    list(APPEND Chrono_sensor_CUDA_HEADERS
      cuda/fsi_sph_render.cuh
    )
  endif()

  source_group("cuda" FILES ${Chrono_sensor_CUDA_SOURCES} ${Chrono_sensor_CUDA_HEADERS})

else()

  set(Chrono_sensor_CUDA_SOURCES "")
  set(Chrono_sensor_CUDA_HEADERS "")

endif()

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------

set(Chrono_sensor_SOURCES
    ChSensorManager.cpp
    ChDynamicsManager.cpp
)

set(Chrono_sensor_HEADERS
    ChApiSensor.h
    ChSensorManager.h
    ChDynamicsManager.h
    ChFsiSphRender.h
    ChSensorRenderTypes.h
)

source_group("source" FILES ${Chrono_sensor_SOURCES} ${Chrono_sensor_HEADERS})

set(Chrono_sensor_SENSORS_SOURCES
    sensors/ChSensor.cpp
    sensors/ChNoiseModel.cpp
    sensors/ChIMUSensor.cpp
    sensors/ChGPSSensor.cpp
    sensors/ChTachometerSensor.cpp
    sensors/Sensor.cpp
)

set(Chrono_sensor_SENSORS_HEADERS
    sensors/ChSensor.h
    sensors/ChNoiseModel.h
    sensors/ChIMUSensor.h
    sensors/ChGPSSensor.h
    sensors/ChTachometerSensor.h
    sensors/ChSensorBuffer.h
    sensors/Sensor.h
)

if(CH_USE_SENSOR_OPTIX)
  set(Chrono_sensor_SENSORS_SOURCES ${Chrono_sensor_SENSORS_SOURCES}
      sensors/ChOptixSensor.cpp
      sensors/ChCameraSensor.cpp
      sensors/ChPhysCameraSensor.cpp
      sensors/ChSegmentationCamera.cpp
      sensors/ChDepthCamera.cpp
      sensors/ChNormalCamera.cpp
      sensors/ChLidarSensor.cpp
      sensors/ChRadarSensor.cpp
  )
  set(Chrono_sensor_SENSORS_HEADERS ${Chrono_sensor_SENSORS_HEADERS}
      sensors/ChOptixSensor.h
      sensors/ChCameraSensor.h
      sensors/ChPhysCameraSensor.h
      sensors/ChSegmentationCamera.h
      sensors/ChDepthCamera.h
      sensors/ChNormalCamera.h
      sensors/ChLidarSensor.h
      sensors/ChRadarSensor.h
 )
endif()

if(CH_USE_SENSOR_VULKAN_RT)
  set(Chrono_sensor_SENSORS_SOURCES ${Chrono_sensor_SENSORS_SOURCES}
      sensors/ChVulkanSensor.cpp
      sensors/ChVulkanCameraSensor.cpp
  )
  set(Chrono_sensor_SENSORS_HEADERS ${Chrono_sensor_SENSORS_HEADERS}
      sensors/ChVulkanSensor.h
      sensors/ChVulkanCameraSensor.h
  )

  if(NOT CH_USE_SENSOR_OPTIX)
    # Build the standard render sensor front-ends against the Vulkan backend so existing
    # demos/apps using ChCameraSensor/ChLidarSensor/ChRadarSensor keep compiling without OptiX.
    list(APPEND Chrono_sensor_SENSORS_SOURCES
        sensors/ChCameraSensor.cpp
        sensors/ChPhysCameraSensor.cpp
        sensors/ChSegmentationCamera.cpp
        sensors/ChDepthCamera.cpp
        sensors/ChNormalCamera.cpp
        sensors/ChLidarSensor.cpp
        sensors/ChRadarSensor.cpp
    )
    list(APPEND Chrono_sensor_SENSORS_HEADERS
        sensors/ChCameraSensor.h
        sensors/ChPhysCameraSensor.h
        sensors/ChSegmentationCamera.h
        sensors/ChDepthCamera.h
        sensors/ChNormalCamera.h
        sensors/ChLidarSensor.h
        sensors/ChRadarSensor.h
    )
  endif()
endif()

if(CH_USE_SENSOR_METAL_RT)
  set(Chrono_sensor_SENSORS_SOURCES ${Chrono_sensor_SENSORS_SOURCES}
      sensors/ChMetalSensor.cpp
  )
  set(Chrono_sensor_SENSORS_HEADERS ${Chrono_sensor_SENSORS_HEADERS}
      sensors/ChMetalSensor.h
  )

  if(NOT CH_USE_SENSOR_OPTIX AND NOT CH_USE_SENSOR_VULKAN_RT)
    # Build the standard render sensor front-ends against the Metal backend.
    list(APPEND Chrono_sensor_SENSORS_SOURCES
        sensors/ChCameraSensor.cpp
        sensors/ChPhysCameraSensor.cpp
        sensors/ChDepthCamera.cpp
        sensors/ChNormalCamera.cpp
        sensors/ChSegmentationCamera.cpp
        sensors/ChLidarSensor.cpp
        sensors/ChRadarSensor.cpp
    )
    list(APPEND Chrono_sensor_SENSORS_HEADERS
        sensors/ChCameraSensor.h
        sensors/ChPhysCameraSensor.h
        sensors/ChDepthCamera.h
        sensors/ChNormalCamera.h
        sensors/ChSegmentationCamera.h
        sensors/ChLidarSensor.h
        sensors/ChRadarSensor.h
    )
  endif()
endif()

source_group("source" FILES ${Chrono_sensor_SENSORS_SOURCES} ${Chrono_sensor_SENSORS_HEADERS})

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR OPTIX LIBRARY
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_OPTIX)

  set(Chrono_sensor_OPTIX_SOURCES
      optix/ChOptixEngine.cpp
      optix/ChOptixGeometry.cpp
      optix/ChOptixPipeline.cpp
      optix/ChOptixUtils.cpp
      optix/ChOptixScene.cpp
      optix/ChFilterOptixRender.cpp
      optix/ChNVDBVolume.cpp
  )
  
  set(Chrono_sensor_OPTIX_HEADERS
      optix/ChOptixEngine.h
      optix/ChOptixGeometry.h
      optix/ChOptixPipeline.h
      optix/ChOptixUtils.h
      optix/ChOptixScene.h
      optix/ChFilterOptixRender.h
      optix/ChOptixDefinitions.h
      optix/ChNVDBVolume.h
  )
  
  source_group("Optix" FILES ${Chrono_sensor_OPTIX_SOURCES} ${Chrono_sensor_OPTIX_HEADERS})

else()

  set(Chrono_sensor_OPTIX_SOURCES "")
  set(Chrono_sensor_OPTIX_HEADERS "")

endif()

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR VULKAN RT LIBRARY
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_VULKAN_RT)
  set(Chrono_sensor_VULKAN_SOURCES
      vulkan/ChVulkanRTUtils.cpp
      vulkan/ChVulkanRTDevice.cpp
      vulkan/ChVulkanRTBuffer.cpp
      vulkan/ChVulkanRTScene.cpp
      vulkan/ChFilterVulkanRTRender.cpp
      vulkan/ChVulkanRTEngine.cpp
  )

  set(Chrono_sensor_VULKAN_HEADERS
      vulkan/ChVulkanRTDefinitions.h
      vulkan/ChVulkanRTUtils.h
      vulkan/ChVulkanRTDevice.h
      vulkan/ChVulkanRTBuffer.h
      vulkan/ChVulkanRTScene.h
      vulkan/ChFilterVulkanRTRender.h
      vulkan/ChVulkanRTEngine.h
  )

  set(Chrono_sensor_VULKAN_SHADER_SOURCES
      vulkan/shaders/chrono_sensor_vkrt.rgen
      vulkan/shaders/chrono_sensor_vkrt.rmiss
      vulkan/shaders/chrono_sensor_vkrt.rchit
      vulkan/shaders/chrono_sensor_vkrt_shadow.rmiss
      vulkan/shaders/chrono_sensor_vkrt_shadow.rahit
  )

  source_group("Vulkan RT" FILES ${Chrono_sensor_VULKAN_SOURCES} ${Chrono_sensor_VULKAN_HEADERS})
  source_group("Vulkan RT\\Shaders" FILES ${Chrono_sensor_VULKAN_SHADER_SOURCES})
else()
  set(Chrono_sensor_VULKAN_SOURCES "")
  set(Chrono_sensor_VULKAN_HEADERS "")
  set(Chrono_sensor_VULKAN_SHADER_SOURCES "")
endif()

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR METAL RT LIBRARY (Apple)
#-----------------------------------------------------------------------------

if(CH_USE_SENSOR_METAL_RT)
  set(Chrono_sensor_METAL_SOURCES
      metal/ChMetalSceneBuilder.cpp
      metal/ChMetalRTScene.cpp
      metal/ChMetalRTEngine.cpp
      metal/ChMetalRTDevice.mm
      metal/ChMetalRTRenderer.mm
      metal/ChMetalPhysCamOps.mm
      metal/ChFilterMetalRTRender.mm
      metal/ChFilterMetalVisualize.mm
      metal/ChFilterMetalVisualizePointCloud.mm
      metal/ChMetalPerfOverlay.mm
  )
  set(Chrono_sensor_METAL_HEADERS
      metal/ChMetalRTDefinitions.h
      metal/ChMetalRenderTypes.h
      metal/ChMetalSceneBuilder.h
      metal/ChMetalPhysCamOps.h
      metal/ChMetalRTDevice.h
      metal/ChMetalRTScene.h
      metal/ChMetalRTRenderer.h
      metal/ChFilterMetalRTRender.h
      metal/ChFilterMetalVisualize.h
      metal/ChFilterMetalVisualizePointCloud.h
      metal/ChMetalPerfOverlay.h
      metal/ChOrbitCameraControl.h
      metal/ChMetalRTEngine.h
  )
  # The .mm files are Objective-C++ with ARC.
  set_source_files_properties(
      metal/ChMetalRTDevice.mm
      metal/ChMetalRTRenderer.mm
      metal/ChMetalPhysCamOps.mm
      metal/ChFilterMetalRTRender.mm
      metal/ChFilterMetalVisualize.mm
      metal/ChFilterMetalVisualizePointCloud.mm
      metal/ChMetalPerfOverlay.mm
      PROPERTIES LANGUAGE CXX COMPILE_FLAGS "-x objective-c++ -fobjc-arc")
  # Metal Shading Language sources. Embedded as C string literals and compiled at run time
  # by the Metal driver, so they are headers rather than separately-compiled shader files,
  # but they live under shaders/ like the OptiX and Vulkan RT programs do.
  set(Chrono_sensor_METAL_SHADER_HEADERS
      metal/shaders/ChMetalRTShaderMSL.h
      metal/shaders/ChMetalPhysCamOpsMSL.h
  )

  source_group("Metal RT" FILES ${Chrono_sensor_METAL_SOURCES} ${Chrono_sensor_METAL_HEADERS})
  source_group("Metal RT\\Shaders" FILES ${Chrono_sensor_METAL_SHADER_HEADERS})
else()
  set(Chrono_sensor_METAL_SOURCES "")
  set(Chrono_sensor_METAL_HEADERS "")
  set(Chrono_sensor_METAL_SHADER_HEADERS "")
endif()

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------

set(Chrono_sensor_FILTERS_SOURCES
    filters/ChFilter.cpp
    filters/ChFilterAccess.cpp
    filters/ChFilterIMUUpdate.cpp
    filters/ChFilterGPSUpdate.cpp
    filters/ChFilterTachometerUpdate.cpp
)

set(Chrono_sensor_FILTERS_HEADERS
    filters/ChFilter.h
    filters/ChFilterAccess.h
    filters/ChFilterIMUUpdate.h
    filters/ChFilterGPSUpdate.h
    filters/ChFilterTachometerUpdate.h
)

if(CH_USE_SENSOR_OPTIX OR CH_USE_SENSOR_VULKAN_RT)
  # Camera/demo filters. Under Vulkan-only these files compile CPU/host-buffer
  # paths and avoid CUDA/NPP symbols.
  set(Chrono_sensor_FILTERS_SOURCES ${Chrono_sensor_FILTERS_SOURCES}
      filters/ChFilterCameraNoise.cpp
      filters/ChFilterGrayscale.cpp
      filters/ChFilterImageOps.cpp
      filters/ChFilterVisualize.cpp
      filters/ChFilterSave.cpp
  )
  set(Chrono_sensor_FILTERS_HEADERS ${Chrono_sensor_FILTERS_HEADERS}
      filters/ChFilterCameraNoise.h
      filters/ChFilterGrayscale.h
      filters/ChFilterImageOps.h
      filters/ChFilterVisualize.h
      filters/ChFilterSave.h
  )
endif()

if(CH_USE_SENSOR_METAL_RT)
  # Host-buffer camera filters for the Metal backend. ChFilterVisualize is included: it
  # already carries a CHRONO_HAS_METAL_RT branch that displays a host buffer through GLFW,
  # and every pre-existing sensor demo uses it to open a window. ChFilterMetalVisualize
  # remains available as the native viewer with orbit controls.
  set(Chrono_sensor_FILTERS_SOURCES ${Chrono_sensor_FILTERS_SOURCES}
      filters/ChFilterVisualize.cpp
      filters/ChFilterImageOps.cpp
      filters/ChFilterGrayscale.cpp
      filters/ChFilterCameraNoise.cpp
      filters/ChFilterSave.cpp
      filters/ChFilterPhysCameraDefocusBlur.cpp
      filters/ChFilterPhysCameraVignetting.cpp
      filters/ChFilterPhysCameraAggregator.cpp
      filters/ChFilterPhysCameraNoise.cpp
      filters/ChFilterPhysCameraExpsrToDV.cpp
      # Point-cloud filters. The CUDA kernels in cuda/pointcloud.cu are unavailable here, but
      # each of these files already carries a host implementation behind #ifdef CHRONO_HAS_OPTIX
      # -- the transform is per-element polar-to-Cartesian, so the CPU path is equivalent and
      # fast enough (a 1024x64 scan is 65k points).
      filters/ChFilterPCfromDepth.cpp
      filters/ChFilterSavePtCloud.cpp
      filters/ChFilterRadarXYZReturn.cpp
      filters/ChFilterRadarXYZVisualize.cpp
      filters/ChFilterRadarProcess.cpp
      filters/ChFilterRadarSavePC.cpp
      filters/ChFilterLidarNoise.cpp
      filters/ChFilterVisualizePointCloud.cpp
      filters/ChFilterLidarIntensityClip.cpp
      filters/ChFilterLidarReduce.cpp
      filters/ChFilterRadarVisualizeCluster.cpp
  )
  set(Chrono_sensor_FILTERS_HEADERS ${Chrono_sensor_FILTERS_HEADERS}
      filters/ChFilterVisualize.h
      filters/ChFilterImageOps.h
      filters/ChFilterGrayscale.h
      filters/ChFilterCameraNoise.h
      filters/ChFilterSave.h
      filters/ChFilterPhysCameraDefocusBlur.h
      filters/ChFilterPhysCameraVignetting.h
      filters/ChFilterPhysCameraAggregator.h
      filters/ChFilterPhysCameraNoise.h
      filters/ChFilterPhysCameraExpsrToDV.h
      filters/ChFilterPCfromDepth.h
      filters/ChFilterSavePtCloud.h
      filters/ChFilterRadarXYZReturn.h
      filters/ChFilterRadarXYZVisualize.h
      filters/ChFilterRadarProcess.h
      filters/ChFilterRadarSavePC.h
      filters/ChFilterLidarNoise.h
      filters/ChFilterVisualizePointCloud.h
      filters/ChFilterLidarIntensityClip.h
      filters/ChFilterLidarReduce.h
      filters/ChFilterRadarVisualizeCluster.h
  )
endif()

if(CH_USE_SENSOR_OPTIX)
  set(Chrono_sensor_FILTERS_SOURCES ${Chrono_sensor_FILTERS_SOURCES}
      filters/ChFilterCameraExposure.cpp
      filters/ChFilterLidarNoise.cpp
      filters/ChFilterLidarIntensityClip.cpp
      filters/ChFilterLidarReduce.cpp
      filters/ChFilterPCfromDepth.cpp
      filters/ChFilterRadarProcess.cpp
      filters/ChFilterRadarXYZReturn.cpp
      filters/ChFilterRadarSavePC.cpp
      filters/ChFilterRadarXYZVisualize.cpp
      filters/ChFilterRadarVisualizeCluster.cpp
      filters/ChFilterSavePtCloud.cpp
      filters/ChFilterVisualizePointCloud.cpp
      filters/ChFilterPhysCameraDefocusBlur.cpp
      filters/ChFilterPhysCameraVignetting.cpp
      filters/ChFilterPhysCameraAggregator.cpp
      filters/ChFilterPhysCameraNoise.cpp
      filters/ChFilterPhysCameraExpsrToDV.cpp
  )
  set(Chrono_sensor_FILTERS_HEADERS ${Chrono_sensor_FILTERS_HEADERS}
      filters/ChFilterCameraExposure.h
      filters/ChFilterLidarNoise.h
      filters/ChFilterLidarReduce.h
      filters/ChFilterLidarIntensityClip.h
      filters/ChFilterPCfromDepth.h
      filters/ChFilterRadarProcess.h
      filters/ChFilterRadarXYZReturn.h
      filters/ChFilterRadarSavePC.h
      filters/ChFilterRadarXYZVisualize.h
      filters/ChFilterRadarVisualizeCluster.h
      filters/ChFilterSavePtCloud.h
      filters/ChFilterVisualizePointCloud.h
      filters/ChFilterPhysCameraDefocusBlur.h
      filters/ChFilterPhysCameraVignetting.h
      filters/ChFilterPhysCameraAggregator.h
      filters/ChFilterPhysCameraNoise.h
      filters/ChFilterPhysCameraExpsrToDV.h
  )
endif()

if(CH_USE_SENSOR_VULKAN_RT AND NOT CH_USE_SENSOR_OPTIX)
  # Vulkan-only lidar/radar/point-cloud/physics-camera filters. These files use
  # host-visible Vulkan RT buffers and must not pull CUDA or OptiX dependencies.
  list(APPEND Chrono_sensor_FILTERS_SOURCES
      filters/ChFilterLidarNoise.cpp
      filters/ChFilterLidarIntensityClip.cpp
      filters/ChFilterLidarReduce.cpp
      filters/ChFilterPCfromDepth.cpp
      filters/ChFilterRadarProcess.cpp
      filters/ChFilterRadarXYZReturn.cpp
      filters/ChFilterRadarSavePC.cpp
      filters/ChFilterRadarXYZVisualize.cpp
      filters/ChFilterRadarVisualizeCluster.cpp
      filters/ChFilterSavePtCloud.cpp
      filters/ChFilterVisualizePointCloud.cpp
      filters/ChFilterPhysCameraDefocusBlur.cpp
      filters/ChFilterPhysCameraVignetting.cpp
      filters/ChFilterPhysCameraAggregator.cpp
      filters/ChFilterPhysCameraNoise.cpp
      filters/ChFilterPhysCameraExpsrToDV.cpp
  )
  list(APPEND Chrono_sensor_FILTERS_HEADERS
      filters/ChFilterLidarNoise.h
      filters/ChFilterLidarReduce.h
      filters/ChFilterLidarIntensityClip.h
      filters/ChFilterPCfromDepth.h
      filters/ChFilterRadarProcess.h
      filters/ChFilterRadarXYZReturn.h
      filters/ChFilterRadarSavePC.h
      filters/ChFilterRadarXYZVisualize.h
      filters/ChFilterRadarVisualizeCluster.h
      filters/ChFilterSavePtCloud.h
      filters/ChFilterVisualizePointCloud.h
      filters/ChFilterPhysCameraDefocusBlur.h
      filters/ChFilterPhysCameraVignetting.h
      filters/ChFilterPhysCameraAggregator.h
      filters/ChFilterPhysCameraNoise.h
      filters/ChFilterPhysCameraExpsrToDV.h
  )
endif()

source_group("filters" FILES ${Chrono_sensor_FILTERS_SOURCES} ${Chrono_sensor_FILTERS_HEADERS})

#-----------------------------------------------------------------------------
# LIST THE UTILITY FILES THAT WILL BE EXPOSED TO THE USER
#-----------------------------------------------------------------------------

set(Chrono_sensor_UTILS_SOURCES
    utils/ChSensorUtils.cpp
    utils/ChSensorUtilsJSON.cpp
    utils/ChGPSUtils.cpp
    utils/Kdtree.cpp
    utils/Dbscan.cpp
)

set(Chrono_sensor_UTILS_HEADERS
    utils/ChSensorUtils.h
    utils/ChSensorUtilsJSON.h
    utils/ChGPSUtils.h
    utils/Kdtree.h
    utils/Dbscan.h
)

if(CH_USE_SENSOR_OPTIX)
  set(Chrono_sensor_UTILS_HEADERS ${Chrono_sensor_UTILS_HEADERS}
      utils/CudaMallocHelper.h
  )
endif()

source_group("utils" FILES ${Chrono_sensor_UTILS_SOURCES} ${Chrono_sensor_UTILS_HEADERS})

set(SENSOR_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("utils\\stb" FILES ${SENSOR_STB_FILES})


set(SENSOR_TINYOBJ_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
)
source_group("utils\\tinyobjloader" FILES ${SENSOR_TINYOBJ_FILES})

# Disable all warnings from external code (see cmake/ChronoWarnings.cmake)
ch_disable_warnings_on_sources(${SENSOR_STB_FILES} ${SENSOR_TINYOBJ_FILES})

#-----------------------------------------------------------------------------
# Shader directory in the build tree
#
# The runtime reads shader files out of this directory (GetShaderFromFile in
# optix/ChOptixUtils.cpp), so its contents must track the sources. The copy itself is a
# BUILD step, not a configure step: see Chrono_sensor_shader_copy (NVRTC) and
# Chrono_sensor_ptx_copy (NVCC) below. Copying here with file(COPY) instead would leave a
# stale copy whenever a shader is edited and only the build is re-run, so the old shader
# would keep being used with no error and no warning.
#-----------------------------------------------------------------------------

if(NOT CH_BUILD_SENSOR_SHADERS)
  set(CH_BUILD_SENSOR_SHADERS "${PROJECT_BINARY_DIR}/sensor_shaders")
endif()
if(NOT CH_INSTALL_SENSOR_SHADERS)
  set(CH_INSTALL_SENSOR_SHADERS "share/chrono_sensor/shaders")
endif()

if(CH_USE_SENSOR_OPTIX)
  message(STATUS "Shader directory in BUILD tree: ${CH_BUILD_SENSOR_SHADERS}")
  # Created at configure time so that install(DIRECTORY ...) below always has a directory,
  # even before a build has populated it.
  file(MAKE_DIRECTORY ${CH_BUILD_SENSOR_SHADERS})
endif()

if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
  set(CH_BUILD_SENSOR_VULKAN_SHADERS "${CH_BUILD_SENSOR_SHADERS}/vulkan")
  file(MAKE_DIRECTORY ${CH_BUILD_SENSOR_VULKAN_SHADERS})
  set(Chrono_sensor_VULKAN_SPV)
  foreach(_vk_shader ${Chrono_sensor_VULKAN_SHADER_SOURCES})
    get_filename_component(_vk_shader_name ${_vk_shader} NAME)
    set(_vk_spv "${CH_BUILD_SENSOR_VULKAN_SHADERS}/${_vk_shader_name}.spv")
    add_custom_command(
        OUTPUT ${_vk_spv}
        COMMAND ${CH_VULKAN_GLSLANG_VALIDATOR} -V --target-env vulkan1.2 -o ${_vk_spv} ${CMAKE_CURRENT_SOURCE_DIR}/${_vk_shader}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${_vk_shader}
        COMMENT "Compile Vulkan RT shader ${_vk_shader_name}"
        VERBATIM)
    list(APPEND Chrono_sensor_VULKAN_SPV ${_vk_spv})
  endforeach()
  add_custom_target(Chrono_sensor_vulkan_spv ALL DEPENDS ${Chrono_sensor_VULKAN_SPV})
endif()

# ----------------------------------------------------------------------------
# Name of the Chrono::Sensor configuration file in the BUILD tree
# ----------------------------------------------------------------------------

set(Chrono_sensor_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h)
source_group("" FILES ${Chrono_sensor_CONFIG_FILE})

#-----------------------------------------------------------------------------
# Create the Chrono_sensor library
#-----------------------------------------------------------------------------

# Collect all sources
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_SENSORS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_SENSORS_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_UTILS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_UTILS_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_OPTIX_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_OPTIX_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_VULKAN_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_VULKAN_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_METAL_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_METAL_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_METAL_SHADER_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_FILTERS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_FILTERS_HEADERS})

list(APPEND ALL_CH_SENSOR_FILES ${SENSOR_STB_FILES})
list(APPEND ALL_CH_SENSOR_FILES ${SENSOR_TINYOBJ_FILES})

list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_CONFIG_FILE})

add_library(Chrono_sensor ${ALL_CH_SENSOR_FILES} ${Chrono_sensor_CUDA_SOURCES} ${Chrono_sensor_VULKAN_SHADER_SOURCES})
add_library(Chrono::sensor ALIAS Chrono_sensor)

if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
  add_dependencies(Chrono_sensor Chrono_sensor_vulkan_spv)
endif()

set_target_properties(Chrono_sensor PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_sensor PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_sensor PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_sensor PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_sensor PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

# ------------------------------------------------------------------------------
# Find and set everything needed for OptiX
# ------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:NOMINMAX>)
endif()

if(CH_USE_SENSOR_OPTIX)
  target_include_directories(Chrono_sensor SYSTEM PUBLIC "${OptiX_INCLUDE}")

  # CUDA::cudart_static (and the other CUDA::* imports linked below) bring in the CUDA
  # Toolkit's own include directories transitively. Re-declaring that same path here as
  # SYSTEM forces -isystem treatment on GCC/Clang regardless of CMake's default handling
  # for imported targets; the MSVC-side equivalent (/external:*) is added further below.
  target_include_directories(Chrono_sensor SYSTEM PRIVATE ${CUDAToolkit_INCLUDE_DIRS})

  target_link_libraries(Chrono_sensor PUBLIC CUDA::cudart_static)

  # Declare the backend that goes with the link above.
  #
  # With OptiX enabled, Chrono::Sensor exports CUDA to its consumers: it links
  # CUDA::cudart_static PUBLIC, and ChSensorBuffer.h -- an installed header --
  # includes <cuda_fp16.h> under CHRONO_HAS_OPTIX. Every consumer therefore
  # compiles against CUDA's vector types whether it asked to or not.
  #
  # CHRONO_USE_CUDA is what tells chrono/gpu/ChGpuPrimitives.h that those types
  # already exist. Without it that header takes its "no GPU SDK here" branch and
  # defines its own int2/float3/make_int2/..., which then collide with CUDA's.
  # Exporting the headers but not the macro is the same visibility mismatch the
  # GPU modules avoid by construction; sensor predates that helper and links
  # CUDA by hand, so it has to say so by hand too.
  #
  # This is deliberately just the define: chrono_apply_gpu_backend() would also
  # link cuBLAS, cuSPARSE and NVRTC, which sensor does not want, and would set
  # CUDA_ARCHITECTURES a second time.
  target_compile_definitions(Chrono_sensor PUBLIC CHRONO_USE_CUDA)

  target_link_libraries(Chrono_sensor PRIVATE CUDA::nppc)
  target_link_libraries(Chrono_sensor PRIVATE CUDA::nppig)
  target_link_libraries(Chrono_sensor PRIVATE CUDA::nppidei)

  if(CH_USE_SENSOR_NVDB)
    target_link_libraries(Chrono_sensor PRIVATE OpenVDB::openvdb)
    target_link_libraries(Chrono_sensor PRIVATE OpenVDB::nanovdb)
  endif()

  if(CH_ENABLE_MODULE_FSI_SPH)
    target_link_libraries(Chrono_sensor PUBLIC Chrono_fsisph)
  endif()

endif()

if(CH_USE_SENSOR_VULKAN_RT)
  target_link_libraries(Chrono_sensor PUBLIC Vulkan::Vulkan)
  target_link_libraries(Chrono_sensor PRIVATE Threads::Threads)
  if(CH_USE_SENSOR_VULKAN_RT_GPU)
    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CHRONO_SENSOR_VULKAN_RT_GPU_ENABLED>)
  endif()
endif()

if(CH_USE_SENSOR_METAL_RT)
  find_library(CH_COCOA_FRAMEWORK Cocoa REQUIRED)
  target_link_libraries(Chrono_sensor PRIVATE
      ${CH_METAL_FRAMEWORK}
      ${CH_FOUNDATION_FRAMEWORK}
      ${CH_QUARTZCORE_FRAMEWORK}
      ${CH_COREGRAPHICS_FRAMEWORK}
      ${CH_IMAGEIO_FRAMEWORK}
      ${CH_COCOA_FRAMEWORK})
endif()

# print cxx compiler flags\
## TODO: DARIOM need to be passed to the host compiler?
# if(MSVC AND MSVC_VERSION GREATER_EQUAL 1900)
#   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
# endif()

if(CH_USE_SENSOR_OPTIX)
  set_target_properties(Chrono_sensor PROPERTIES CUDA_ARCHITECTURES "${CHRONO_CUDA_ARCHITECTURES}")

  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)  # Eigen host constexpr helpers called from __host__ __device__ functions
endif()

# ------------------------------------------------------------------------------
# Optionally use NVRTC to compile shader code rather than NVCC to PTX
# ------------------------------------------------------------------------------

if(CH_USE_SENSOR_OPTIX)

  if(CH_USE_SENSOR_NVRTC)
    message(STATUS "Looking for CUDA NVRTC support")
    find_package(CUDAToolkit REQUIRED)
    if(NOT TARGET CUDA::nvrtc)
        message(WARNING "  CUDA::nvrtc target is missing; NVRTC will be turned off")
        set(CH_USE_SENSOR_NVRTC OFF CACHE BOOL "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX" FORCE)
    endif()
  endif()
  
  if(CH_USE_SENSOR_NVRTC)
    message(STATUS "  CUDA NVRTC found and enabled")
    message(STATUS "  Building Chrono::Sensor with NVRTC ON")

    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVRTC>)
    target_link_libraries(Chrono_sensor PRIVATE CUDA::nvrtc)
  
    # Flags used directly in the source code as defines

    #set(CUDA_NVRTC_FLAGS -arch compute_30 -use_fast_math -lineinfo -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    set(CUDA_NVRTC_FLAGS -use_fast_math -std=c++17 -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    mark_as_advanced(CUDA_NVRTC_FLAGS)
    message(STATUS "  CUDA NVRTC flags: ${CUDA_NVRTC_FLAGS}")

    set(CUDA_NVRTC_FLAG_LIST)
    foreach(item ${CUDA_NVRTC_FLAGS}) #CUDA_NVRTC_FLAGS CUDA_NVCC_FLAGS
      set(CUDA_NVRTC_FLAG_LIST "${CUDA_NVRTC_FLAG_LIST} \\\n  \"${item}\",")
    endforeach()
    set(CUDA_NVRTC_FLAG_LIST "${CUDA_NVRTC_FLAG_LIST} \\\n  0,")
  
    # CUDA 13 places CCCL/libcu++ headers in <cuda>/include/cccl, but curand_kernel.h
    # includes them as <cuda/std/...>. NVRTC does not automatically add that directory.
    set(_CHRONO_CUDA_CCCL_INCLUDE_DIRS)
    foreach(_cuda_include_dir ${CUDAToolkit_INCLUDE_DIRS})
      if(EXISTS "${_cuda_include_dir}/cccl/cuda/std/type_traits")
        list(APPEND _CHRONO_CUDA_CCCL_INCLUDE_DIRS "${_cuda_include_dir}/cccl")
      endif()
    endforeach()
    if(DEFINED CUDAToolkit_TARGET_DIR AND EXISTS "${CUDAToolkit_TARGET_DIR}/include/cccl/cuda/std/type_traits")
      list(APPEND _CHRONO_CUDA_CCCL_INCLUDE_DIRS "${CUDAToolkit_TARGET_DIR}/include/cccl")
    endif()
    list(REMOVE_DUPLICATES _CHRONO_CUDA_CCCL_INCLUDE_DIRS)

    set(CUDA_NVRTC_INCLUDE_DIRS
        ${OptiX_INCLUDE}
        ${CUDAToolkit_INCLUDE_DIRS}
        ${_CHRONO_CUDA_CCCL_INCLUDE_DIRS}
        ${CMAKE_INSTALL_PREFIX}/include
        ${CMAKE_SOURCE_DIR}/src
        CACHE STRING "NVRTC include dirs as list." FORCE)
    mark_as_advanced(CUDA_NVRTC_INCLUDE_DIRS)
  
    set(CUDA_NVRTC_INCLUDE_LIST)
    foreach(item ${CUDA_NVRTC_INCLUDE_DIRS})
      set(CUDA_NVRTC_INCLUDE_LIST "${CUDA_NVRTC_INCLUDE_LIST} \\\n  \"${item}\",")
    endforeach()
    set(CUDA_NVRTC_INCLUDE_LIST "${CUDA_NVRTC_INCLUDE_LIST} \\\n  0,")
  
    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVRTC>)
   
    # Install the shader sources
    install(FILES ${Chrono_sensor_RT_SOURCES}
            DESTINATION include/chrono_sensor/optix/shaders)

    # Stage the *.cu shader sources into the build tree as a BUILD step.
    #
    # NVRTC compiles these at runtime by reading them from CH_BUILD_SENSOR_SHADERS, so the
    # staged copies must track the sources. Staging them at configure time instead would mean
    # that editing a shader and re-running only the build keeps using the previous version,
    # with no error and no warning.
    #
    # One rule per shader, each with a real OUTPUT and DEPENDS, so a copy runs only when that
    # shader changes and an otherwise unchanged tree still reports "no work to do". A bare
    # add_custom_target would be phony, hence always out of date, and would re-run every copy on
    # every build. (Chrono_sensor_ptx_copy in the NVCC branch below is such a target; it can be
    # because its inputs are build artifacts rather than sources.)
    #
    # Paths in Chrono_sensor_RT_SOURCES are relative to CMAKE_CURRENT_SOURCE_DIR, while a custom
    # command runs in the build directory, so they are made absolute here.
    set(Chrono_sensor_RT_STAGED "")
    set(_staged_shader_names "")
    foreach(_shader ${Chrono_sensor_RT_SOURCES})
      get_filename_component(_shader_name "${_shader}" NAME)
      list(APPEND _staged_shader_names "${_shader_name}")
      # The trailing touch keeps the declared OUTPUT no older than its DEPENDS even when
      # copy_if_different decides the bytes are unchanged, as happens when a source is touched
      # but not edited. Without it, a generator that decides staleness purely by comparing
      # timestamps would find this rule dirty on every subsequent build. The Ninja generator does
      # not need it, because CMake emits "restat = 1" for custom commands and Ninja then treats an
      # unchanged output as clean; the touch makes the rule correct without relying on that.
      add_custom_command(
        OUTPUT  "${CH_BUILD_SENSOR_SHADERS}/${_shader_name}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/${_shader}"
                "${CH_BUILD_SENSOR_SHADERS}/${_shader_name}"
        COMMAND ${CMAKE_COMMAND} -E touch "${CH_BUILD_SENSOR_SHADERS}/${_shader_name}"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${_shader}"
        COMMENT "Staging Chrono::Sensor shader ${_shader_name}"
        VERBATIM
      )
      list(APPEND Chrono_sensor_RT_STAGED "${CH_BUILD_SENSOR_SHADERS}/${_shader_name}")
    endforeach()

    add_custom_target(Chrono_sensor_shader_copy ALL DEPENDS ${Chrono_sensor_RT_STAGED})

    # ALL by itself is not sufficient: building a single target, for example one demo or one
    # unit test, would skip the copy and run against a stale shader. Making the library depend
    # on the copy means anything that links Chrono_sensor gets current shaders.
    add_dependencies(Chrono_sensor Chrono_sensor_shader_copy)

    # Exported for utest_SEN_shader_staging, which asserts that every staged copy still matches its
    # source. The list comes from the same variable the staging rules above are generated from, so
    # the test cannot silently disagree with the build about which shaders exist, and it includes the
    # NVDB shader whenever that configuration adds one.
    set(CH_SENSOR_STAGED_SHADER_NAMES "${_staged_shader_names}"
        CACHE INTERNAL "Names of the OptiX shader files staged into the build tree")
    set(CH_SENSOR_SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/optix/shaders"
        CACHE INTERNAL "Directory holding the OptiX shader sources in the repository")
    set(CH_SENSOR_STAGED_SHADER_DIR "${CH_BUILD_SENSOR_SHADERS}"
        CACHE INTERNAL "Directory the runtime reads OptiX shaders from")
  else()
    message(STATUS "  Building Chrono::Sensor with NVRTC OFF")
    
    # Create helper PTX library
    add_library(Chrono_sensor_ptx OBJECT ${Chrono_sensor_RT_SOURCES} ${Chrono_sensor_RT_HEADERS})
    set_target_properties(Chrono_sensor_ptx PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})
    target_include_directories(Chrono_sensor_ptx PRIVATE ${CMAKE_SOURCE_DIR}/src)
    set_property(TARGET Chrono_sensor_ptx PROPERTY CUDA_PTX_COMPILATION ON)
    target_include_directories(Chrono_sensor_ptx PRIVATE "${OptiX_INCLUDE}")
    target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::cudart_static)
    target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::nppc)
    target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::nppig)
    target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::nppidei)
    if(CH_USE_SENSOR_NVDB)
      target_link_libraries(Chrono_sensor_ptx PRIVATE OpenVDB::openvdb)
      target_link_libraries(Chrono_sensor_ptx PRIVATE OpenVDB::nanovdb)
    endif()
    
    # Copy PTX files to known location
    # Notes:
    #   1. We cannot install the PTX files since these are "objects" not known at configuration time.
    #   2. By design, CMake does not allow redirecting output of object files. So we cannot set output properties on the Chrono_sensor_ptx target.
    #   3. A POST_BUILD command cannot be associated with an OBJECT library target. So we cannot do this after Chrono_sensor_ptx.
    #  The solution is to create a custom target to copy the ptx files. 
    add_custom_target(Chrono_sensor_ptx_copy ALL
      COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_OBJECTS:Chrono_sensor_ptx> "${CH_BUILD_SENSOR_SHADERS}"
      COMMAND_EXPAND_LISTS
      COMMENT "Copy Chrono::Sensor PTX files."
    )
    
    # Link PTX helper library, use NVCC, and set macro with location of PTX files
    add_dependencies(Chrono_sensor_ptx_copy Chrono_sensor_ptx)
    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVCC>)
  endif()

  # Install the shaders directory
  install(DIRECTORY ${CH_BUILD_SENSOR_SHADERS}/ DESTINATION ${CH_INSTALL_SENSOR_SHADERS})

endif() # CH_USE_SENSOR_OPTIX

if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
  install(FILES ${Chrono_sensor_VULKAN_SPV}
          DESTINATION ${CH_INSTALL_SENSOR_SHADERS}/vulkan)
endif()

# ------------------------------------------------------------------------------
# Find optional GL support
# ------------------------------------------------------------------------------

message(STATUS "Looking for optional OpenGL support")

find_package(OpenGL QUIET)
find_package(GLEW QUIET)
find_package(glfw3 QUIET)

message(STATUS "  OpenGL found: ${OPENGL_FOUND}")
message(STATUS "  GLEW found:   ${GLEW_FOUND}")
message(STATUS "  glfw3 found:  ${glfw3_FOUND}")

if(OPENGL_FOUND AND GLEW_FOUND AND glfw3_FOUND)
  message(STATUS "  GL libraries found")
  message(STATUS "    OpenGL include dir: ${OPENGL_INCLUDE_DIR}")
  message(STATUS "    OpenGL libraries:   ${OPENGL_LIBRARIES}")
  message(STATUS "    GLEW include dir:   ${GLEW_INCLUDE_DIRS}")
  message(STATUS "    GLEW libraries:     ${GLEW_LIBRARIES}")

  target_compile_definitions(Chrono_sensor PUBLIC $<$<COMPILE_LANGUAGE:CXX>:USE_SENSOR_GLFW>)
  target_link_libraries(Chrono_sensor PUBLIC GLEW::glew)
  target_link_libraries(Chrono_sensor PUBLIC glfw)
  target_link_libraries(Chrono_sensor PUBLIC OpenGL::GL)
else()
  message(STATUS "  GL libraries not found; Chrono::Sensor OpenGL support disabled")
endif()

if(CH_USE_SENSOR_OPTIX AND CH_USE_SENSOR_NVDB)
  # CMake Module path
  set(OpenVDB_DIR "" CACHE PATH "OpenVDB CMake Module Path")
  set(CMAKE_MODULE_PATH ${OpenVDB_DIR} ${CMAKE_MODULE_PATH} )
  # Print out the CMake module path
  message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
  # print cmake cxx standard
  message(STATUS "CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}")
  find_package(OpenVDB REQUIRED COMPONENTS nanovdb)

  target_compile_definitions(Chrono_sensor PUBLIC USE_SENSOR_NVDB)
  message(STATUS "NANO VDB Visualization enabled in Chrono::Sensor.")
endif()

if(MSVC)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)               # missing DLL interface
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4100>)               # unreferenced formal parameter
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4661>)               # template member not defined
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4005>)               # macro defined twice
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4251>)   # missing DLL interface (forward to host compiler for .cu files)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4100>)   # unreferenced formal parameter (forward to host compiler for .cu files)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4661>)   # template member not defined (forward to host compiler for .cu files)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4005>)   # macro defined twice (forward to host compiler for .cu files)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--diag-suppress=1394>) # missing DLL interface (matches /wd4251 for CXX)

  # nvcc places the host-side launch stub for a __global__ kernel in a generated
  # <tu>.cudafe1.stub.c, and when the kernel lives in an anonymous namespace it declares that
  # stub without "static" in the .cudafe1.cpp but defines it "static" with a qualified name in
  # the stub. MSVC accepts the combination only as a nonstandard extension, hence C4211. The
  # construct is in nvcc-generated code, not in Chrono sources (see fsi_sph_render.cu), so
  # there is nothing to fix here and no include-directory mechanism can reach a file that nvcc
  # writes into the temp directory.
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4211>)   # redefined extern to static (nvcc launch stub)

  # Treat headers included via <...> (CUDA Toolkit, OptiX, standard library, etc.) as external
  # and silence warnings originating from them, e.g. the double4/double4_16a/double4_32a
  # deprecation warning raised from NVIDIA's own curand_poisson.h. Mirrors the SYSTEM include
  # directories declared above, which give the equivalent -isystem treatment on GCC/Clang.
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/external:W0>)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/external:anglebrackets>)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/external:W0>)
  target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/external:anglebrackets>)
endif()

target_compile_definitions(Chrono_sensor PUBLIC $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_SENSOR>)

target_link_libraries(Chrono_sensor PRIVATE Chrono_core ${LIBRARIES})

install(TARGETS Chrono_sensor
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_sensor)

# ----------------------------------------------------------------------------
# Generate and install configuration files
# ----------------------------------------------------------------------------

message(STATUS "Create Chrono::Sensor configuration file")

if(CH_USE_SENSOR_OPTIX)

  # For the build tree
  set(SHADER_DIR ${CH_BUILD_SENSOR_SHADERS})
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    set(VULKAN_SHADER_DIR "${CH_BUILD_SENSOR_SHADERS}/vulkan")
  else()
    set(VULKAN_SHADER_DIR "")
  endif()
  message(STATUS "   build tree:   SHADER_DIR = ${SHADER_DIR}")
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    message(STATUS "   build tree:   VULKAN_SHADER_DIR = ${VULKAN_SHADER_DIR}")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in ${Chrono_sensor_CONFIG_FILE} @ONLY)
  
  # For the install tree
  set(SHADER_DIR "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_SENSOR_SHADERS}")
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    set(VULKAN_SHADER_DIR "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_SENSOR_SHADERS}/vulkan")
  else()
    set(VULKAN_SHADER_DIR "")
  endif()
  message(STATUS "   install tree: SHADER_DIR = ${SHADER_DIR}")
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    message(STATUS "   install tree: VULKAN_SHADER_DIR = ${VULKAN_SHADER_DIR}")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in
      ${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor-install.h @ONLY)

else()

  set(SHADER_DIR "")
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    set(VULKAN_SHADER_DIR "${CH_BUILD_SENSOR_SHADERS}/vulkan")
  else()
    set(VULKAN_SHADER_DIR "")
  endif()
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    message(STATUS "   build tree:   VULKAN_SHADER_DIR = ${VULKAN_SHADER_DIR}")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in ${Chrono_sensor_CONFIG_FILE} @ONLY)

  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    set(VULKAN_SHADER_DIR "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_SENSOR_SHADERS}/vulkan")
  else()
    set(VULKAN_SHADER_DIR "")
  endif()
  if(CH_USE_SENSOR_VULKAN_RT AND CH_USE_SENSOR_VULKAN_RT_GPU)
    message(STATUS "   install tree: VULKAN_SHADER_DIR = ${VULKAN_SHADER_DIR}")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in
      ${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor-install.h @ONLY)

endif()

# Install configuration file
install(FILES "${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor-install.h"
        RENAME "ChConfigSensor.h"
        DESTINATION include/chrono_sensor)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_sensor_HEADERS}         DESTINATION include/chrono_sensor)
install(FILES ${Chrono_sensor_SENSORS_HEADERS} DESTINATION include/chrono_sensor/sensors)
install(FILES ${Chrono_sensor_UTILS_HEADERS}   DESTINATION include/chrono_sensor/utils)
install(FILES ${Chrono_sensor_OPTIX_HEADERS}   DESTINATION include/chrono_sensor/optix)
install(FILES ${Chrono_sensor_VULKAN_HEADERS}  DESTINATION include/chrono_sensor/vulkan)
install(FILES ${Chrono_sensor_FILTERS_HEADERS} DESTINATION include/chrono_sensor/filters)
install(FILES ${Chrono_sensor_CUDA_HEADERS}    DESTINATION include/chrono_sensor/cuda)
install(FILES ${Chrono_sensor_RT_HEADERS}      DESTINATION include/chrono_sensor/optix/shaders)

# Copy and install FindOptiX.cmake
if(CH_USE_SENSOR_OPTIX)
  file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindOptiX.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
  install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindOptiX.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})
endif()

# Install 3rd party headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

# On Windows, extract DLLs from targets and install (always look for Release DLLs)
#if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#    get_target_property(GLEW_DLL GLEW::glew IMPORTED_LOCATION_RELEASE)
#    get_target_property(GLFW_DLL glfw IMPORTED_LOCATION_RELEASE)
#    if(EXISTS "${GLEW_DLL}")
#      install(FILES "${GLEW_DLL}" DESTINATION bin)
#    endif()
#    if(EXISTS "${GLFW_DLL}")
#      install(FILES "${GLFW_DLL}" DESTINATION bin)
#    endif()
#endif()
