#=============================================================================
# CMake configuration file for Chrono Sensor module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_SENSOR "Enable the Chrono Sensor module" OFF)

if(NOT ENABLE_MODULE_SENSOR)
  return()
endif()

message(STATUS "\n==== Chrono Sensor module ====\n")

# Return now if CUDA is not available
if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "Chrono::Sensor requires CUDA, but CUDA was not found; disabling Chrono::Sensor")

  mark_as_advanced(FORCE GLM_INCLUDE_DIR)
  mark_as_advanced(FORCE GLEW_DIR)
  mark_as_advanced(FORCE glfw3_DIR)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    mark_as_advanced(FORCE GLEW_DLL)
    mark_as_advanced(FORCE GLFW_DLL)
  endif()

  set(ENABLE_MODULE_SENSOR OFF CACHE BOOL "Enable the Chrono Sensor module" FORCE)
  return()
endif()

mark_as_advanced(CLEAR GLM_INCLUDE_DIR)
mark_as_advanced(CLEAR GLEW_DIR)
mark_as_advanced(CLEAR glfw3_DIR)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  mark_as_advanced(CLEAR GLEW_DLL)
  mark_as_advanced(CLEAR GLFW_DLL)
endif()

option(USE_NVDB "Enable Chrono::FSI deformable terrain sim rendering in Chrono::Sensor" OFF)


#-----------------------------------------------------------------------------
# LIST CUDA FILES USED FOR RT PROGRAMS - TO BE COMPILED TO PTX SHADERS
#-----------------------------------------------------------------------------

set(ChronoEngine_sensor_RT_SOURCES
    optix/shaders/box.cu
    optix/shaders/sphere.cu
    optix/shaders/cylinder.cu
    optix/shaders/camera.cu
    optix/shaders/lidar.cu
    optix/shaders/miss.cu
    optix/shaders/material_shaders.cu
    optix/shaders/radar.cu
)

if(USE_NVDB)
  list(APPEND ChronoEngine_sensor_RT_SOURCES
    optix/shaders/nvdb_vol_intersect.cu
  )
endif()

set(ChronoEngine_sensor_RT_HEADERS
    optix/shaders/device_utils.h
)

source_group("RT Programs" FILES
    ${ChronoEngine_sensor_RT_SOURCES}
	${ChronoEngine_sensor_RT_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST CUDA FILES THAT ARE TO BE COMPILED AS SOURCE
#-----------------------------------------------------------------------------

set(ChronoEngine_sensor_CUDA_SOURCES
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
)

set(ChronoEngine_sensor_CUDA_HEADERS
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
)

source_group("Cuda" FILES
    ${ChronoEngine_sensor_CUDA_SOURCES}
	${ChronoEngine_sensor_CUDA_HEADERS}
)


#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------


set(ChronoEngine_sensor_SOURCES
    ChSensorManager.cpp
    ChDynamicsManager.cpp
)

set(ChronoEngine_sensor_HEADERS
  	ChApiSensor.h
    ChSensorManager.h
    ChDynamicsManager.h
)

source_group("Source" FILES
    ${ChronoEngine_sensor_SOURCES}
  	${ChronoEngine_sensor_HEADERS}
)

set(ChronoEngine_sensor_SENSORS_SOURCES
    sensors/ChSensor.cpp
    sensors/ChNoiseModel.cpp
    sensors/ChOptixSensor.cpp
    sensors/ChCameraSensor.cpp
    sensors/ChSegmentationCamera.cpp
    sensors/ChDepthCamera.cpp
    sensors/ChLidarSensor.cpp
    sensors/ChRadarSensor.cpp
    sensors/ChIMUSensor.cpp
    sensors/ChGPSSensor.cpp
    sensors/ChTachometerSensor.cpp
    sensors/Sensor.cpp
)

set(ChronoEngine_sensor_SENSORS_HEADERS
    sensors/ChSensor.h
    sensors/ChNoiseModel.h
    sensors/ChOptixSensor.h
    sensors/ChCameraSensor.h
    sensors/ChSegmentationCamera.h
    sensors/ChDepthCamera.h
    sensors/ChLidarSensor.h
    sensors/ChRadarSensor.h
    sensors/ChIMUSensor.h
    sensors/ChGPSSensor.h
    sensors/ChTachometerSensor.h
  	sensors/ChSensorBuffer.h
    sensors/Sensor.h
)

source_group("Source" FILES
    ${ChronoEngine_sensor_SENSORS_SOURCES}
  	${ChronoEngine_sensor_SENSORS_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------

set(ChronoEngine_sensor_SCENE_SOURCES
    optix/scene/ChScene.cpp
)

set(ChronoEngine_sensor_SCENE_HEADERS
    optix/scene/ChScene.h
)

source_group("Scene" FILES
    ${ChronoEngine_sensor_SCENE_SOURCES}
  	${ChronoEngine_sensor_SCENE_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR OPTIX LIBRARY
#-----------------------------------------------------------------------------

set(ChronoEngine_sensor_OPTIX_SOURCES
    optix/ChOptixEngine.cpp
    optix/ChOptixGeometry.cpp
    optix/ChOptixPipeline.cpp
    optix/ChOptixUtils.cpp
    optix/ChFilterOptixRender.cpp
    optix/ChNVDBVolume.cpp
)

set(ChronoEngine_sensor_OPTIX_HEADERS
    optix/ChOptixEngine.h
    optix/ChOptixGeometry.h
    optix/ChOptixPipeline.h
    optix/ChOptixUtils.h
    optix/ChFilterOptixRender.h
    optix/ChOptixDefinitions.h
    optix/ChNVDBVolume.h
)

source_group("Optix" FILES
    ${ChronoEngine_sensor_OPTIX_SOURCES}
  	${ChronoEngine_sensor_OPTIX_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------

set(ChronoEngine_sensor_FILTERS_SOURCES
  	filters/ChFilter.cpp
  	filters/ChFilterIMUUpdate.cpp
  	filters/ChFilterGPSUpdate.cpp
    filters/ChFilterCameraNoise.cpp
    filters/ChFilterLidarNoise.cpp
  	filters/ChFilterVisualize.cpp
    filters/ChFilterSave.cpp
    filters/ChFilterSavePtCloud.cpp
  	filters/ChFilterGrayscale.cpp
    filters/ChFilterLidarReduce.cpp
  	filters/ChFilterAccess.cpp
    filters/ChFilterPCfromDepth.cpp
    filters/ChFilterVisualizePointCloud.cpp
    filters/ChFilterImageOps.cpp
    filters/ChFilterLidarIntensityClip.cpp
    filters/ChFilterRadarProcess.cpp
    filters/ChFilterRadarXYZReturn.cpp
    filters/ChFilterRadarSavePC.cpp
    filters/ChFilterRadarXYZVisualize.cpp
    filters/ChFilterRadarVisualizeCluster.cpp
    filters/ChFilterTachometerUpdate.cpp
)

set(ChronoEngine_sensor_FILTERS_HEADERS
  	filters/ChFilter.h
    filters/ChFilterIMUUpdate.h
  	filters/ChFilterGPSUpdate.h
    filters/ChFilterCameraNoise.h
    filters/ChFilterLidarNoise.h
  	filters/ChFilterVisualize.h
    filters/ChFilterSave.h
    filters/ChFilterSavePtCloud.h
  	filters/ChFilterGrayscale.h
    filters/ChFilterLidarReduce.h
  	filters/ChFilterAccess.h
    filters/ChFilterPCfromDepth.h
    filters/ChFilterVisualizePointCloud.h
    filters/ChFilterImageOps.h
    filters/ChFilterLidarIntensityClip.h
    filters/ChFilterRadarProcess.h
    filters/ChFilterRadarXYZReturn.h
    filters/ChFilterRadarSavePC.h
    filters/ChFilterRadarXYZVisualize.h
    filters/ChFilterRadarVisualizeCluster.h
    filters/ChFilterTachometerUpdate.h
)

source_group("Filters" FILES
    ${ChronoEngine_sensor_FILTERS_SOURCES}
  	${ChronoEngine_sensor_FILTERS_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------

if(USE_TENSOR_RT)
    set(ChronoEngine_sensor_TENSORRT_SOURCES
      	tensorrt/ChFilterUFF.cpp
        tensorrt/ChFilterONNX.cpp
    )
    set(ChronoEngine_sensor_TENSORRT_HEADERS
      	tensorrt/ChFilterUFF.h
        tensorrt/ChFilterONNX.h
        tensorrt/ChTRTUtils.h
    )

    source_group("TensorRT" FILES
        ${ChronoEngine_sensor_TENSORRT_SOURCES}
      	${ChronoEngine_sensor_TENSORRT_HEADERS}
    )
endif()


#-----------------------------------------------------------------------------
# LIST THE UTILITY FILES THAT WILL BE EXPOSED TO THE USER
#-----------------------------------------------------------------------------

set(ChronoEngine_sensor_UTILS_SOURCES
    utils/ChUtilsJSON.cpp
    utils/ChGPSUtils.cpp
    utils/Kdtree.cpp
    utils/Dbscan.cpp
)

set(ChronoEngine_sensor_UTILS_HEADERS
  	utils/CudaMallocHelper.h
    utils/ChUtilsJSON.h
    utils/ChGPSUtils.h
    utils/Kdtree.h
    utils/Dbscan.h
)

source_group(Utils FILES
    ${ChronoEngine_sensor_UTILS_SOURCES}
  	${ChronoEngine_sensor_UTILS_HEADERS}
)

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

#-----------------------------------------------------------------------------
# Create the ChronoEngine_sensor library
#-----------------------------------------------------------------------------
# Collect all sources
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_SENSORS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_SENSORS_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_UTILS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_UTILS_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_OPTIX_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_OPTIX_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_FILTERS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_FILTERS_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_SCENE_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_SCENE_HEADERS})

list(APPEND ALL_CH_SENSOR_FILES ${SENSOR_STB_FILES})
list(APPEND ALL_CH_SENSOR_FILES ${SENSOR_TINYOBJ_FILES})

if(USE_TENSOR_RT)
    list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_TENSORRT_SOURCES})
    list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_TENSORRT_HEADERS})
endif()

add_library(ChronoEngine_sensor ${ALL_CH_SENSOR_FILES}
                                ${ChronoEngine_sensor_CUDA_SOURCES})

add_library(ChronoEngine::ChronoEngine_sensor ALIAS ChronoEngine_sensor)


# ------------------------------------------------------------------------------
# Find and set everything needed for OptiX
# ------------------------------------------------------------------------------

find_package(OptiX REQUIRED)

message(STATUS "OptiX include directory: ${OptiX_INCLUDE_DIR}")

file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindOptiX.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)

install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindOptiX.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
        )

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  target_compile_definitions(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:NOMINMAX>)
endif()

target_link_libraries(ChronoEngine_sensor PUBLIC OptiX::OptiX) # needs to be exposed to consumers
target_link_libraries(ChronoEngine_sensor PRIVATE CUDA::cudart_static)
target_link_libraries(ChronoEngine_sensor PRIVATE CUDA::nppc)
target_link_libraries(ChronoEngine_sensor PRIVATE CUDA::nppig)
target_link_libraries(ChronoEngine_sensor PRIVATE CUDA::nppidei)

if(USE_NVDB)
  target_link_libraries(ChronoEngine_sensor PRIVATE OpenVDB::openvdb)
  target_link_libraries(ChronoEngine_sensor PRIVATE OpenVDB::nanovdb)
endif()

# print cxx compiler flags\
## TODO: DARIOM need to be passed to the host compiler?
# if(MSVC AND MSVC_VERSION GREATER_EQUAL 1900)
#   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
# endif()

target_compile_options(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)
target_compile_options(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)


# ------------------------------------------------------------------------------
# Optionally use NVRTC to compile shader code rather than NVCC to PTX
# ------------------------------------------------------------------------------

set(USE_CUDA_NVRTC ON CACHE BOOL "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX")
if(USE_CUDA_NVRTC)
  find_package(CUDAToolkit REQUIRED)

  if(NOT TARGET CUDA::nvrtc)
    message(WARNING "CUDA::nvrtc target is missing; NVRTC will be turned off.")
    set(USE_CUDA_NVRTC OFF CACHE BOOL "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX" FORCE)
  else()
    message(STATUS "CUDA::nvrtc found and enabled.")
    target_compile_definitions(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVRTC>)

    target_link_libraries(ChronoEngine_sensor PRIVATE CUDA::nvrtc)

    install(FILES ${ChronoEngine_sensor_RT_SOURCES}
            DESTINATION include/chrono_sensor/optix/shaders)  


    ##
    # Flags used directly in the source code as defines
    ##

    #set(CUDA_NVRTC_FLAGS -arch compute_30 -use_fast_math -lineinfo -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    set(CUDA_NVRTC_FLAGS -use_fast_math -std=c++11 -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    mark_as_advanced(CUDA_NVRTC_FLAGS)

    set(CUDA_NVRTC_FLAG_LIST)
    foreach(item ${CUDA_NVRTC_FLAGS}) #CUDA_NVRTC_FLAGS CUDA_NVCC_FLAGS
      set(CUDA_NVRTC_FLAG_LIST "${CUDA_NVRTC_FLAG_LIST} \\\n  \"${item}\",")
    endforeach()
    set(CUDA_NVRTC_FLAG_LIST "${CUDA_NVRTC_FLAG_LIST} \\\n  0,")

    set(CUDA_NVRTC_INCLUDE_DIRS
      ${OptiX_INCLUDE_DIR}
      ${OptiX_INCLUDE_DIR}/optixu
      ${CUDAToolkit_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}/optix/shaders
      ${CMAKE_INSTALL_PREFIX}/include
      ${CMAKE_SOURCE_DIR}/src CACHE STRING "NVRTC include dirs as list." FORCE)
    mark_as_advanced(CUDA_NVRTC_INCLUDE_DIRS)

    set(CUDA_NVRTC_INCLUDE_LIST)
    foreach(item ${CUDA_NVRTC_INCLUDE_DIRS})
      set(CUDA_NVRTC_INCLUDE_LIST "${CUDA_NVRTC_INCLUDE_LIST} \\\n  \"${item}\",")
    endforeach()
    set(CUDA_NVRTC_INCLUDE_LIST "${CUDA_NVRTC_INCLUDE_LIST} \\\n  0,")

    target_compile_definitions(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVRTC>)
    target_compile_definitions(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:SHADER_OUTPUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/optix/shaders/">)

  endif()
else()
  # Generate the PTX files only if NVRTC is disabled
  add_library(PTX_TARGET ${ChronoEngine_sensor_RT_SOURCES} )
  set_target_properties(PTX_TARGET PROPERTY CUDA_PTX_COMPILATION ON)
  set_target_properties(PTX_TARGET PROPERTY LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/sensor_ptx/")

  set(PTX_TARGET_GENERATED_FILES $<TARGET_OBJECTS:PTX_TARGET>)
  source_group("Generated Files" FILES ${PTX_TARGET_GENERATED_FILES})
  target_link_libraries(ChronoEngine_sensor PRIVATE ${PTX_TARGET_GENERATED_FILES})

  target_compile_definitions(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVCC>)

  target_compile_definitions(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:SHADER_OUTPUT_PATH="${CMAKE_BINARY_DIR}/lib/sensor_ptx/">)

  install(FILES ${PTX_TARGET_GENERATED_FILES}
          DESTINATION lib/sensor_ptx)

endif()


# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in
    ${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h"
    DESTINATION include/chrono_sensor)


# ------------------------------------------------------------------------------
# Find optional GL support
# ------------------------------------------------------------------------------

find_package(OpenGL QUIET)
find_package(GLEW QUIET)
find_package(glfw3 QUIET)

message(STATUS "OpenGL found: ${OPENGL_FOUND}")
message(STATUS "GLEW found:   ${GLEW_FOUND}")
message(STATUS "GLFW3 found:  ${glfw3_FOUND}")

# On windows, ask for the GLEW and GLFW DLLs so that we can copy. This is
# optional. If not specified, it is the user's responsibility to make them
# available at runtime.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(GLEW_DLL "${GLEW_DIR}/../../../bin/glew32d.dll" CACHE FILEPATH "Path to GLEW DLL")
  set(GLFW_DLL "${glfw3_DIR}/../../../bin/glfw3.dll;${glfw3_DIR}/../../../bin/glfw3_d.dll" CACHE FILEPATH "Path to GLFW DLL")
endif()

if(OPENGL_FOUND AND GLEW_FOUND AND glfw3_FOUND)
  target_compile_definitions(ChronoEngine_sensor PUBLIC $<$<COMPILE_LANGUAGE:CXX>:USE_SENSOR_GLFW>)
  target_link_libraries(ChronoEngine_sensor PUBLIC OpenGL::GL)
  target_link_libraries(ChronoEngine_sensor PUBLIC GLEW::glew)
  target_link_libraries(ChronoEngine_sensor PUBLIC glfw)
  message(STATUS "GL libraries found.")
else()
  message(STATUS "GL libraries not found. OpenGL support disabled.")
endif()


if(USE_NVDB)
  # CMake Module path
  set(OpenVDB_DIR "" CACHE PATH "OpenVDB CMake Module Path")
  set(CMAKE_MODULE_PATH ${OpenVDB_DIR} ${CMAKE_MODULE_PATH} )
  # Print out the CMake module path
  message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
  # print cmake cxx standard
  message(STATUS "CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}")
  find_package(OpenVDB REQUIRED COMPONENTS nanovdb)

  target_compile_definitions(ChronoEngine_sensor PUBLIC USE_SENSOR_NVDB)
  message(STATUS "NANO VDB Visualization enabled in Chrono::Sensor.")
endif()


# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_options(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4661>)
    target_compile_options(ChronoEngine_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4005>)
endif()


target_compile_definitions(ChronoEngine_sensor PUBLIC $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_SENSOR>)

target_link_libraries(ChronoEngine_sensor PRIVATE ChronoEngine ${LIBRARIES})



# ------------------------------------------------------------------------------
# Optionally find TensorRT Version___ and set libaries and includes
# ------------------------------------------------------------------------------

option(USE_TENSOR_RT "Enable the TensorRT for Sensor Module" OFF)

IF(USE_TENSOR_RT)

    set(TENSOR_RT_INSTALL_DIR "" CACHE PATH "Path to TensorRT")

    #TensorRT Libraries
    find_library(TENSOR_RT_NVINFER nvinfer ${TENSOR_RT_INSTALL_DIR}/lib)
    find_library(TENSOR_RT_PARSERS nvparsers ${TENSOR_RT_INSTALL_DIR}/lib)
    find_library(TENSOR_RT_ONNXPARSER nvonnxparser ${TENSOR_RT_INSTALL_DIR}/lib)

    find_path(TENSOR_RT_INCLUDE_PATH NAMES NvInfer.h PATHS ${TENSOR_RT_INSTALL_DIR}/include)

    target_include_directories(ChronoEngine_sensor PRIVATE ${TENSOR_RT_INCLUDE_PATH})
    target_link_libraries(ChronoEngine_sensor PRIVATE ${TENSOR_RT_NVINFER})
    target_link_libraries(ChronoEngine_sensor PRIVATE ${TENSOR_RT_PARSERS})
    target_link_libraries(ChronoEngine_sensor PRIVATE ${TENSOR_RT_ONNXPARSER})

    mark_as_advanced(TENSOR_RT_NVINFER)
    mark_as_advanced(TENSOR_RT_PARSERS)
    mark_as_advanced(TENSOR_RT_ONNXPARSER)

ENDIF()

#-------------------------------------------------------------------------------
# Install the ChronoEngine_sensor library
#-------------------------------------------------------------------------------

install(TARGETS ChronoEngine_sensor
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_sensor)

install(FILES ${ChronoEngine_sensor_HEADERS}
		DESTINATION include/chrono_sensor)
install(FILES ${ChronoEngine_sensor_SENSORS_HEADERS}
		DESTINATION include/chrono_sensor/sensors)
install(FILES ${ChronoEngine_sensor_UTILS_HEADERS}
		DESTINATION include/chrono_sensor/utils)
install(FILES ${ChronoEngine_sensor_OPTIX_HEADERS}
        DESTINATION include/chrono_sensor/optix)
install(FILES ${ChronoEngine_sensor_FILTERS_HEADERS}
        DESTINATION include/chrono_sensor/filters)
install(FILES ${ChronoEngine_sensor_CUDA_HEADERS}
      DESTINATION include/chrono_sensor/cuda)
install(FILES ${ChronoEngine_sensor_SCENE_HEADERS}
      DESTINATION include/chrono_sensor/optix/scene)
install(FILES ${ChronoEngine_sensor_RT_HEADERS}
      DESTINATION include/chrono_sensor/optix/shaders)
      

#-------------------------------------------------------------------------------
# On Windows, copy the GLEW and GLFW DLLs (if specified)
#-------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(EXISTS "${GLEW_DLL}")
      ADD_CUSTOM_COMMAND(TARGET ChronoEngine_sensor POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                                 "${GLEW_DLL}"
                                 "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${GLEW_DLL}" DESTINATION bin)
    endif()
    if(EXISTS "${GLFW_DLL}")
      ADD_CUSTOM_COMMAND(TARGET ChronoEngine_sensor POST_BUILD
                         COMMAND ${CMAKE_COMMAND} -E copy_if_different
                         "${GLFW_DLL}"
                         "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>")
      install(FILES "${GLFW_DLL}" DESTINATION bin)
    endif()
endif()
