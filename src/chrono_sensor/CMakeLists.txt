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
if(NOT CUDA_FOUND)
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
# optional.  If not specified, it is the user's responsibility to make them
# available at runtime.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(GLEW_DLL  "" CACHE FILEPATH "The GLEW DLL")
    SET(GLFW_DLL  "" CACHE FILEPATH "The GLFW DLL")
endif()

if(${OPENGL_FOUND} AND ${GLEW_FOUND} AND ${glfw3_FOUND})
  set(HAVE_GL TRUE)
  message(STATUS "GL libraries found.")
else()
  set(HAVE_GL FALSE)
  message(STATUS "GL libraries not found. OpenGL support disabled.")
endif()

# ------------------------------------------------------------------------------
# Find and set everything needed for OptiX
# ------------------------------------------------------------------------------

find_package(OptiX REQUIRED)

message(STATUS "OptiX include directory: ${OptiX_INCLUDE}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_compile_definitions(NOMINMAX)
	message(STATUS "NOMINMAX set for windows")
endif()

set(CH_SENSOR_INCLUDES ${CH_SENSOR_INCLUDES}
    ${OptiX_INCLUDE}
    ${CUDA_INCLUDE_DIRS}
)

list(APPEND LIBRARIES ${CUDA_nppc_LIBRARY})
list(APPEND LIBRARIES ${CUDA_nppig_LIBRARY})
list(APPEND LIBRARIES ${CUDA_nppidei_LIBRARY})

set(CH_SENSOR_INCLUDES ${CH_SENSOR_INCLUDES} "${CUDA_TOOLKIT_ROOT_DIR}/include")
list(APPEND CUDA_NVCC_FLAGS "--use_fast_math")


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


    list(APPEND LIBRARIES ${TENSOR_RT_NVINFER})
    list(APPEND LIBRARIES ${TENSOR_RT_PARSERS})
    list(APPEND LIBRARIES ${TENSOR_RT_ONNXPARSER})

    mark_as_advanced(TENSOR_RT_NVINFER)
    mark_as_advanced(TENSOR_RT_PARSERS)
    mark_as_advanced(TENSOR_RT_ONNXPARSER)

    #TensorRT Include directory
    #set(TENSOR_RT_INCLUDE_PATH "${TENSOR_RT_INSTALL_DIR}/include" CACHE PATH "Path to TensorRT includes")
    set(CH_SENSOR_INCLUDES ${CH_SENSOR_INCLUDES} "${TENSOR_RT_INCLUDE_PATH}")
ENDIF()


# ------------------------------------------------------------------------------
# Optionally use NVRTC to compile shader code rather than NVCC to PTX
# ------------------------------------------------------------------------------

set(USE_CUDA_NVRTC ON CACHE BOOL "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX")
if(USE_CUDA_NVRTC)
  find_library(CUDA_NVRTC_LIBRARY nvrtc HINTS ${CUDA_TOOLKIT_ROOT_DIR}
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64 
    ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64)
  if(NOT CUDA_NVRTC_LIBRARY)
    message(STATUS "USE_CUDA_NVRTC disabled due to missing CUDA_NVRTC_LIBRARY")
    set(USE_CUDA_NVRTC OFF)
  else()
    list(APPEND LIBRARIES ${CUDA_NVRTC_LIBRARY})
    # add_definitions( -DUSE_CUDA_NVRTC )
    # add_definitions( -DCMAKE_SHADER_OUTPUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/optix/shaders/" )

    #set(CUDA_NVRTC_FLAGS -arch compute_30 -use_fast_math -lineinfo -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    set(CUDA_NVRTC_FLAGS -use_fast_math -std=c++11 -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    mark_as_advanced(CUDA_NVRTC_FLAGS)

    set(CUDA_NVRTC_FLAG_LIST)
    foreach(item ${CUDA_NVRTC_FLAGS}) #CUDA_NVRTC_FLAGS CUDA_NVCC_FLAGS
      set(CUDA_NVRTC_FLAG_LIST "${CUDA_NVRTC_FLAG_LIST} \\\n  \"${item}\",")
    endforeach()
    set(CUDA_NVRTC_FLAG_LIST "${CUDA_NVRTC_FLAG_LIST} \\\n  0,")

    set(CUDA_NVRTC_INCLUDE_DIRS
      ${OptiX_INCLUDE}
      ${OptiX_INCLUDE}/optixu
      ${CUDA_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}/optix/shaders
      ${CMAKE_INSTALL_PREFIX}/include
      ${CMAKE_SOURCE_DIR}/src CACHE STRING "NVRTC include dirs as list." FORCE)
    mark_as_advanced(CUDA_NVRTC_INCLUDE_DIRS)

    set(CUDA_NVRTC_INCLUDE_LIST)
    foreach(item ${CUDA_NVRTC_INCLUDE_DIRS})
      set(CUDA_NVRTC_INCLUDE_LIST "${CUDA_NVRTC_INCLUDE_LIST} \\\n  \"${item}\",")
    endforeach()
    set(CUDA_NVRTC_INCLUDE_LIST "${CUDA_NVRTC_INCLUDE_LIST} \\\n  0,")

  endif()

endif()


# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in
		${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h"
		DESTINATION include/chrono_sensor)

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
)

set(ChronoEngine_sensor_OPTIX_HEADERS
    optix/ChOptixEngine.h
    optix/ChOptixGeometry.h
    optix/ChOptixPipeline.h
    optix/ChOptixUtils.h
    optix/ChFilterOptixRender.h
    optix/ChOptixDefinitions.h
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

# Generate the OBJ files
CUDA_WRAP_SRCS(ChronoEngine_sensor OBJ generated_obj_files ${ChronoEngine_sensor_CUDA_SOURCES} )

# Generate the PTX files only if NVRTC is disabled
if(NOT USE_CUDA_NVRTC)
  set(CUDA_GENERATED_OUTPUT_DIR "${CMAKE_BINARY_DIR}/lib/sensor_ptx")
  # add_definitions(-DCMAKE_SHADER_OUTPUT_PATH="${CUDA_GENERATED_OUTPUT_DIR}/")
  CUDA_WRAP_SRCS(ChronoEngine_sensor PTX generated_rt_files ${ChronoEngine_sensor_RT_SOURCES} )
  source_group("Generated Files" FILES ${generated_rt_files})
endif()

# reset the cuda generate directory for cuda files being compiled into obj
set(CUDA_GENERATED_OUTPUT_DIR "")

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

list(APPEND ALL_CH_SENSOR_FILES ${generated_obj_files})

if(NOT USE_CUDA_NVRTC)
    list(APPEND ALL_CH_SENSOR_FILES ${generated_rt_files})
endif()

if(USE_TENSOR_RT)
    list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_TENSORRT_SOURCES})
    list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_TENSORRT_HEADERS})
endif()

cuda_add_library(ChronoEngine_sensor ${ALL_CH_SENSOR_FILES})

if(USE_CUDA_NVRTC)
  target_compile_definitions(ChronoEngine_sensor PUBLIC -DUSE_CUDA_NVRTC )
  target_compile_definitions(ChronoEngine_sensor PUBLIC -DCMAKE_SHADER_OUTPUT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/optix/shaders/" )
else()
  target_compile_definitions(ChronoEngine_sensor PUBLIC -DCMAKE_SHADER_OUTPUT_PATH="${CMAKE_BINARY_DIR}/lib/sensor_ptx/")
endif()

if(${HAVE_GL})
  target_compile_definitions(ChronoEngine_sensor PUBLIC -DUSE_SENSOR_GLFW )
endif()

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")
endif()

target_compile_definitions(ChronoEngine_sensor PUBLIC CH_API_COMPILE_SENSOR)

set_target_properties(ChronoEngine_sensor PROPERTIES
                    COMPILE_FLAGS "${CH_CXX_FLAGS}"
                    LINK_FLAGS "${CH_LINKERFLAG_LIB}")

target_include_directories(ChronoEngine_sensor PUBLIC ${CH_SENSOR_INCLUDES})

target_link_libraries(ChronoEngine_sensor ChronoEngine ${LIBRARIES})
if(${HAVE_GL})
  target_link_libraries(ChronoEngine_sensor ${OPENGL_LIBRARIES})
  target_link_libraries(ChronoEngine_sensor GLEW::glew)
  target_link_libraries(ChronoEngine_sensor glfw)
endif()

# Make some variables available to users
set(CH_SENSOR_INCLUDES  "${CH_SENSOR_INCLUDES}"  PARENT_SCOPE)
SET(SENSOR_LIBRARIES    "${LIBRARIES}"           PARENT_SCOPE)
set(CH_SENSOR_CXX_FLAGS "${CH_SENSOR_CXX_FLAGS}" PARENT_SCOPE)
set(CH_SENSOR_C_FLAGS   "${CH_SENSOR_C_FLAGS}"   PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Install the ChronoEngine_sensor library
#-------------------------------------------------------------------------------

install(TARGETS ChronoEngine_sensor
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

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

if(NOT USE_CUDA_NVRTC)
  install(FILES ${generated_rt_files}
          DESTINATION lib/sensor_ptx)
else()
  install(FILES ${ChronoEngine_sensor_RT_SOURCES}
          DESTINATION include/chrono_sensor/optix/shaders)  
endif()

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
