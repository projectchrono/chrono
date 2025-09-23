#=============================================================================
# CMake configuration file for Chrono Sensor module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_SENSOR "Enable the Chrono Sensor module" OFF)

if(NOT CH_ENABLE_MODULE_SENSOR)
  return()
endif()

message(STATUS "\n==== Chrono Sensor module ====\n")

# Return now if CUDA is not available
if(NOT CHRONO_CUDA_FOUND)
  message(WARNING "Chrono::Sensor requires CUDA, but CUDA was not found; disabling Chrono::Sensor")

  mark_as_advanced(FORCE GLM_INCLUDE_DIR)
  mark_as_advanced(FORCE GLEW_DIR)
  mark_as_advanced(FORCE glfw3_DIR)

  set(CH_ENABLE_MODULE_SENSOR OFF CACHE BOOL "Enable the Chrono Sensor module" FORCE)
  return()
endif()

mark_as_advanced(CLEAR GLM_INCLUDE_DIR)
mark_as_advanced(CLEAR GLEW_DIR)
mark_as_advanced(CLEAR glfw3_DIR)

option(CH_USE_NVDB "Enable Chrono::FSI deformable terrain sim rendering in Chrono::Sensor" OFF)

#-----------------------------------------------------------------------------
# LIST CUDA FILES USED FOR RT PROGRAMS - TO BE COMPILED TO PTX SHADERS
#-----------------------------------------------------------------------------

set(Chrono_sensor_RT_SOURCES
    optix/shaders/box.cu
    optix/shaders/sphere.cu
    optix/shaders/cylinder.cu
    optix/shaders/camera.cu
    optix/shaders/lidar.cu
    optix/shaders/miss.cu
    optix/shaders/material_shaders.cu
    optix/shaders/radar.cu
)

if(CH_USE_NVDB)
  list(APPEND Chrono_sensor_RT_SOURCES
    optix/shaders/nvdb_vol_intersect.cu
  )
endif()

set(Chrono_sensor_RT_HEADERS
    optix/shaders/device_utils.h
)

source_group("RT Programs" FILES
    ${Chrono_sensor_RT_SOURCES}
	${Chrono_sensor_RT_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST CUDA FILES THAT ARE TO BE COMPILED AS SOURCE
#-----------------------------------------------------------------------------

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
)

source_group("Cuda" FILES
    ${Chrono_sensor_CUDA_SOURCES}
	${Chrono_sensor_CUDA_HEADERS}
)


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
)

source_group("Source" FILES
    ${Chrono_sensor_SOURCES}
  	${Chrono_sensor_HEADERS}
)

set(Chrono_sensor_SENSORS_SOURCES
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

set(Chrono_sensor_SENSORS_HEADERS
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
    ${Chrono_sensor_SENSORS_SOURCES}
  	${Chrono_sensor_SENSORS_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------

set(Chrono_sensor_SCENE_SOURCES
    optix/scene/ChScene.cpp
)

set(Chrono_sensor_SCENE_HEADERS
    optix/scene/ChScene.h
)

source_group("Scene" FILES
    ${Chrono_sensor_SCENE_SOURCES}
  	${Chrono_sensor_SCENE_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR OPTIX LIBRARY
#-----------------------------------------------------------------------------

set(Chrono_sensor_OPTIX_SOURCES
    optix/ChOptixEngine.cpp
    optix/ChOptixGeometry.cpp
    optix/ChOptixPipeline.cpp
    optix/ChOptixUtils.cpp
    optix/ChFilterOptixRender.cpp
    optix/ChNVDBVolume.cpp
)

set(Chrono_sensor_OPTIX_HEADERS
    optix/ChOptixEngine.h
    optix/ChOptixGeometry.h
    optix/ChOptixPipeline.h
    optix/ChOptixUtils.h
    optix/ChFilterOptixRender.h
    optix/ChOptixDefinitions.h
    optix/ChNVDBVolume.h
)

source_group("Optix" FILES
    ${Chrono_sensor_OPTIX_SOURCES}
  	${Chrono_sensor_OPTIX_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------

set(Chrono_sensor_FILTERS_SOURCES
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

set(Chrono_sensor_FILTERS_HEADERS
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
    ${Chrono_sensor_FILTERS_SOURCES}
  	${Chrono_sensor_FILTERS_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------

if(CH_USE_TENSOR_RT)
    set(Chrono_sensor_TENSORRT_SOURCES
      	tensorrt/ChFilterUFF.cpp
        tensorrt/ChFilterONNX.cpp
    )
    set(Chrono_sensor_TENSORRT_HEADERS
      	tensorrt/ChFilterUFF.h
        tensorrt/ChFilterONNX.h
        tensorrt/ChTRTUtils.h
    )

    source_group("TensorRT" FILES
        ${Chrono_sensor_TENSORRT_SOURCES}
      	${Chrono_sensor_TENSORRT_HEADERS}
    )
endif()


#-----------------------------------------------------------------------------
# LIST THE UTILITY FILES THAT WILL BE EXPOSED TO THE USER
#-----------------------------------------------------------------------------

set(Chrono_sensor_UTILS_SOURCES
    utils/ChUtilsJSON.cpp
    utils/ChGPSUtils.cpp
    utils/Kdtree.cpp
    utils/Dbscan.cpp
)

set(Chrono_sensor_UTILS_HEADERS
  	utils/CudaMallocHelper.h
    utils/ChUtilsJSON.h
    utils/ChGPSUtils.h
    utils/Kdtree.h
    utils/Dbscan.h
)

source_group(Utils FILES
    ${Chrono_sensor_UTILS_SOURCES}
  	${Chrono_sensor_UTILS_HEADERS}
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
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_FILTERS_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_FILTERS_HEADERS})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_SCENE_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_SCENE_HEADERS})

list(APPEND ALL_CH_SENSOR_FILES ${SENSOR_STB_FILES})
list(APPEND ALL_CH_SENSOR_FILES ${SENSOR_TINYOBJ_FILES})

if(CH_USE_TENSOR_RT)
    list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_TENSORRT_SOURCES})
    list(APPEND ALL_CH_SENSOR_FILES ${Chrono_sensor_TENSORRT_HEADERS})
endif()

add_library(Chrono_sensor ${ALL_CH_SENSOR_FILES}
                                ${Chrono_sensor_CUDA_SOURCES})
add_library(Chrono::sensor ALIAS Chrono_sensor)

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

message(STATUS "Chrono CUDA architectures: ${CHRONO_CUDA_ARCHITECTURES}")

find_package(OptiX REQUIRED)

message(STATUS "OptiX include directory: ${OptiX_INCLUDE_DIR}")

file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindOptiX.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)

install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindOptiX.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
        )

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:NOMINMAX>)
endif()

target_link_libraries(Chrono_sensor PUBLIC OptiX::OptiX) # needs to be exposed to consumers
target_link_libraries(Chrono_sensor PRIVATE CUDA::cudart_static)
target_link_libraries(Chrono_sensor PRIVATE CUDA::nppc)
target_link_libraries(Chrono_sensor PRIVATE CUDA::nppig)
target_link_libraries(Chrono_sensor PRIVATE CUDA::nppidei)

if(CH_USE_NVDB)
  target_link_libraries(Chrono_sensor PRIVATE OpenVDB::openvdb)
  target_link_libraries(Chrono_sensor PRIVATE OpenVDB::nanovdb)
endif()

# print cxx compiler flags\
## TODO: DARIOM need to be passed to the host compiler?
# if(MSVC AND MSVC_VERSION GREATER_EQUAL 1900)
#   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
# endif()

target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>)
target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)

set_target_properties(Chrono_sensor PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})

# ------------------------------------------------------------------------------
# Optionally use NVRTC to compile shader code rather than NVCC to PTX
# ------------------------------------------------------------------------------

set(CH_USE_CUDA_NVRTC ON CACHE BOOL "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX")

if(CH_USE_CUDA_NVRTC)
  message(STATUS "Building Chrono::Sensor with NVRTC ON")

  find_package(CUDAToolkit REQUIRED)

  if(NOT TARGET CUDA::nvrtc)
    message(WARNING "CUDA::nvrtc target is missing; NVRTC will be turned off")
    set(CH_USE_CUDA_NVRTC OFF CACHE BOOL "Compile shader code at run-time with NVRTC rather than NVCC at build time to PTX" FORCE)
  else()
    message(STATUS "CUDA::nvrtc found and enabled")
    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVRTC>)

    target_link_libraries(Chrono_sensor PRIVATE CUDA::nvrtc)

    install(FILES ${Chrono_sensor_RT_SOURCES}
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

    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVRTC>)
    target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:SHADER_OUTPUT_PATH="${CMAKE_INSTALL_PREFIX}/include/chrono_sensor/optix/shaders/">)

  endif()
else()
  message(STATUS "Building Chrono::Sensor with NVRTC OFF")

  # Create helper PTX library
  add_library(Chrono_sensor_ptx OBJECT ${Chrono_sensor_RT_SOURCES} ${Chrono_sensor_RT_HEADERS})
  set_target_properties(Chrono_sensor_ptx PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})
  target_include_directories(Chrono_sensor_ptx PRIVATE ${CMAKE_SOURCE_DIR}/src)
  set_property(TARGET Chrono_sensor_ptx PROPERTY CUDA_PTX_COMPILATION ON)
  target_link_libraries(Chrono_sensor_ptx PRIVATE OptiX::OptiX)
  target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::cudart_static)
  target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::nppc)
  target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::nppig)
  target_link_libraries(Chrono_sensor_ptx PRIVATE CUDA::nppidei)
  if(CH_USE_NVDB)
    target_link_libraries(Chrono_sensor_ptx PRIVATE OpenVDB::openvdb)
    target_link_libraries(Chrono_sensor_ptx PRIVATE OpenVDB::nanovdb)
  endif()

  install(DIRECTORY $<TARGET_OBJECTS:Chrono_sensor_ptx>
          DESTINATION lib/sensor_ptx)

  # Link PTX helper library, use NVCC, and set macro with location of PTX files
  target_link_libraries(Chrono_sensor PUBLIC ${Chrono_sensor_ptx})
  target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:USE_CUDA_NVCC>)
  target_compile_definitions(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:SHADER_OUTPUT_PATH="${CMAKE_INSTALL_PREFIX}/lib/sensor_ptx/">)
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
message(STATUS "   include dir: ${OPENGL_INCLUDE_DIR}")
message(STATUS "   libraries:   ${OPENGL_LIBRARIES}")
message(STATUS "GLEW found:   ${GLEW_FOUND}")
message(STATUS "   include dir: ${GLEW_INCLUDE_DIRS}")
message(STATUS "   libraries:   ${GLEW_LIBRARIES}")
message(STATUS "glfw3 found:  ${glfw3_FOUND}")

if(OPENGL_FOUND AND GLEW_FOUND AND glfw3_FOUND)
  target_compile_definitions(Chrono_sensor PUBLIC $<$<COMPILE_LANGUAGE:CXX>:USE_SENSOR_GLFW>)
  target_link_libraries(Chrono_sensor PUBLIC GLEW::glew)
  target_link_libraries(Chrono_sensor PUBLIC glfw)
  target_link_libraries(Chrono_sensor PUBLIC OpenGL::GL)
  message(STATUS "GL libraries found")
else()
  message(STATUS "GL libraries not found; OpenGL support disabled")
endif()


if(CH_USE_NVDB)
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


# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4661>)
    target_compile_options(Chrono_sensor PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4005>)
endif()


target_compile_definitions(Chrono_sensor PUBLIC $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_SENSOR>)

target_link_libraries(Chrono_sensor PRIVATE Chrono_core ${LIBRARIES})

# ------------------------------------------------------------------------------
# Optionally find TensorRT Version___ and set libaries and includes
# ------------------------------------------------------------------------------

option(CH_USE_TENSOR_RT "Enable the TensorRT for Sensor Module" OFF)

IF(CH_USE_TENSOR_RT)

    set(TENSOR_RT_INSTALL_DIR "" CACHE PATH "Path to TensorRT")

    #TensorRT Libraries
    find_library(TENSOR_RT_NVINFER nvinfer ${TENSOR_RT_INSTALL_DIR}/lib)
    find_library(TENSOR_RT_PARSERS nvparsers ${TENSOR_RT_INSTALL_DIR}/lib)
    find_library(TENSOR_RT_ONNXPARSER nvonnxparser ${TENSOR_RT_INSTALL_DIR}/lib)

    find_path(TENSOR_RT_INCLUDE_PATH NAMES NvInfer.h PATHS ${TENSOR_RT_INSTALL_DIR}/include)

    target_include_directories(Chrono_sensor PRIVATE ${TENSOR_RT_INCLUDE_PATH})
    target_link_libraries(Chrono_sensor PRIVATE ${TENSOR_RT_NVINFER})
    target_link_libraries(Chrono_sensor PRIVATE ${TENSOR_RT_PARSERS})
    target_link_libraries(Chrono_sensor PRIVATE ${TENSOR_RT_ONNXPARSER})

    mark_as_advanced(TENSOR_RT_NVINFER)
    mark_as_advanced(TENSOR_RT_PARSERS)
    mark_as_advanced(TENSOR_RT_ONNXPARSER)

ENDIF()

install(TARGETS Chrono_sensor
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_sensor)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_sensor_HEADERS}
		DESTINATION include/chrono_sensor)
install(FILES ${Chrono_sensor_SENSORS_HEADERS}
		DESTINATION include/chrono_sensor/sensors)
install(FILES ${Chrono_sensor_UTILS_HEADERS}
		DESTINATION include/chrono_sensor/utils)
install(FILES ${Chrono_sensor_OPTIX_HEADERS}
        DESTINATION include/chrono_sensor/optix)
install(FILES ${Chrono_sensor_FILTERS_HEADERS}
        DESTINATION include/chrono_sensor/filters)
install(FILES ${Chrono_sensor_CUDA_HEADERS}
      DESTINATION include/chrono_sensor/cuda)
install(FILES ${Chrono_sensor_SCENE_HEADERS}
      DESTINATION include/chrono_sensor/optix/scene)
install(FILES ${Chrono_sensor_RT_HEADERS}
      DESTINATION include/chrono_sensor/optix/shaders)
      
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
