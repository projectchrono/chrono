#=============================================================================
# CMake configuration file for Chrono Sensor module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_SENSOR "Enable the Chrono Sensor module" OFF)

IF(NOT ENABLE_MODULE_SENSOR)
  return()
ENDIF()

message(STATUS "==== Chrono Sensor module ====")

# Return now if CUDA is not available
if(NOT CUDA_FOUND)
    message(WARNING "Chrono::Sensor requires CUDA, but CUDA was not found; disabling Chrono::Sensor")
    set(ENABLE_MODULE_SENSOR OFF CACHE BOOL "Enable the Chrono Sensor module" FORCE)
    return()
endif()

#Check for GLFW to use as window to display data for debug purposes
mark_as_advanced(CLEAR GLFW_INCLUDE_DIR)
mark_as_advanced(CLEAR GLFW_LIBRARY)

find_package(GLFW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

SET(CH_SENSOR_INCLUDES
    ${CH_SENSOR_INCLUDES}
    ${GLFW_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
)

# ------------------------------------------------------------------------------
# Find and set everything needed for OptiX
# ------------------------------------------------------------------------------
find_package(OptiX REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_compile_definitions(NOMINMAX)
	message(STATUS "NOMINMAX set for windows")
endif()

SET(CH_SENSOR_INCLUDES
    ${CH_SENSOR_INCLUDES}
    ${OptiX_INCLUDE}/optixu
    ${OptiX_INCLUDE}
    ${CUDA_INCLUDE_DIRS}
)

# add the necessary optix libraries to the list to link
list(APPEND LIBRARIES ${optix_LIBRARY})
list(APPEND LIBRARIES ${optix_rpath})
list(APPEND LIBRARIES ${CUDA_LIBRARIES})
list(APPEND LIBRARIES ${CUDA_npp_LIBRARY})


#add the necessary opengl, glfw, glew libraries to link list
list(APPEND LIBRARIES ${GLFW_LIBRARY})
list(APPEND LIBRARIES ${GLEW_LIBRARY})
list(APPEND LIBRARIES ${OPENGL_LIBRARIES})

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
  find_library(CUDA_nvrtc_LIBRARY nvrtc ${CUDA_TOOLKIT_ROOT_DIR}/lib64)
  mark_as_advanced(CUDA_nvrtc_LIBRARY)

  if(NOT CUDA_nvrtc_LIBRARY)
    set(USE_CUDA_NVRTC OFF)
  else()
    list(APPEND LIBRARIES ${CUDA_nvrtc_LIBRARY})
    add_definitions( -DUSE_CUDA_NVRTC )
    add_definitions( -DSENSOR_CUDA_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/rtkernels/" )

    #set(CUDA_NVRTC_FLAGS -arch compute_30 -use_fast_math -lineinfo -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
    set(CUDA_NVRTC_FLAGS -use_fast_math -default-device -rdc true -D__x86_64 CACHE STRING "NVRTC flags as list." FORCE)
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
      ${CMAKE_CURRENT_SOURCE_DIR}/rtkernels
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
    rtkernels/box.cu
    rtkernels/sphere.cu
    rtkernels/cylinder.cu
    rtkernels/camera.cu
    rtkernels/lidar.cu
    rtkernels/miss.cu
    rtkernels/exception.cu
    rtkernels/triangle_mesh.cu
    rtkernels/camera_shaders.cu
    rtkernels/lidar_shaders.cu
    rtkernels/shadow_shaders.cu
)
set(ChronoEngine_sensor_RT_HEADERS
    rtkernels/ray_utils.h

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
)

source_group("Cuda" FILES
    ${ChronoEngine_sensor_CUDA_SOURCES}
	${ChronoEngine_sensor_CUDA_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_SOURCES
    ChSensor.cpp
    ChOptixSensor.cpp
    ChCameraSensor.cpp
    ChLidarSensor.cpp
    ChIMUSensor.cpp
    ChGPSSensor.cpp
    ChSensorManager.cpp
    ChDynamicsManager.cpp
    Sensor.cpp
)
set(ChronoEngine_sensor_HEADERS
  	ChApiSensor.h
    ChSensor.h
    ChOptixSensor.h
    ChLidarSensor.h
    ChIMUSensor.h
    ChGPSSensor.h
    ChSensorManager.h
    ChDynamicsManager.h
  	ChSensorBuffer.h
    Sensor.h
)

source_group("Source" FILES
    ${ChronoEngine_sensor_SOURCES}
  	${ChronoEngine_sensor_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_SCENE_SOURCES
    scene/ChScene.cpp
)
set(ChronoEngine_sensor_SCENE_HEADERS
    scene/ChScene.h
    scene/lights.h
)

source_group("Scene" FILES
    ${ChronoEngine_sensor_SCENE_SOURCES}
  	${ChronoEngine_sensor_SCENE_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR OPTIX LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_OPTIX_SOURCES
    optixcpp/ChOptixEngine.cpp
    optixcpp/ChOptixUtils.cpp
)
set(ChronoEngine_sensor_OPTIX_HEADERS
    optixcpp/ChOptixEngine.h
    optixcpp/ChOptixUtils.h
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
  	filters/ChFilterOptixRender.cpp
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
)
set(ChronoEngine_sensor_FILTERS_HEADERS
  	filters/ChFilter.h
  	filters/ChFilterOptixRender.h
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
)

source_group("Filters" FILES
    ${ChronoEngine_sensor_FILTERS_SOURCES}
  	${ChronoEngine_sensor_FILTERS_HEADERS}
)


#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------
IF(USE_TENSOR_RT)
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
ENDIF()


#-----------------------------------------------------------------------------
# LIST THE UTILITY FILES THAT WILL BE EXPOSED TO THE USER
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_UTILS_SOURCES
    utils/ChVisualMaterialUtils.cpp
    utils/ChUtilsJSON.cpp
)
set(ChronoEngine_sensor_UTILS_HEADERS
    utils/ChVisualMaterialUtils.h
  	utils/CudaMallocHelper.h
    utils/ChUtilsJSON.h
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
  set(CUDA_GENERATED_OUTPUT_DIR "${CMAKE_BINARY_DIR}/sensor_ptx")
  add_definitions( -DPTX_GENERATED_PATH="${CUDA_GENERATED_OUTPUT_DIR}/" )
  CUDA_WRAP_SRCS(ChronoEngine_sensor PTX generated_rt_files ${ChronoEngine_sensor_RT_SOURCES} )
  source_group("Generated Files" FILES ${generated_rt_files})
endif()

# reset the cuda generate directory for cuda files being compiled into obj
set(CUDA_GENERATED_OUTPUT_DIR "")

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the Sensor module.
# Make some variables visible from parent directory
# ----------------------------------------------------------------------------

list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_SOURCES})
list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_HEADERS})
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

IF(USE_TENSOR_RT)
    list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_TENSORRT_SOURCES})
    list(APPEND ALL_CH_SENSOR_FILES ${ChronoEngine_sensor_TENSORRT_HEADERS})
ENDIF()

add_library(ChronoEngine_sensor SHARED
    ${ALL_CH_SENSOR_FILES}
)

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")

    #FIGURE OUT WHERE ALL THE LIBRARY DLLS FROM WINDOWS ARE
    #SET(CH_IRRLICHT_DLL "${IRRLICHT_ROOT}/bin/Win32-VisualStudio/Irrlicht.dll")
endif()

set_target_properties(ChronoEngine_sensor PROPERTIES
                    COMPILE_FLAGS "${CH_CXX_FLAGS}"
                    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
				  	COMPILE_DEFINITIONS "CH_API_COMPILE_SENSOR")

target_include_directories(ChronoEngine_sensor PUBLIC ${CH_SENSOR_INCLUDES})

target_link_libraries(ChronoEngine_sensor ChronoEngine ${LIBRARIES})

#MAKE INCLUDES AND DEPENDENCIES AVAILABLE TO USERS
set(CH_SENSOR_INCLUDES "${CH_SENSOR_INCLUDES}" PARENT_SCOPE)
SET(SENSOR_LIBRARIES         "${LIBRARIES}"                  PARENT_SCOPE)
set(CH_SENSOR_CXX_FLAGS     "${CH_SENSOR_CXX_FLAGS}"        PARENT_SCOPE)
set(CH_SENSOR_C_FLAGS       "${CH_SENSOR_C_FLAGS}"          PARENT_SCOPE)

#-------------------------------------------------------------------------------
# On Windows, copy the GLEW and Optix DLLs to the bin/ directory
# We do this through a custom command so that the DLLs are copied to the
# appropriate directory (depending on the build type); however, we use
# copy_if_different. 

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  # GLEW doesn't provide a 'root' variable so we make our own
  get_filename_component(GLEW_ROOT ${GLEW_INCLUDE_DIR}/../ ABSOLUTE)

  # Set variables that point to the DLLs. OptiX (and thus Chrono::Sensor) don't 
  # support 32 bit, so we just take the x64 version of GLEW.
  # CMake/the user already defines the variable optix_DLL but you could do...
  # SET(CH_OPTIX_DLL "${OptiX_INSTALL_DIR)/bin64/optix.6.5.0.dll")
  SET(CH_GLEW_DLL "${GLEW_ROOT}/bin/Release/x64/glew32.dll")

  ADD_CUSTOM_COMMAND(
    TARGET ChronoEngine_sensor POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${optix_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_GLEW_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    MAIN_DEPENDENCY ChronoEngine_sensor
    COMMENT "Copying OptiX and GLEW DLLs..."
  )

  # copy the dlls into the installed package to be on the safe side 
  install(FILES "${optix_DLL}" DESTINATION bin)
  install(FILES "${CH_GLEW_DLL}" DESTINATION bin)

  # Make variables visible from outside this directory
  SET(CH_OPTIX_DLL "${optix_DLL}" PARENT_SCOPE)
  SET(CH_GLEW_DLL "${CH_GLEW_DLL}" PARENT_SCOPE)

ENDIF()

#-------------------------------------------------------------------------------
# Install the ChronoEngine_sensor library

install(TARGETS ChronoEngine_sensor
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_sensor_HEADERS}
		DESTINATION include/chrono_sensor)
install(FILES ${ChronoEngine_sensor_UTILS_HEADERS}
		DESTINATION include/chrono_sensor/utils)
install(FILES ${ChronoEngine_sensor_OPTIX_HEADERS}
        DESTINATION include/chrono_sensor/optixcpp)
install(FILES ${ChronoEngine_sensor_FILTERS_HEADERS}
        DESTINATION include/chrono_sensor/filters)
install(FILES ${ChronoEngine_sensor_CUDA_HEADERS}
      DESTINATION include/chrono_sensor/cuda)
install(FILES ${ChronoEngine_sensor_SCENE_HEADERS}
      DESTINATION include/chrono_sensor/scene)
