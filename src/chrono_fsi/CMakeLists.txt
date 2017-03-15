#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

# Return now if this module is not enabled
#
# TODO: REMOVE CHRONO_VEHICLE & CHRONO_PARALLEL AS REQUIREMENTS!!!!!
#
if(NOT ENABLE_MODULE_FSI OR NOT ENABLE_MODULE_PARALLEL)
  mark_as_advanced(FORCE CUDA_TOOLKIT_ROOT_DIR)
  mark_as_advanced(FORCE CUDA_USE_STATIC_CUDA_RUNTIME)
  mark_as_advanced(FORCE USE_FSI_DOUBLE)
  return()
endif()

message(STATUS "==== Chrono FSI module ====")

mark_as_advanced(CLEAR CUDA_TOOLKIT_ROOT_DIR)
mark_as_advanced(CLEAR CUDA_USE_STATIC_CUDA_RUNTIME)
mark_as_advanced(CLEAR USE_FSI_DOUBLE)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

find_package(CUDA)

#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --device-c")

# Chrono FSI on GCC requires explicit C++11 support. '-std=c++11' must also be
# passed to the host compiler to override -std=c++14 if it is enabled.
IF(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++11")
  SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++11")
ENDIF()
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_30,code=sm_30")
#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ---gpu-code=sm_30")

message(STATUS "  CUDA toolkit includes:    ${CUDA_TOOLKIT_ROOT_DIR}/include")
message(STATUS "  CUDA SDK includes:        ${CUDA_SDK_ROOT_DIR}/common/inc")

option(CUDA_PROPAGATE_HOST_FLAGS "set host flags off" FALSE)
option(USE_FSI_DOUBLE "Compile Chrono::FSI with double precision math" ON)
IF(USE_FSI_DOUBLE)
  SET(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
ENDIF()

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the FSI module.
# Make some variables cisible from parent directory
# ----------------------------------------------------------------------------

set(CH_FSI_INCLUDES
    "${CUDA_TOOLKIT_ROOT_DIR}/include"
)

set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES}" PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFSI.h.in
               ${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFSI.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFSI.h"
        DESTINATION include/chrono_fsi)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FSI FLUID-SOLID INTERACTION LIBRARY

SET(ChronoEngine_FSI_SOURCES
    ChBce.cu
    ChCollisionSystemFsi.cu
    ChDeviceUtils.cu
    ChFluidDynamics.cu
    ChFsiDataManager.cu
    ChFsiForceParallel.cu
    ChFsiGeneral.cu
    ChFsiInterface.cpp
    ChSystemFsi.cpp
    ChFsiTypeConvert.cpp
)

SET(ChronoEngine_FSI_HEADERS
    ChBce.cuh
    ChCollisionSystemFsi.cuh
    ChDeviceUtils.cuh
    ChFluidDynamics.cuh
    ChParams.cuh
    ChFsiDataManager.cuh
    ChFsiForceParallel.cuh
    ChFsiGeneral.cuh
    ChFsiInterface.h
    ChSystemFsi.h
    ChSphGeneral.cuh
    ChApiFsi.h
    ChFsiTypeConvert.h
    custom_math.h
)

SOURCE_GROUP("" FILES
    ${ChronoEngine_FSI_SOURCES}
    ${ChronoEngine_FSI_HEADERS})

set(ChronoEngine_FSI_UTILS_SOURCES
    utils/ChUtilsGeneratorBce.cpp
    utils/ChUtilsGeneratorFluid.cpp
    utils/ChUtilsGeneratorFsi.cpp
    utils/ChUtilsPrintSph.cpp
)

set(ChronoEngine_FSI_UTILS_HEADERS
    utils/ChUtilsGeneratorBce.h
    utils/ChUtilsGeneratorFluid.h
    utils/ChUtilsGeneratorFsi.h
    utils/ChUtilsPrintSph.h
    utils/ChUtilsPrintStruct.h
)

source_group(utils FILES
    ${ChronoEngine_FSI_UTILS_SOURCES}
    ${ChronoEngine_FSI_UTILS_HEADERS})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_fsi library
#-----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBRARIES ChronoEngine_opengl)
ENDIF()

if(ENABLE_MODULE_PARALLEL)
  set(CXX_FLAGS ${CH_PARALLEL_CXX_FLAGS})
  include_directories(${CH_PARALLEL_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_parallel)
endif()

if(ENABLE_MODULE_VEHICLE)
  include_directories(${CH_VEHICLE_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_vehicle)
endif()

CUDA_ADD_LIBRARY(ChronoEngine_fsi SHARED
    ${ChronoEngine_FSI_SOURCES}
    ${ChronoEngine_FSI_HEADERS}
    ${ChronoEngine_FSI_UTILS_SOURCES}
    ${ChronoEngine_FSI_UTILS_HEADERS}
)

SET_TARGET_PROPERTIES(ChronoEngine_fsi PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_FSI")

TARGET_LINK_LIBRARIES(ChronoEngine_fsi ${LIBRARIES})

INSTALL(TARGETS ChronoEngine_fsi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

INSTALL(FILES ${ChronoEngine_FSI_HEADERS}
        DESTINATION include/chrono_fsi)
INSTALL(FILES ${ChronoEngine_FSI_UTILS_HEADERS}
        DESTINATION include/chrono_fsi/utils)
