#===============================================================================
# CMake configuration file for the Chrono_Granular library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================


option(ENABLE_MODULE_GRANULAR "Enable the Chrono Granular module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_GRANULAR)
  mark_as_advanced(FORCE BLAZE_DIR)
  mark_as_advanced(FORCE USE_GRANULAR_SIMD)

  # GCC > 4.9 does not support CUDA (<= 8.0) by default.
  # Users may explicitly select CUDA support if they are sure that their compiler
  # does make use of any conflicting features, such as C++14.
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
      message(STATUS "GCC version:  ${CMAKE_CXX_COMPILER_VERSION} does not support USE_GRANULAR_CUDA by default")
      set(USE_GRANULAR_CUDA OFF CACHE OFF "")
      mark_as_advanced(FORCE USE_GRANULAR_CUDA)
    endif()
  endif()

  return()
endif()

message(STATUS "==== Chrono Granular module ====")

mark_as_advanced(CLEAR BLAZE_DIR)
mark_as_advanced(CLEAR USE_GRANULAR_SIMD)
mark_as_advanced(CLEAR USE_GRANULAR_CUDA)

# ------------------------------------------------------------------------------
# Additional compiler flags
# ------------------------------------------------------------------------------

set(CH_GRANULAR_CXX_FLAGS "")
set(CH_GRANULAR_C_FLAGS "")

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----
find_package(CUDA QUIET REQUIRED)

cmake_dependent_option(USE_GRANULAR_CUDA "Enable CUDA support in Chrono::Granular" ON "CUDA_FOUND" OFF)

message( "Dan Flags are " ${CUDA_NVCC_FLAGS})

IF(USE_GRANULAR_CUDA)
  SET(CUDA_SEPARABLE_COMPILATION ON)
message ( ${CMAKE_SYSTEM_NAME} ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} )
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -G -lineinfo)
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    IF(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
      IF(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 4.9)
        SET(CUDA_SEPARABLE_COMPILATION OFF)
        SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++11")
        SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++11")
      ENDIF()
    ENDIF()
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC --compiler-options -Wall -lineinfo)
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  ENDIF()

  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -gencode=arch=compute_60,code=sm_60 -gencode=arch=compute_50,code=sm_50 -gencode=arch=compute_35,code=sm_35)
  message( "Dan 2 Flags are " ${CUDA_NVCC_FLAGS})
  #INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/FindCudaArch.cmake)
  #SELECT_NVCC_ARCH_FLAGS(NVCC_FLAGS_EXTRA)
  #LIST(APPEND CUDA_NVCC_FLAGS ${NVCC_FLAGS_EXTRA})
  message( "Dan 3 Flags are " ${CUDA_NVCC_FLAGS})

  SET(CHRONO_GRANULAR_USE_CUDA "#define CHRONO_GRANULAR_USE_CUDA")
ELSE()
  SET(CHRONO_GRANULAR_USE_CUDA "#undef CHRONO_GRANULAR_USE_CUDA")
ENDIF()

# ----- SIMD (SSE or AVX) support -----

cmake_dependent_option(USE_GRANULAR_SIMD "Enable SIMD support in Chrono::Granular (if available)" ON "CHRONO_HAS_AVX" OFF)

if(USE_GRANULAR_SIMD)
  set(CHRONO_GRANULAR_USE_SIMD "#define CHRONO_GRANULAR_USE_SIMD")
endif()


# ----- Thrust library -----

find_package(Thrust)

# ----- Blaze library -----

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "C:/Program\ Files\ \(x86\)" "C:/Program\ Files")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ENDIF()

# ----- Configure Blaze and Thrust -----

IF(ENABLE_OPENMP)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)
ELSEIF(ENABLE_TBB)
  add_definitions(-DBLAZE_USE_CPP_THREADS)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_TBB)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_TBB)
ELSE()
  add_definitions(-DBLAZE_USE_CPP_THREADS)
  add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CPP)
  add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP)
ENDIF()

# ----- BOOST -- required for Blaze -----

set(BOOST_ROOT "" CACHE PATH "Where is Boost located?")
find_package(Boost REQUIRED)

if (Boost_FOUND)
  message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
else()
  mark_as_advanced(CLEAR BOOST_ROOT)
endif()

# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigGranular.h.in
  ${PROJECT_BINARY_DIR}/chrono_granular/ChConfigGranular.h
  )

install(FILES "${PROJECT_BINARY_DIR}/chrono_granular/ChConfigGranular.h"
        DESTINATION include/chrono_granular)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the GRANULAR module
# ------------------------------------------------------------------------------

SET(CH_GRANULAR_INCLUDES
    ${BLAZE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${THRUST_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES(${CH_GRANULAR_INCLUDES})

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_GRANULAR_CXX_FLAGS "${CH_GRANULAR_CXX_FLAGS}" PARENT_SCOPE)
SET(CH_GRANULAR_C_FLAGS "${CH_GRANULAR_C_FLAGS}" PARENT_SCOPE)
SET(CH_GRANULAR_INCLUDES "${CH_GRANULAR_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono-Granular module
# ------------------------------------------------------------------------------

SET(ChronoEngine_Granular_BASE
    ChApiGranular.h
    ChGranularDefines.h
    )

SOURCE_GROUP("" FILES ${ChronoEngine_Granular_BASE})

SET(ChronoEngine_Granular_PHYSICS
    physics/ChGranular.h
    physics/ChGranular.cpp
    physics/ChSettingsGranularSMC.h
    physics/ChGranularTriMesh.h
	physics/ChGranularTriMesh.cpp
    )

SOURCE_GROUP(physics FILES ${ChronoEngine_Granular_PHYSICS})

SET(ChronoEngine_Granular_CUDA
    physics/ChGranularGPU_SMC.cu
    physics/ChGranularGPU_SMC.cuh
    physics/ChGranularGPU_SMC_trimesh.cu
    physics/ChGranularCollision.cuh
    physics/ChGranularCollision.cu
    physics/ChGranularBoxTriangle.cu
	utils/ChGranularUtilities_CUDA.cuh
	utils/ChCudaMathUtils.cuh
    )

SOURCE_GROUP(cuda FILES ${ChronoEngine_Granular_CUDA})

SET(ChronoEngine_Granular_UTILITIES
    utils/ChGranularUtilities.h
    )

SOURCE_GROUP(utilities FILES ${ChronoEngine_Granular_UTILITIES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_granular library
# ------------------------------------------------------------------------------

CUDA_ADD_LIBRARY(ChronoEngine_granular SHARED
            ${ChronoEngine_Granular_BASE}
            ${ChronoEngine_Granular_PHYSICS}
            ${ChronoEngine_Granular_CUDA}
            )
SET(CHRONO_GRANULAR_LINKED_LIBRARIES ChronoEngine ${CUDA_FRAMEWORK} ${MPI_LIBRARIES} ${TBB_LIBRARIES})

SET_TARGET_PROPERTIES(ChronoEngine_granular PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_GRANULAR")

TARGET_LINK_LIBRARIES(ChronoEngine_granular ${CHRONO_GRANULAR_LINKED_LIBRARIES})

INSTALL(TARGETS ChronoEngine_granular
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_granular
        FILES_MATCHING PATTERN "*.h")

IF(USE_GRANULAR_CUDA)
  INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
          DESTINATION include/chrono_granular
          FILES_MATCHING PATTERN "*.cuh")
ENDIF()

mark_as_advanced(FORCE
    CUDA_BUILD_CUBIN
    CUDA_BUILD_EMULATION
    CUDA_SEPARABLE_COMPILATION
    CUDA_SDK_ROOT_DIR
    CUDA_VERBOSE_BUILD
    CUDA_HOST_COMPILER)

message( "Flags are " ${CUDA_NVCC_FLAGS})








