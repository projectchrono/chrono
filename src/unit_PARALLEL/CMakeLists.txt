#=============================================================================
# CMake configuration file for Chrono_Parallel
#=============================================================================

cmake_minimum_required(VERSION 2.8)

project(ChronoEngine_Parallel)

# Location of additional CMAKE modules
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ------------------------------------------------------------------------------
# User-specified for optional modules
# ------------------------------------------------------------------------------
SET(ENABLE_UTILS  TRUE  CACHE BOOL "Build the ChronoParallel_Utils library") 
SET(ENABLE_TESTS  FALSE CACHE BOOL "Build test programs")

# ------------------------------------------------------------------------------
# Required Chrono library and headers
# ------------------------------------------------------------------------------
SET(CHRONO_INC       ""   CACHE PATH "Location of the Chrono headers")
SET(CHRONO_LIB_PATH  ""   CACHE PATH "Location of the ChronoEngine library")

# ------------------------------------------------------------------------------
# Specify output locations for libraries and executables
# ------------------------------------------------------------------------------
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

IF (MSVC OR XCODE_VERSION)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ENDIF()

# ------------------------------------------------------------------------------
# Figure out SSE level support
# ------------------------------------------------------------------------------
FIND_PACKAGE(SSE)

# Set substitution variables for configuration file.
IF(SSE_FOUND)
  MESSAGE(STATUS "SSE version: ${SSE_VERSION}")
  SET(CHRONO_PARALLEL_HAS_SSE "#define CHRONO_PARALLEL_HAS_SSE")
  SET(CHRONO_PARALLEL_SSE_LEVEL "#define CHRONO_PARALLEL_SSE_LEVEL \"${SSE_VERSION}\"")
ELSE()
  MESSAGE("No SSE support")
ENDIF()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # using Clang
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse${SSE_VERSION}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse${SSE_VERSION}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xsse${SSE_VERSION}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -xsse${SSE_VERSION}")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
endif()

# ------------------------------------------------------------------------------
# Thrust-related settings
# ------------------------------------------------------------------------------
#add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
#add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)

# ------------------------------------------------------------------------------
# OpenMP
# ------------------------------------------------------------------------------
find_package("OpenMP" REQUIRED)

# Add the OpenMP-specific compiler and linker flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# Determine OpenMP version
IF(OPENMP_FOUND)
  INCLUDE(CheckOpenMPVersion)
  MESSAGE(STATUS "OpenMP version: ${OMP_VERSION}")
  SET(CHRONO_PARALLEL_OMP_VERSION "#define CHRONO_PARALLEL_OMP_VERSION \"${OMP_VERSION}\"")
  IF(OMP_20)
    SET(CHRONO_PARALLEL_OMP_20 "#define CHRONO_PARALLEL_OMP_20")
  ELSE()
    SET(CHRONO_PARALLEL_OMP_20 "")
  ENDIF()
  IF(OMP_30)
    SET(CHRONO_PARALLEL_OMP_30 "#define CHRONO_PARALLEL_OMP_30")
  ELSE()
    SET(CHRONO_PARALLEL_OMP_30 "")
  ENDIF()
  IF(OMP_40)
    SET(CHRONO_PARALLEL_OMP_40 "#define CHRONO_PARALLEL_OMP_40")
  ELSE()
    SET(CHRONO_PARALLEL_OMP_40 "")
  ENDIF()
ENDIF()

# ------------------------------------------------------------------------------
# MPI
# ------------------------------------------------------------------------------
#find_package("MPI" )

# Add the MPI-specific compiler and linker flags
# Also, search for #includes in MPI's paths

#list(APPEND CMAKE_C_COMPILE_FLAGS ${MPI_C_COMPILE_FLAGS})
#list(APPEND CMAKE_C_LINK_FLAGS ${MPI_C_LINK_FLAGS})
#include_directories(${MPI_C_INCLUDE_PATH})

#list(APPEND CMAKE_CXX_COMPILE_FLAGS ${MPI_CXX_COMPILE_FLAGS})
#list(APPEND CMAKE_CXX_LINK_FLAGS ${MPI_CXX_LINK_FLAGS})
#include_directories(${MPI_CXX_INCLUDE_PATH})

# ------------------------------------------------------------------------------
# CUDA -- required for the Thrust library
# ------------------------------------------------------------------------------
# Use the FindCUDA.cmake to find the CUDA toolkit and set many interesting 
# variables and compilation rules. This saves eons of time.
# Interesting vars that are set: paths CUDA_SDK_ROOT_DIR and CUDA_TOOLKIT_ROOT_DIR

find_package(CUDA QUIET)
IF(CUDA_FOUND)
    message(STATUS "CUDA found")
ELSE()
    message(FATAL_ERROR "CUDA not found! (you may switch off ENABLE_UNIT_GPU and build without this unit)")
ENDIF()

# Detect what architecture we're on
# Paths we'll use to find some of the extra libraries
# Ensure that correct path for architecture is selected
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
SET(CUDA_TOOLKIT_LIB_DIR
    ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64)
SET(CUDA_SDK_LIB_DIR
    ${CUDA_SDK_ROOT_DIR}/common/lib/x64
    ${CUDA_SDK_ROOT_DIR}/common/lib/linux/x86_64
    ${CUDA_SDK_ROOT_DIR}/common/lib/darwin)
ELSE()
    message(FATAL_ERROR "Architecture detected as 32-bit. Only 64-bit Builds Supported!")
ENDIF()

find_library(CH_CUDART NAMES cudart libcudart PATHS ${CUDA_TOOLKIT_LIB_DIR})

# These need to get straightened out
# Anything using the SDK should only be used by nvcc, no need to include them for anything being sent to the system compiler
INCLUDE_DIRECTORIES(
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    ${CUDA_SDK_ROOT_DIR}/common/inc
    ${CUDA_TOOLKIT_INCLUDE}
    )

# ------------------------------------------------------------------------------
# Compiler settings
# ------------------------------------------------------------------------------

# set the compiler as a variable
IF(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    IF(CMAKE_SIZEOF_VOID_P MATCHES 4)
        SET (COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    ELSE()
        SET (COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    ENDIF()
ENDIF()

IF(MSVC AND CMAKE_CL_64)
    SET (COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
ELSEIF(MSVC)
    SET (COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
ENDIF()

IF(MSVC)
    # Avoid deprecation warnings
    ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    
    # Specify a single Build type, so we can import the correct .lib for Chrono
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "For single-configuration generators (e.g. make) set the type of build: Release, Debug, RelWithDebInfo, MinSizeRel")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")

    # Set compiler flags based on which compiler we're using
    IF("${COMPILER}" STREQUAL "COMPILER_MSVC")
        SET(CH_BUILDFLAGS "-DWIN32; -DNOMINMAX; -MP")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET(CH_BUILDFLAGS "-DWIN64; -D_WIN64; -DNOMINMAX; -MP")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_GCC")
        SET(CH_BUILDFLAGS "-DWIN32 -D_MINGW -D_WINDOWS")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_GCC_X64")
        SET(CH_BUILDFLAGS "-DWIN64 -D_MINGW -D_WINDOWS -m64")
    ENDIF()
ENDIF()

# ------------------------------------------------------------------------------
# Find the ChronoEngine library
# ------------------------------------------------------------------------------

# Hack to be able to link different libraries depending on build type
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # find the CE library. First make sure we didn't already find it; if we did, unset the variable
    # need this to easily switch between Debug and Release build types
    IF(CHRONO_LIB)
        UNSET(CHRONO_LIB CACHE)
    ENDIF()

    # Find the Chrono library based on the build type
    FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH}/${CMAKE_BUILD_TYPE} REQUIRED)

    # Copy the Chrono DLL from/to appropriate place, based on build type
    SET(CHRONO_DLL "${CHRONO_LIB_PATH}/../bin/${CMAKE_BUILD_TYPE}/ChronoEngine.dll")
    FILE(COPY ${CHRONO_DLL} DESTINATION "${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}")

    # Report what library we use and which DLL was copied
    MESSAGE(STATUS "Found chrono lib: ${CHRONO_LIB}")
    MESSAGE(STATUS "Copied chrono dll: ${CHRONO_DLL}")
ELSE()
    # Find Chrono libs
    FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)
ENDIF()

# ------------------------------------------------------------------------------
# Add paths to required Chrono headers
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(
     ${CHRONO_INC}
    "${CHRONO_INC}/collision"
    "${CHRONO_INC}/collision/bullet"
    "${CHRONO_INC}/collision/bullet/LinearMath"
    "${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision"
    )

# ------------------------------------------------------------------------------
# Add paths to the top of the source directory and the binary directory
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

# ------------------------------------------------------------------------------
# Add subdirectories, depending on user-specified options
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(chrono_parallel)

IF(ENABLE_UTILS)

  SET(ENABLE_OPENGL  TRUE  CACHE BOOL "Build the ChronoParallel_Utils_OpenGL library")

  IF(${ENABLE_OPENGL})
    find_package(OpenGL)
    find_package(GLM)
    find_package(GLEW)
    find_package(GLFW)
    find_package(freetype)
    IF(${OPENGL_FOUND} AND ${GLM_FOUND})
      SET(CHRONO_PARALLEL_HAS_OPENGL "#define CHRONO_PARALLEL_HAS_OPENGL")
    ELSE()
      # Unable to find everything, turn off OpenGL
      SET(ENABLE_OPENGL  FALSE  CACHE BOOL "Build the ChronoParallel_Utils_OpenGL library") 
    ENDIF()
  ENDIF()

  ADD_SUBDIRECTORY(chrono_utils)
  IF(ENABLE_OPENGL)
    ADD_SUBDIRECTORY(chrono_utils/opengl)
  ENDIF()

ENDIF()

IF(ENABLE_TESTS)
  IF(NOT ENABLE_UTILS)
    MESSAGE(STATUS "The test programs require building the ChronoParallel_Utils library.")
  ELSE()
    ADD_SUBDIRECTORY(tests)
  ENDIF()
ENDIF()

# ----------------------------------------------------------------------------
# Generate configuration header file.
# ----------------------------------------------------------------------------
# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/cmake/ChConfigParallel.in
  ${PROJECT_BINARY_DIR}/chrono_parallel/ChConfigParallel.h
  )

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})

