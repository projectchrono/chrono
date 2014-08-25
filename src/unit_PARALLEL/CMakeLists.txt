#=============================================================================
# CMake configuration file for Chrono_Parallel
#=============================================================================

cmake_minimum_required(VERSION 2.8)

project(ChronoEngine_Parallel)

# Policy CMP0046 warning when a dependency does not exist
# According to a dev from kitware The add_dependencies 
# command is for inter-target ordering dependencies, not
# file-level dependencies.
# This warning appears in Cmake 3.0

if(CMAKE_MAJOR_VERSION EQUAL 3)
    if(COMMAND cmake_policy)
      cmake_policy(SET CMP0046 OLD)
    endif(COMMAND cmake_policy)
endif(CMAKE_MAJOR_VERSION EQUAL 3)

IF(APPLE)
  set(CMAKE_MACOSX_RPATH OFF)
ENDIF()

# Enable CTest
enable_testing()
include(CTest)

# Location of additional CMAKE modules
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ------------------------------------------------------------------------------
# User-specified for optional modules
# ------------------------------------------------------------------------------
INCLUDE(CMakeDependentOption)

OPTION(BUILD_UTILS "Build the ChronoEngine_Utils library" ON)
OPTION(BUILD_OPENGL "Build the ChronoEngine_OpenGL library" ON)

# If building the ChronoEngine_Utils library, provide an option to build demo
# programs (default ON). Otherwise, do not build demo programs.
CMAKE_DEPENDENT_OPTION(BUILD_DEMOS "Build demo programs" ON "BUILD_UTILS" OFF)

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
OPTION(USE_SSE "Compile with SSE support for floating point math" ON)
OPTION(USE_DOUBLE "Compile with double precision math (no SSE support)" OFF)

IF(USE_DOUBLE)
  OPTION(USE_SSE "Compile with SSE support for floating point math" OFF)
  SET(CHRONO_PARALLEL_USE_DOUBLE "#define CHRONO_PARALLEL_USE_DOUBLE")
ELSEIF(USE_SSE)
  FIND_PACKAGE(SSE)

  # Set substitution variables for configuration file.
  IF(SSE_FOUND)
    MESSAGE(STATUS "SSE version: ${SSE_VERSION}")
    SET(CHRONO_PARALLEL_HAS_SSE "#define CHRONO_PARALLEL_HAS_SSE")
    SET(CHRONO_PARALLEL_SSE_LEVEL "#define CHRONO_PARALLEL_SSE_LEVEL \"${SSE_VERSION}\"")
  ELSE()
    MESSAGE("No SSE support")
  ENDIF()

  IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # using Clang
  ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS} -std=c++11")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SSE_FLAGS} -std=c++11")
  ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xsse${SSE_VERSION}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -xsse${SSE_VERSION}")
  ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # using Visual Studio C++
  ENDIF()

ENDIF()

#Set c++11 flags
IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
# using Clang
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
# using Visual Studio C++
ENDIF()


# ------------------------------------------------------------------------------
# Thrust-related settings
# ------------------------------------------------------------------------------
#add_definitions(-DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP)
#add_definitions(-DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP)

# ------------------------------------------------------------------------------
# OpenMP
# ------------------------------------------------------------------------------
find_package("OpenMP")

# Add the OpenMP-specific compiler and linker flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

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
    message(FATAL_ERROR "CUDA not found!")
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
# Anything using the SDK should only be used by nvcc,
# no need to include them for anything being sent to the system compiler
INCLUDE_DIRECTORIES(
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    ${CUDA_SDK_ROOT_DIR}/common/inc
    ${CUDA_TOOLKIT_INCLUDE}
    )

# ------------------------------------------------------------------------------
# Find the ChronoEngine library
# ------------------------------------------------------------------------------

FIND_PACKAGE(ChronoEngine)

IF(CHRONOENGINE_LIBRARY_DEBUG)
  MESSAGE(STATUS "ChronoEngine library (debug) found")
ENDIF()
IF(CHRONOENGINE_LIBRARY_RELEASE)
  MESSAGE(STATUS "ChronoEngine library (release) found")
ENDIF()

# Stop now if the path to the Chrono SDK is invalid or if we could not find
# at least one of the debug or release Chrono libraries.
IF(NOT EXISTS "${CH_CHRONO_SDKDIR}/physics/ChSystem.h")
  MESSAGE(FATAL_ERROR "Cannot find the Chrono SDK. Make sure CH_CHRONO_SDKDIR is set correctly.")
  RETURN()
ENDIF()
IF(NOT CHRONOENGINE_LIBRARY_DEBUG AND NOT CHRONOENGINE_LIBRARY_RELEASE)
  MESSAGE(FATAL_ERROR "Cannot find the ChronoEngine library. Check CH_LIBDIR_DEBUG and CH_LIBDIR_RELEASE.")
  RETURN()
ENDIF()

# Add paths to Chrono headers
INCLUDE_DIRECTORIES(${CHRONOENGINE_INCLUDES})

# ------------------------------------------------------------------------------
# Dependencies for ChronoEngine_OpenGL
# ------------------------------------------------------------------------------
IF(BUILD_OPENGL)

  find_package(OpenGL)
  find_package(GLM)
  find_package(GLEW)
  find_package(GLFW)

  # On windows, ask for the GLEW and GLFW DLLs so that we can copy. This is
  # optional.  If not specified, it is the user's responsibility to make them
  # available at runtime.
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(GLEW_DLL  "" CACHE FILEPATH "The GLEW DLL")
    SET(GLFW_DLL  "" CACHE FILEPATH "The GLFW DLL")
  ENDIF()

  # If all dependencies are available, enable building OpenGL support.
  IF(OPENGL_FOUND AND GLM_FOUND AND GLEW_FOUND AND GLFW_FOUND)
    SET(CHRONO_PARALLEL_HAS_OPENGL "#define CHRONO_PARALLEL_HAS_OPENGL")
    SET(ENABLE_OPENGL ON)
  ELSE()
    # Unable to find everything, turn off OpenGL
    MESSAGE("Cannot build ChronoEngine_OpenGL (missing dependencies).")
    SET(CHRONO_PARALLEL_HAS_OPENGL "")
    SET(ENABLE_OPENGL OFF)
  ENDIF()

ELSE()

  SET(CHRONO_PARALLEL_HAS_OPENGL "")
  SET(ENABLE_OPENGL OFF)

ENDIF()

# ------------------------------------------------------------------------------
# Automatically copy DLLs 
# ------------------------------------------------------------------------------
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  # Get the path to the Chrono bin directory (at this point, we know that at
  # least one of CH_LIBDIR_DEBUG or CH_LIBDIR_RELEASE is properly specified)
  IF(CH_LIBDIR_DEBUG)
      SET(CH_BINDIR "${CH_LIBDIR_DEBUG}/../../bin")
  ENDIF()
  IF(CH_LIBDIR_RELEASE)
      SET(CH_BINDIR "${CH_LIBDIR_RELEASE}/../../bin")
  ENDIF()

  # Create custom target for copying DLLs; add it to the default build target
  ADD_CUSTOM_TARGET(COPY_DLLS ALL)

  # Create custom commands, invoked post-build to copy DLLs to the appropriate
  # directory (depending on the configuration selected at build time in VS)
  ADD_CUSTOM_COMMAND(
    TARGET COPY_DLLS POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine.dll"
        "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
  )

  # If building OpenGL support, also copy the DLLs for GLEW and GLFW (if specified)
  IF(ENABLE_OPENGL)
    IF(EXISTS ${GLEW_DLL})
      ADD_CUSTOM_COMMAND(
        TARGET COPY_DLLS POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GLEW_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
      )
    ENDIF()
    IF(EXISTS ${GLFW_DLL})
      ADD_CUSTOM_COMMAND(
        TARGET COPY_DLLS POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${GLFW_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
      )
    ENDIF()
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

# ------------------------------------------------------------------------------
# Add paths to the top of the source directory
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

# ------------------------------------------------------------------------------
# Add subdirectories, depending on user-specified options
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(chrono_parallel)

IF(BUILD_UTILS)
  ADD_SUBDIRECTORY(chrono_utils)
ENDIF()

IF(ENABLE_OPENGL)
  ADD_SUBDIRECTORY(chrono_opengl)
ENDIF()

IF(BUILD_DEMOS)
  ADD_SUBDIRECTORY(tests)
ENDIF()

IF(BUILD_TESTING)
  ADD_SUBDIRECTORY(unit_testing)
ENDIF()
