#=============================================================================
# CMake configuration file for PARALLEL unit
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

# The Chrono Parallel module requires OpenMP.  Provide an option for building
# this module dependent on OPENMP_FOUND

cmake_dependent_option(ENABLE_UNIT_PARALLEL "Enable the Chrono Parallel module" OFF
                       "OPENMP_FOUND" OFF)

# Options for additional modules, dependent on this unit being enabled.
# Note that these must be specified here, otherwise they would not be
# hidden if we return when the module is disabled.

cmake_dependent_option(BUILD_OPENGL "Build the OpenGL support library" OFF
                       "ENABLE_UNIT_PARALLEL" OFF)
#TODO --  for now disbale vehicle support...
#OPTION(BUILD_VEHICLE "Enable support for vehicle modeling (requires the Chrono::Vehicle library)" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_UNIT_PARALLEL)
  mark_as_advanced(FORCE BLAZE_DIR)
  mark_as_advanced(FORCE BOOST_DIR)
  return()
endif()

message(STATUS "==== Chrono Parallel module ====")

mark_as_advanced(CLEAR BLAZE_DIR)
mark_as_advanced(CLEAR BOOST_DIR)


# ------------------------------------------------------------------------------
# Figure out SSE level support
# ------------------------------------------------------------------------------
OPTION(USE_SSE "Compile with SSE support for floating point math" OFF)
OPTION(USE_DOUBLE "Compile with double precision math (no SSE support)" ON)

IF(USE_DOUBLE)

  OPTION(USE_SSE "Compile with SSE support for floating point math" OFF)
  SET(CHRONO_PARALLEL_USE_DOUBLE "#define CHRONO_PARALLEL_USE_DOUBLE")
  
    IF (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    ELSEIF (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wa,-q")
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -Wa,-q")
    ELSEIF (${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    ELSEIF (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    ENDIF()
  
ELSEIF(USE_SSE)

  FIND_PACKAGE(SSE)

  # Set substitution variables for configuration file.
  IF(SSE_FOUND)
    MESSAGE(STATUS "SSE version: ${SSE_VERSION}")
    SET(CHRONO_PARALLEL_HAS_SSE "#define CHRONO_PARALLEL_HAS_SSE")
    SET(CHRONO_PARALLEL_SSE_LEVEL "#define CHRONO_PARALLEL_SSE_LEVEL \"${SSE_VERSION}\"")
    SET(CHRONO_PARALLEL_SSE_LEVEL "#define CHRONO_PARALLEL_SSE_${SSE_STR}")
  ELSE()
    MESSAGE("No SSE support")
  ENDIF()

  IF (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # using Clang
  ELSEIF (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SSE_FLAGS}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SSE_FLAGS}")
  ELSEIF (${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -xsse${SSE_VERSION}")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -xsse${SSE_VERSION}")
  ELSEIF (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    # using Visual Studio C++
  ENDIF()

ENDIF()

#Set c++11 flags

#    IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
#    # using Clang
#    ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
#      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
#    ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
#    ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
#    # using Visual Studio C++
#    ENDIF()

# ------------------------------------------------------------------------------
# OpenMP
# ------------------------------------------------------------------------------

# Add the OpenMP-specific compiler and linker flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# ------------------------------------------------------------------------------
# Thrust
# ------------------------------------------------------------------------------
find_package(Thrust)

# ------------------------------------------------------------------------------
# BLAZE -- required for some parallel solvers
# ------------------------------------------------------------------------------
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(BLAZE_DIR ""  CACHE PATH   "Where is Blaze located?")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(BLAZE_DIR NAMES blaze/Blaze.h PATHS "/usr/include" "/usr/local/include")
ENDIF()

# ------------------------------------------------------------------------------
# BOOST -- required for BLAZE
# ------------------------------------------------------------------------------
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
   SET(BOOST_DIR ""  CACHE PATH   "Where is Boost located?")
ENDIF()

# ------------------------------------------------------------------------------
# Find the ChronoVehicle library
# ------------------------------------------------------------------------------
IF(BUILD_VEHICLE)

  FIND_PACKAGE(ChronoVehicle)

  IF(CHRONOVEHICLE_LIBRARY_DEBUG)
    MESSAGE(STATUS "ChronoVehicle library (debug) found")
  ENDIF()
  IF(CHRONOVEHICLE_LIBRARY_RELEASE)
    MESSAGE(STATUS "ChronoVehicle library (release) found")
  ENDIF()

  SET(ENABLE_VEHICLE ON)

  # Disable ChronoVehicle support if we could not find the ChronoVehicle SDK or
  # if we could not find one of the debug or release ChronoVehicle libraries.
  IF(NOT EXISTS "${CH_VEHICLE_SDKDIR}/subsys/ChVehicle.h")
    MESSAGE("Cannot find the ChronoVehicle SDK. Make sure CH_VEHICLE_SDKDIR is set correctly.")
    SET(ENABLE_VEHICLE OFF)
  ENDIF()
  IF(NOT CHRONOVEHICLE_LIBRARY_DEBUG AND NOT CHRONOVEHICLE_LIBRARY_RELEASE)
    MESSAGE("Cannot find the ChronoVehicle library. Check CH_VEHICLE_LIBDIR_DEBUG and CH_VEHICLE_LIBDIR_RELEASE.")
    SET(ENABLE_VEHICLE OFF)
  ENDIF()

ELSE()

  SET(ENABLE_VEHICLE OFF)

ENDIF()

IF(ENABLE_VEHICLE)

  # Add path to ChronoVehicle headers.
  INCLUDE_DIRECTORIES(${CHRONOVEHICLE_INCLUDES})

  # Prepare substitution variables for configuration header.
  SET(CHRONO_PARALLEL_HAS_VEHICLE "#define CHRONO_PARALLEL_HAS_VEHICLE")
  SET(CHRONO_VEHICLE_DATA_DIR "${CH_VEHICLE_SDKDIR}/data/")

ELSE()

  # Dummy substitution variables.
  SET(CHRONO_PARALLEL_HAS_VEHICLE "")
  SET(CHRONO_VEHICLE_DATA_DIR "")
 
ENDIF()

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


#  # If ChronoVehicle support is enabled, copy the ChronoVehicle DLL
#  IF(ENABLE_VEHICLE)
#    IF(CH_VEHICLE_LIBDIR_DEBUG)
#        SET(CH_VEHICLE_BINDIR "${CH_VEHICLE_LIBDIR_DEBUG}/../../bin")
#    ENDIF()
#    IF(CH_VEHICLE_LIBDIR_RELEASE)
#        SET(CH_VEHICLE_BINDIR "${CH_VEHICLE_LIBDIR_RELEASE}/../../bin")
#    ENDIF()
#    ADD_CUSTOM_COMMAND(
#        TARGET COPY_DLLS POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_if_different
#            "${CH_VEHICLE_BINDIR}/$<CONFIGURATION>/ChronoVehicle.dll"
#            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
#    )
#  ENDIF()


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

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR} ${BLAZE_DIR} ${BOOST_DIR} ${THRUST_INCLUDE_DIR})

# ------------------------------------------------------------------------------
# Add paths to the top of the source directory for the PARALLEL module
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# ------------------------------------------------------------------------------
# Add subdirectories, depending on user-specified options
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(chrono_parallel)
ADD_SUBDIRECTORY(chrono_utils)

IF(ENABLE_OPENGL)
  ADD_SUBDIRECTORY(chrono_opengl)
ENDIF()

