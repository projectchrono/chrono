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
    SET(CHRONO_OPENGL "#define CHRONO_OPENGL")
    SET(ENABLE_OPENGL ON)
  ELSE()
    # Unable to find everything, turn off OpenGL
    MESSAGE("Cannot build ChronoEngine_OpenGL (missing dependencies).")
    SET(CHRONO_OPENGL "#undef CHRONO_OPENGL")
    SET(ENABLE_OPENGL OFF)
  ENDIF()

ELSE()

  SET(CHRONO_OPENGL "#undef CHRONO_OPENGL")
  SET(ENABLE_OPENGL OFF)

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

# ------------------------------------------------------------------------------
# Collect all include directories necessary for the PARALLEL module
# ------------------------------------------------------------------------------

SET(CH_PARALLEL_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
    ${BLAZE_DIR}
    ${BOOST_DIR}
    ${THRUST_INCLUDE_DIR}
)

SET(CH_OPENGL_INCLUDES
    ${GLM_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
)

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

SET(CH_PARALLEL_INCLUDES "${CH_PARALLEL_INCLUDES}" PARENT_SCOPE)
SET(ENABLE_OPENGL "${ENABLE_OPENGL}" PARENT_SCOPE)
SET(CH_OPENGL_INCLUDES "${CH_OPENGL_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add subdirectories, depending on user-specified options
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(chrono_parallel)
ADD_SUBDIRECTORY(chrono_utils)

IF(ENABLE_OPENGL)
  ADD_SUBDIRECTORY(chrono_opengl)
ENDIF()

