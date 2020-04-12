#=============================================================================
# CMake configuration file for the Chrono CSharp module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_CSHARP "Enable the Chrono CSharp module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_CSHARP)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  return()
endif()

message(STATUS "==== Chrono CSharp module ====")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

# Python package name
set(CHPY_PACKAGENAME cschrono)

#-----------------------------------------------------------------------------
# CMake policy settings
# TODO: switch to new behavior
#-----------------------------------------------------------------------------

# this to fix warnings about the use of LOCATION with CMake version >= 3.0
if(POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
endif()

# this to fix warnings about the use of SWIG_MODULE_${mymodulename}_REAL_NAME with CMake version >= 3.13
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 OLD)
endif()

# UseSWIG honors SWIG_MODULE_NAME via -module flag.
if(POLICY CMP0086)
	cmake_policy(SET CMP0086 OLD)
endif()

#-----------------------------------------------------------------------------
# Enable debugging CMake output
#-----------------------------------------------------------------------------

set(DBG_SCRIPT false)

#-----------------------------------------------------------------------------
# Find SWIG
#-----------------------------------------------------------------------------

message(STATUS "...find SWIG")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

# The generated .cxx wrapper can be so huge that the /bigobj flag is required in VC++
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
endif()

# Set location of SWIG-generated files.
# Note that *.py files are also generated in this location.
set(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/swig")
SET(CMAKE_SWIG_FLAGS "-c++")

if(DBG_SCRIPT)
  message("SWIG_USE_FILE:      ${SWIG_USE_FILE}")
  message("CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")
  message("Installation prefix path:  ${CMAKE_INSTALL_PREFIX}")
endif()

#-----------------------------------------------------------------------------
# MODULE for the core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "...add Chrono::CSharp CORE module")

# Python module name
set(CHPY_CORE core)

# Interface files
set(CHPY_CORE_MODULE_FILE
    ChModuleCore.i
    )
set(CHPY_CORE_WRAPPER_FILES
    ChException.i
    ChCoordsys.i
    ChVector.i
    ChQuaternion.i
    )

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY ON)
source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})

# Create the SWIG module.
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  SWIG_ADD_MODULE(${CHPY_CORE} csharp ${CHPY_CORE_MODULE_FILE}) 
else()
  SWIG_ADD_LIBRARY(${CHPY_CORE} LANGUAGE csharp SOURCES ${CHPY_CORE_MODULE_FILE})
endif()
SWIG_LINK_LIBRARIES(${CHPY_CORE} ${PYTHON_LIBRARY} ChronoEngine)

# Ensure that the PYD library file is generated in the bin/ directory.
set_target_properties(${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} PROPERTIES 
                      PROJECT_LABEL "ChronoEngine_csharp_${CHPY_CORE}"
                      OUTPUT_NAME   "${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}"
                      LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                      )

target_compile_definitions(${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

ADD_DEPENDENCIES (${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} ChronoEngine)




if (DBG_SCRIPT)
  message("Module name:     ${CHPY_CORE}")
  message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}")
endif()
