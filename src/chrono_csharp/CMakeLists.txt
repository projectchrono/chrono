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

cmake_minimum_required(VERSION 3.8.0)
mark_as_advanced(CLEAR SWIG_EXECUTABLE)

#-----------------------------------------------------------------------------
# CMake policy settings
# TODO: switch to new behavior
#-----------------------------------------------------------------------------

# Fix warnings about the use of SWIG_MODULE_${name}_REAL_NAME with CMake version >= 3.13
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 OLD)
endif()

# UseSWIG honors SWIG_MODULE_NAME via -module flag.
if(POLICY CMP0086)
	cmake_policy(SET CMP0086 OLD)
endif()

#-----------------------------------------------------------------------------
# Find SWIG
#-----------------------------------------------------------------------------

message(STATUS "...find SWIG")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

# Add /bigobj flag to deal with large generated .cxx wrappers
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
endif()

# Set location of SWIG-generated files.
set(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/swig_csharp")
SET(CMAKE_SWIG_FLAGS "-c++")

message(STATUS "SWIG_USE_FILE:      ${SWIG_USE_FILE}")
message(STATUS "CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")

#-----------------------------------------------------------------------------
# MODULE for the core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "...add Chrono::CSharp CORE module")

# Set module name.
set(MODULE_CORE ChronoEngine_csharp_core)

# Set interface file.
set(INTERFACE_FILE core/ChModuleCore.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${INTERFACE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${INTERFACE_FILE} PROPERTIES CPLUSPLUS ON)
set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY ON)
source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})

# Create the SWIG module.
SWIG_ADD_LIBRARY(${MODULE_CORE} LANGUAGE csharp SOURCES ${INTERFACE_FILE})
SWIG_LINK_LIBRARIES(${MODULE_CORE} ChronoEngine)

# Ensure that the resultiung library is generated in the bin/ directory.
set_target_properties(${SWIG_MODULE_${MODULE_CORE}_REAL_NAME} PROPERTIES 
                      PROJECT_LABEL "${MODULE_CORE}"
                      OUTPUT_NAME   "${SWIG_MODULE_${MODULE_CORE}_REAL_NAME}"
                      LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                      )

target_compile_definitions(${SWIG_MODULE_${MODULE_CORE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

ADD_DEPENDENCIES (${SWIG_MODULE_${MODULE_CORE}_REAL_NAME} ChronoEngine)


message(STATUS "Module name:     ${MODULE_CORE}")
message(STATUS "SWIG_REAL_NAME:  ${SWIG_MODULE_${MODULE_CORE}_REAL_NAME}")
