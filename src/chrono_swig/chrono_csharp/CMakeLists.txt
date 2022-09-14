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

message(STATUS "\n==== Chrono CSharp module ====\n")

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
  set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS}   /bigobj")
endif()

# Set location of SWIG-generated files.
set(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/chrono_csharp")

# Set SWIG flags.  Disable selected SWIG warnings 
set(CMAKE_SWIG_FLAGS "-c++;-w401,503,833")

message(STATUS "SWIG_USE_FILE:      ${SWIG_USE_FILE}")
message(STATUS "CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")

# Disable additional warnings for generated C++ code
if(MSVC)
   add_compile_options(/wd4189)  # local variable is initialized but not referenced
   add_compile_options(/wd4456)  # declaration hides previous local declaration
   add_compile_options(/wd4702)  # unreachable code
   add_compile_options(/wd4706)  # assignment within conditional expression
endif()

#-----------------------------------------------------------------------------

set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

# Output library destination
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(LIB_OUT_DIR ${EXECUTABLE_OUTPUT_PATH})
else()
    set(LIB_OUT_DIR "${PROJECT_BINARY_DIR}/lib")
endif()

#-----------------------------------------------------------------------------
# Wrapper for CORE Chrono module
#-----------------------------------------------------------------------------

message(STATUS "...add CSharp CORE module")

# Set module name.
set(MODULE_CORE ChronoEngine_csharp_core)

# Set interface file.
set(INTERFACE_FILE_CORE ../interface/core/ChModuleCore.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES CPLUSPLUS ON)
##set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY ON)
##source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})

# Create the SWIG module.
SWIG_ADD_LIBRARY(${MODULE_CORE} LANGUAGE csharp SOURCES ${INTERFACE_FILE_CORE})
SWIG_LINK_LIBRARIES(${MODULE_CORE} ChronoEngine)

set_target_properties(${SWIG_MODULE_${MODULE_CORE}_REAL_NAME} PROPERTIES 
                      PROJECT_LABEL "${MODULE_CORE}"
                      OUTPUT_NAME   "${SWIG_MODULE_${MODULE_CORE}_REAL_NAME}"
                      LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                      )

target_compile_definitions(${SWIG_MODULE_${MODULE_CORE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_CORE}_REAL_NAME} ChronoEngine)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS ChronoEngine_csharp_core
            LIBRARY DESTINATION bin)
else()
    install(TARGETS ChronoEngine_csharp_core
            LIBRARY DESTINATION lib)
endif()

message(STATUS "Module name:     ${MODULE_CORE}")
message(STATUS "SWIG_REAL_NAME:  ${SWIG_MODULE_${MODULE_CORE}_REAL_NAME}")

#-----------------------------------------------------------------------------
# Wrapper for VEHICLE Chrono module
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_VEHICLE)

  message(STATUS "...add CSharp VEHICLE module")
  
  # Set module name.
  set(MODULE_VEHICLE ChronoEngine_csharp_vehicle)
  
  # Set interface file.
  set(INTERFACE_FILE_VEHICLE ../interface/vehicle/ChModuleVehicle.i)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES CPLUSPLUS ON)
  
  # Create the SWIG module.
  SWIG_ADD_LIBRARY(${MODULE_VEHICLE} LANGUAGE csharp SOURCES ${INTERFACE_FILE_VEHICLE})
  SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoEngine)
  SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoEngine_vehicle)
  SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoModels_vehicle)
  
  set_target_properties(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "${MODULE_VEHICLE}"
                        OUTPUT_NAME   "${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        )
  
  target_compile_definitions(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")
  
  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ChronoEngine_vehicle)
  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ChronoModels_vehicle)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      install(TARGETS ChronoEngine_csharp_vehicle
              LIBRARY DESTINATION bin)
  else()
      install(TARGETS ChronoEngine_csharp_vehicle
              LIBRARY DESTINATION lib)
  endif()
  
  message(STATUS "Module name:     ${MODULE_VEHICLE}")
  message(STATUS "SWIG_REAL_NAME:  ${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME}")

endif()

#-----------------------------------------------------------------------------
# Install swig-generate C# files

install(DIRECTORY ${CMAKE_SWIG_OUTDIR}/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")
