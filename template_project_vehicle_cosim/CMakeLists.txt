#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on the Chrono::Vehicle cosimulation framework.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
 
cmake_minimum_required(VERSION 3.10)

#--------------------------------------------------------------
# === 1 === 
# Set the project name
#--------------------------------------------------------------

project(vehicle_cosim)

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#	                COMPONENTS req_module1 req_module1 ...
#	                OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono components can be requested (case insensitive): 
#   Cascade, Cosimulation, Irrlicht, OpenGL, Matlab, Multicore, Gpu,
#   PardisoMKL, PardisoProject, Postprocess, Python, Vehicle,
#   VehicleCosimm, VSG.
# A component can be requested either as required or optional
# (see the CMake documentation for find_package).
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the chrono-config.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# Here, we specfy the VehicleCosim module (required).
#--------------------------------------------------------------

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS VehicleCosim
             CONFIG)

# Return now if Chrono or a required component was not found.
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Important! To ensure ABI compatibility, use the same C++ standard
# as the one used to build the Chrono libraries.
#--------------------------------------------------------------

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
endif()

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------

add_executable(vehicle_cosim vehicle_cosim.cpp)

#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

target_compile_definitions(vehicle_cosim PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(vehicle_cosim PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(vehicle_cosim PUBLIC ${CHRONO_LINKER_FLAGS})

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

target_link_libraries(vehicle_cosim ${CHRONO_LIBRARIES})

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command()
