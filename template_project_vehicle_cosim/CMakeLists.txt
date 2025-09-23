#--------------------------------------------------------------
# CMake configuration script for linking a vehicle co-simulation
# external project to Chrono libraries.
# For a particular user project, modify sections 1 - 4 below,
# as appropriate.
#--------------------------------------------------------------
 
cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

#--------------------------------------------------------------
# 1. Set the project name
#--------------------------------------------------------------

project(vehicle_cosim)

#--------------------------------------------------------------
# 2. Find REQUIRED and OPTIONAL Chrono modules
#
# Invoke the find_package function in CONFIG mode:
#    find_package(Chrono
#	                COMPONENTS req_module1 req_module1 ...
#	                OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (case insensitive):
#   Cascade, CSharp, FMI, FSI, DEM, Irrlicht, OpenGL, VSG, Matlab,
#   Modal, Multicore, PardisoMKL, Parsers, Postprocess, Sensor,
#   Synchrono, Vehicle, VehicleCosim.
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

find_package(Chrono
             COMPONENTS VehicleCosim
             OPTIONAL_COMPONENTS Irrlicht
             CONFIG)

# Return now if Chrono or a required component was not found.
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# 3. Specify project sources and add executable
#--------------------------------------------------------------

add_executable(vehicle_cosim vehicle_cosim.cpp)

#--------------------------------------------------------------
# Set properties for the executable target
#--------------------------------------------------------------

target_compile_definitions(vehicle_cosim PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 

if(MSVC)
    set_target_properties(vehicle_cosim PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()

#--------------------------------------------------------------
# Link to Chrono targets for the requested modules
#--------------------------------------------------------------

target_link_libraries(vehicle_cosim PRIVATE ${CHRONO_TARGETS})
target_link_libraries(vehicle_cosim PRIVATE MPI::MPI_CXX)

#--------------------------------------------------------------
# 4. OPTIONAL (Windows only)
# 
# Add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
#--------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_CHRONO_DLLS_copy_command()
