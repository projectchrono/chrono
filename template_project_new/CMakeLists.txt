#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on Chrono and on optional Chrono modules.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
 

cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)
#--------------------------------------------------------------
# === 1 === 
# Set the project name
#--------------------------------------------------------------

project(my_project)


LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
# find_package(OpenMP)
# find_package(Eigen3)
find_package(Chrono COMPONENTS Irrlicht PardisoMKL)

# include(${INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/MathFunctions/MathFunctionTargets.cmake)


#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

add_executable(my_demo my_example.cpp)
target_link_libraries(my_demo PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_pardisomkl)

add_DLL_copy_command()
