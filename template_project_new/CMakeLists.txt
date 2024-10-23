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
find_package(Chrono COMPONENTS Irrlicht PardisoMKL MultiCore)


#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

add_executable(my_demo my_example.cpp)
set_target_properties(my_demo PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
target_link_libraries(my_demo PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht)

add_executable(demo_FEA_cablesMKL demo_FEA_cablesMKL.cpp)
set_target_properties(demo_FEA_cablesMKL PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
target_link_libraries(demo_FEA_cablesMKL PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_pardisomkl)

add_executable(demo_MCORE_cratering demo_MCORE_cratering.cpp)
set_target_properties(demo_MCORE_cratering PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
target_link_libraries(demo_MCORE_cratering PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_pardisomkl ChronoEngine::ChronoEngine_multicore)

add_DLL_copy_command()
