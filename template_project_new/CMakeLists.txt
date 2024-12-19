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
find_package(Chrono COMPONENTS Irrlicht PardisoMKL MultiCore GPU)


#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Add executables
#--------------------------------------------------------------

add_executable(my_demo my_example.cpp)
target_link_libraries(my_demo PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht)

add_executable(demo_FEA_cablesMKL demo_FEA_cablesMKL.cpp)
target_link_libraries(demo_FEA_cablesMKL PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_pardisomkl)

add_executable(demo_MCORE_cratering demo_MCORE_cratering.cpp)
target_link_libraries(demo_MCORE_cratering PRIVATE ChronoEngine::ChronoEngine_multicore) # ChronoEngine::ChronoEngine might be omitted since it is pulled directly by ChronoEngine_multicore

add_executable(demo_GPU_mixer demo_GPU_mixer.cpp)
target_link_libraries(demo_GPU_mixer PRIVATE ChronoEngine::ChronoEngine_gpu)

add_executable(demo_FSI_Poiseuille_flow demo_FSI_Poiseuille_flow.cpp)
target_link_libraries(demo_FSI_Poiseuille_flow PRIVATE ChronoEngine::ChronoEngine_fsi)






add_DLL_copy_command()
