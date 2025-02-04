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

project(my_project CXX)

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono COMPONENTS Irrlicht PardisoMKL MultiCore FSI GPU OpenGL Vehicle vsg sensor parsers synchrono modal)


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
# Set the CUDA architectures for all targets; if undesired set CUDA_ARCHITECTURES property for each target
set(CMAKE_CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})


add_executable(my_demo my_example.cpp)
target_link_libraries(my_demo PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht)

add_executable(demo_FEA_cablesMKL demo_FEA_cablesMKL.cpp)
target_link_libraries(demo_FEA_cablesMKL PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_pardisomkl)

add_executable(demo_MCORE_cratering demo_MCORE_cratering.cpp)
target_link_libraries(demo_MCORE_cratering PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_multicore)

add_executable(demo_MCORE_cratering_OpenGL demo_MCORE_cratering_OpenGL.cpp)
target_link_libraries(demo_MCORE_cratering_OpenGL PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_multicore ChronoEngine::ChronoEngine_opengl)

add_executable(demo_GPU_mixer demo_GPU_mixer.cpp)
target_link_libraries(demo_GPU_mixer PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_gpu ChronoEngine::ChronoEngine_gpu_opengl)

add_executable(demo_FSI_Poiseuille_flow demo_FSI_Poiseuille_flow.cpp)
target_link_libraries(demo_FSI_Poiseuille_flow PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_fsisph ChronoEngine::ChronoEngine_fsisph_vsg ChronoEngine::ChronoEngine_fsisph_opengl)

add_executable(demo_FSI_WaveTank demo_FSI_WaveTank.cpp)
target_link_libraries(demo_FSI_WaveTank PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_fsisph ChronoEngine::ChronoEngine_fsisph_vsg ChronoEngine::ChronoEngine_fsisph_opengl)

add_executable(demo_GL_cohesion demo_GL_cohesion.cpp)
target_link_libraries(demo_GL_cohesion PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_opengl)

add_executable(demo_VEH_TrackTestRig demo_VEH_TrackTestRig.cpp)
target_link_libraries(demo_VEH_TrackTestRig PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_vehicle ChronoEngine::ChronoEngine_vehicle_irrlicht ChronoEngine::ChronoModels_vehicle)

if (CHRONO_VEHICLE_OPENCRG)
  add_executable(demo_VEH_CRGTerrain_IRR demo_VEH_CRGTerrain_IRR.cpp)
  target_link_libraries(demo_VEH_CRGTerrain_IRR PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_vehicle ChronoEngine::ChronoEngine_vehicle_irrlicht ChronoEngine::ChronoModels_vehicle)
  message(STATUS "OpenCRG support enabled")
else()
  message(STATUS "OpenCRG support disabled")
endif()

add_executable(demo_VEH_RenderJSON_VSG demo_VEH_RenderJSON_VSG.cpp)
target_link_libraries(demo_VEH_RenderJSON_VSG PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_vsg ChronoEngine::ChronoEngine_vehicle ChronoEngine::ChronoEngine_vehicle_vsg ChronoEngine::ChronoModels_vehicle)

add_executable(demo_ROBOT_RoboSimian_Granular demo_ROBOT_RoboSimian_Granular.cpp granular.h granular.cpp)
target_link_libraries(demo_ROBOT_RoboSimian_Granular PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_opengl ChronoEngine::ChronoEngine_vehicle ChronoEngine::ChronoEngine_vehicle_opengl ChronoEngine::ChronoModels_robot ChronoEngine::ChronoEngine_multicore)

add_executable(demo_SEN_Gator demo_SEN_Gator.cpp)
target_link_libraries(demo_SEN_Gator PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoEngine_vehicle ChronoEngine::ChronoEngine_vehicle_irrlicht ChronoEngine::ChronoModels_vehicle ChronoEngine::ChronoEngine_sensor)

add_executable(demo_PARSER_Python demo_PARSER_Python.cpp)
target_link_libraries(demo_PARSER_Python PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_parsers)

add_executable(demo_SYN_copter demo_SYN_copter.cpp)
target_link_libraries(demo_SYN_copter PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_vehicle ChronoEngine::ChronoEngine_synchrono ChronoEngine::ChronoEngine_irrlicht ChronoEngine::ChronoModels_vehicle ChronoEngine::ChronoModels_robot)

add_executable(demo_MOD_slewing_beam demo_MOD_slewing_beam.cpp)
target_link_libraries(demo_MOD_slewing_beam PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_modal)
if(CHRONO_IRRLICHT_FOUND)
  target_link_libraries(demo_MOD_slewing_beam PRIVATE ChronoEngine::ChronoEngine_irrlicht)
endif()
if(CHRONO_PARDISOMKL_FOUND)
  target_link_libraries(demo_MOD_slewing_beam PRIVATE ChronoEngine::ChronoEngine_pardisomkl)
endif()
if(CHRONO_MATLAB_FOUND)
  target_link_libraries(demo_MOD_slewing_beam PRIVATE ChronoEngine::ChronoEngine_matlab)
endif()
if(CHRONO_POSTPROCESS_FOUND)
  target_link_libraries(demo_MOD_slewing_beam PRIVATE ChronoEngine::ChronoEngine_postprocess)
endif()



add_DLL_copy_command()
