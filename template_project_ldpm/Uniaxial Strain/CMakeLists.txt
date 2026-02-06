#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on Chrono and on optional Chrono modules.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
 

cmake_minimum_required(VERSION 3.10)

#--------------------------------------------------------------
# === 1 === 
# Modify the project name if you want: 
#--------------------------------------------------------------

project(fea_Beams_Static)

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#                 COMPONENTS req_module1 req_module1 ...
#                 OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (their names
# are case insensitive): Cascade, Cosimulation, FEA, Irrlicht,
# Matlab, Parallel, Postprocess, Python, Vehicle.
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the ChronoConfig.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# CHRONO_FOUND
#   set to true if Chrono and all required components were found
# CHRONO_C_FLAGS
# CHRONO_CXX_FLAGS
#   C and C++ compilation flags
# CHRONO_INCLUDE_DIRS
#   additional paths for included headers
# CHRONO_LIBRARIES
#   list of required libraries (with full path)
# CHRONO_LINKER_FLAGS
#   additional linker flags
# CHRONO_DLLS
#   list of all DLL dependencies (with full path)
# CHRONO_DATA_DIR
#   path to the Chrono data make_directory
# 
# In addition, for each requested component [COMPONENT], the
# following variable is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
# and, for demonstration purposes, the PardisoMKL module (optional)
#--------------------------------------------------------------

#LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
LIST(APPEND CMAKE_PREFIX_PATH "/chrono-concrete/build/lib")
find_package(Chrono	     
             COMPONENTS Irrlicht LDPM 
             OPTIONAL_COMPONENTS PardisoMKL 
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

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
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------

#add_executable(fea_demo ChBuilderBeamCSL.cpp ChElementCSL.cpp ChBeamSectionCSL.cpp demo_FEA_beams_static.cpp )
#add_executable(fea_demo MyChStaticAnalysis.cpp ChMaterialVECT.cpp ChBuilderCSL.cpp ChElementCSL.cpp ChBeamSectionCSL_V3.cpp CSL_UC_1tet.cpp )
#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp CSL_UC.cpp)
#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp LDPM_UC_Loadingplate.cpp)
#add_executable(fea_demo ChSystem.cpp ChSystemSMC.cpp ChTimestepper.cpp ChDirectSolverLS.cpp ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp LDPM_UC_1tet_KEU.cpp)
#add_executable(fea_demo ChMaterialVECT.cpp ChBuilderCSL.cpp ChElementCSL.cpp ChBeamSectionCSL_V3.cpp CSL_UC_tiePlate.cpp)
#add_executable(fea_demo ChSystem.cpp ChSystemSMC.cpp ChTimestepper.cpp ChDirectSolverLS.cpp ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp LDPM_UC_contact.cpp) 
#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp TET_UC_contact.cpp) 
#add_executable(fea_demo demo_FEA_contacts_SMC.cpp)
#add_executable(fea_demo demo_FEA_TriFace.cpp)
#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp LDPM_UC_Loadingplate.cpp)
#add_executable(fea_demo ChAssembly.cpp ChMesh.cpp ChElementGeneric.cpp ChSystem.cpp ChSystemSMC.cpp ChTimestepper.cpp ChDirectSolverLS.cpp ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp ChLinkPointTrifacexyz2xyzrot.cpp LDPM_TPBT.cpp)
#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp ChLinkPointTrifacexyz2xyzrot.cpp SOLID_TPBT.cpp)

#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM_RVE.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp ChContactSurfaceMesh.cpp ChLinkNodeFaceRot.cpp ChMeshLine.cpp ChLinkPoint2Line.cpp ChRepresentedVolumeElement.cpp LDPM_RVE.cpp)

add_executable(fea_demo LDPM_tri.cpp)

#add_executable(fea_demo ChMaterialVECT.cpp ChElementLDPM_RVE.cpp ChSectionLDPM.cpp ChBuilderLDPM.cpp ChContactSurfaceMesh.cpp ChLinkNodeFaceRot.cpp ChMeshLine.cpp ChLinkPoint2Line.cpp demo_FEA_beamsEuler.cpp)
#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

target_compile_definitions(fea_demo PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(fea_demo PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(fea_demo PUBLIC ${CHRONO_LINKER_FLAGS})

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

target_link_libraries(fea_demo ${CHRONO_LIBRARIES})

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
add_DLL_copy_command("${CHRONO_DLLS}")
