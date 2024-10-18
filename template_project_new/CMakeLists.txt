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
find_package(Chrono COMPONENTS Irrlicht)

# include(${INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/MathFunctions/MathFunctionTargets.cmake)


#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

# #--------------------------------------------------------------
# # Important! To ensure ABI compatibility, use the same C++ standard
# # as the one used to build the Chrono libraries.
# #--------------------------------------------------------------

# set(CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

# #--------------------------------------------------------------
# # Enable creation of "application bundles" on MacOSX.
# #--------------------------------------------------------------

# # This is necessary for any Irrlicht-based project (like the example here).
# # For OpenGL-based or non-graphics projects, this is optional and the block
# # below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
# #
# # If creating application bundles, the build output will be named 'myexe.app'.
# # Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# # to run:
# #     start_demo.sh myexe.app

# if(APPLE)
#     set(CMAKE_MACOSX_BUNDLE ON)
# endif()

# #--------------------------------------------------------------
# # Add path to Chrono headers and to headers of all dependencies
# # of the requested modules.
# #--------------------------------------------------------------

# include_directories(${CHRONO_INCLUDE_DIRS})

# #-----------------------------------------------------------------------------
# # Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
# #-----------------------------------------------------------------------------

# if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#   if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
#     add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
#   endif()
# endif()

# #--------------------------------------------------------------
# # Tweaks to disable some warnings with MSVC
# #--------------------------------------------------------------
# if(MSVC)
#     add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
#     add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
#     add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
# endif()

# #--------------------------------------------------------------
# # === 3 ===
# # Add the executable from your project and specify all C++ 
# # files in your project. 
# #--------------------------------------------------------------

add_executable(my_demo my_example.cpp)

#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

# target_compile_definitions(my_demo PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
# target_compile_options(my_demo PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(my_demo PUBLIC ${CHRONO_LINKER_FLAGS})

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

target_link_libraries(my_demo PRIVATE ChronoEngine::ChronoEngine ChronoEngine::ChronoEngine_irrlicht)

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# DLLs will be copied into ${PROJECT_BINARY_DIR}/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")
# add_DLL_copy_command()
