cmake_minimum_required(VERSION 3.12)

project(MyCSharpProject 
        CSharp)

if(NOT Chrono_DIR)
  set(Chrono_DIR "" CACHE PATH "The directory containing a CMake configuration file for Chrono.")
  return()
endif()

find_package(Chrono
             COMPONENTS CSharp
             OPTIONAL_COMPONENTS Irrlicht
             CONFIG)

# Add C# executable
add_executable(ChronoDemo
    main.cs
    ${CHRONO_CSHARP_SOURCES}
)

# set_target_properties(ChronoDemo PROPERTIES CMAKE_DOTNET_TARGET_FRAMEWORK_VERSION "v4.8")
source_group(Chrono FILES ${CHRONO_CSHARP_SOURCE}) # doesn't work

#--------------------------------------------------------------
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This has effect ONLY on Windows.
# 
# By default, DLLs will be copied to ${PROJECT_BINARY_DIR}/bin/${config}.
# If only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set, they will be copied to
# ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config}.
# If the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set,
# DLLs will be copied to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>}.
#--------------------------------------------------------------

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
add_DLL_copy_command()

