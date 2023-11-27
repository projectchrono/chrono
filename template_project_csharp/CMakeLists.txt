cmake_minimum_required(VERSION 3.12)

project(MyCSharpProject 
        CSharp)

find_package(Chrono
             COMPONENTS CSharp
             OPTIONAL_COMPONENTS Irrlicht
             CONFIG)


# Gather .cs files
file(GLOB CHRONO_CSHARP_SOURCE 
     LIST_DIRECTORIES false CONFIGURE_DEPENDS "${CHRONO_CSHARP_SOURCE_DIR}/*.cs")


# Add C# executable
add_executable(ChronoDemo
    main.cs
    ${CHRONO_CSHARP_SOURCE}
)

# set_target_properties(ChronoDemo PROPERTIES CMAKE_DOTNET_TARGET_FRAMEWORK_VERSION "v4.8")
source_group(Chrono FILES ${CHRONO_CSHARP_SOURCE}) # doesn't work

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# DLLs will be copied into ${PROJECT_BINARY_DIR}/bin/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")
add_DLL_copy_command()

