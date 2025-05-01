cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

# Also enable C++ to have access to Chrono features that require C++ (e.g., OpenMP for C++)
project(MyCSharpProject CSharp CXX)

if(NOT Chrono_DIR)
  set(Chrono_DIR "" CACHE PATH "The directory containing a CMake configuration file for Chrono.")
  return()
endif()

# Search for the Chrono package with required components CSharp and Irrlicht
find_package(Chrono
             COMPONENTS CSharp Irrlicht)

# Return if Chrono or a required component was not found
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

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
#--------------------------------------------------------------

add_DLL_copy_command()
