cmake_minimum_required(VERSION 3.12)

project(ChronoDemos CSharp)

if(NOT Chrono_DIR)
  set(Chrono_DIR "" CACHE PATH "The directory containing a CMake configuration file for Chrono.")
  return()
endif()

# Keep track of all DLLs. Each submodule should append to these list
list(APPEND ALL_DLL_NAMES "")
list(APPEND ALL_DEPENDENCY_DLLS "")

# Direct binary output in the same place (where DLLS will also be copied) 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# Traverse subdirectories
add_subdirectory(core)
#add_subdirectory(irrlicht)
add_subdirectory(mbs)
#add_subdirectory(vehicle)

# Add custom command to copy all collected DLLs
add_DLL_copy_command("${ALL_DLL_NAMES}" "${ALL_DEPENDENCY_DLLS}")
