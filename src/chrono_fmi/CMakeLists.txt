#=============================================================================
# CMake configuration file for the Chrono::FMI module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_FMI "Enable the Chrono::FMI module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_FMI)
	return()
endif()

MESSAGE(STATUS "\n==== Chrono FMI module ====\n")

# -------------------------------------------------

if (BUILD_SHARED_LIBS)
  message(NOTICE "For FMI support, Chrono must be configured and built in static mode (as suggested by FMI reference). Disable 'BUILD_SHARED_LIBS'.")
  message(NOTICE "Chrono FMI module disabled.")
  set(ENABLE_MODULE_FMI OFF CACHE BOOL "Enable the Chrono::FMI module" FORCE)
  return()
endif()

if(MSVC)
  if (NOT USE_MSVC_STATIC_RUNTIME)
    message(NOTICE "For FMI support on Windows, Chrono must be configured and built with a statically-linked runtime library. Enable 'USE_MSVC_STATIC_RUNTIME'.")
    message(NOTICE "Chrono FMI module disabled.")
    set(ENABLE_MODULE_FMI OFF CACHE BOOL "Enable the Chrono::FMI module" FORCE)
    return()
  endif()
endif()

set(FMU_TOOLS_DIR "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/fmu_tools/" CACHE INTERNAL "Path to fmu_tools.")

# -------------------------------------------------
# Set Architecture and Operating System Flags
# -------------------------------------------------
set(UNKNOWN_TARGET_SYSTEM FALSE)
if(WIN32)
    set(FMI_PLATFORM "win")
elseif(APPLE)
    set(FMI_PLATFORM "darwin")
elseif(CYGWIN)
    set(FMI_PLATFORM "cygwin")
elseif(UNIX)
    set(FMI_PLATFORM "linux")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(FMI_PLATFORM "${FMI_PLATFORM}64")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(FMI_PLATFORM "${FMI_PLATFORM}32")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(UNKNOWN_TARGET_SYSTEM)
    message(WARNING "Cannot retrieve target operating system or architecture. Please specify it manually through TARGET_SYSTEM variable (e.g. win64)")
    set(TARGET_SYSTEM "win64" CACHE STRING "Target operating system and architecture (e.g. win64, linux32, ...)")
    mark_as_advanced(CLEAR TARGET_SYSTEM)
else()
    mark_as_advanced(FORCE TARGET_SYSTEM)
endif()

set(FMI_PLATFORM "${FMI_PLATFORM}" PARENT_SCOPE)

message(STATUS "FMI platform:         ${FMI_PLATFORM}")
message(STATUS "FMU Tools directory:  ${FMU_TOOLS_DIR}")

# -------------------------------------------------

include(FetchContent)

