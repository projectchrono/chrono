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

if (BUILD_SHARED_LIBS)
  message(NOTICE "For FMI support, Chrono must be configured and built in static mode (as suggested by FMI reference).")
  message("Chrono FMI module disabled.")
  set(ENABLE_MODULE_FMI OFF CACHE BOOL "Enable the Chrono::FMI module" FORCE)
  return()
endif()

set(FMU_TOOLS_DIR "${CMAKE_SOURCE_DIR}/../../fmu_tools/" CACHE PATH "Path to fmu_tools.")

include(FetchContent)
