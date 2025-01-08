
#-------------------------------------------------------------------------------
# Find OpenCRG
# This find script requires the following input variables:
# OpenCRG_INCLUDE_DIR: location of headers
# OpenCRG_LIBRARY: location of import library
# OpenCRG_DLL: location of dll (Win only)
# This find script checks the existence of OpenCRG library and sets the following:
# OpenCRG_FOUND: if OpenCRG has been found
# OpenCRG_SHARED: true if OpenCRG is a shared library
# OpenCRG::OpenCRG: imported target
#-------------------------------------------------------------------------------


mark_as_advanced(FORCE OpenCRG_INCLUDE_DIR_INTERNAL OpenCRG_DLL_INTERNAL)

# Try to find OpenCRG library in default paths + user specified paths
find_path(OpenCRG_INCLUDE_DIR_INTERNAL NAMES crgBaseLib.h PATHS "${OpenCRG_INCLUDE_DIR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  find_file(OpenCRG_DLL_INTERNAL NAMES OpenCRG.dll PATHS "${OpenCRG_DLL}")
endif()

# check if library is found
if(NOT OpenCRG_INCLUDE_DIR_INTERNAL OR NOT EXISTS "${OpenCRG_LIBRARY}")
  message(ERROR "OpenCRG library cannot be found. Please set OpenCRG_INCLUDE_DIR, OpenCRG_LIBRARY variables to proper locations.")
  set(OpenCRG_FOUND FALSE)
  return()
endif()

# Try to locate libraries for multiple configurations
get_filename_component(OpenCRG_LIBRARY_DIR ${OpenCRG_LIBRARY} DIRECTORY)

find_library(OpenCRG_LIBRARY_RELEASE NAMES OpenCRG PATHS "${OpenCRG_LIBRARY_DIR}")
find_library(OpenCRG_LIBRARY_DEBUG NAMES OpenCRG_d PATHS "${OpenCRG_LIBRARY_DIR}" PATHS ${OpenCRG_LIBRARY_DIR})
find_library(OpenCRG_LIBRARY_MINSIZEREL NAMES OpenCRG_s PATHS "${OpenCRG_LIBRARY_DIR}" PATHS ${OpenCRG_LIBRARY_DIR})
find_library(OpenCRG_LIBRARY_RELWITHDEBINFO NAMES OpenCRG_rd PATHS "${OpenCRG_LIBRARY_DIR}" PATHS ${OpenCRG_LIBRARY_DIR})

if (NOT OpenCRG_LIBRARY_DEBUG)
  set(OpenCRG_LIBRARY_DEBUG ${OpenCRG_LIBRARY_RELEASE})
endif()
if (NOT OpenCRG_LIBRARY_MINSIZEREL)
  set(OpenCRG_LIBRARY_MINSIZEREL ${OpenCRG_LIBRARY_RELEASE})
endif()
if (NOT OpenCRG_LIBRARY_RELWITHDEBINFO)
  set(OpenCRG_LIBRARY_RELWITHDEBINFO ${OpenCRG_LIBRARY_RELEASE})
endif()
mark_as_advanced(FORCE OpenCRG_LIBRARY_RELEASE OpenCRG_LIBRARY_DEBUG OpenCRG_LIBRARY_MINSIZEREL OpenCRG_LIBRARY_RELWITHDEBINFO)


if(${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND NOT OpenCRG_DLL_INTERNAL)
  message(STATUS "Found OpenCRG as static library.")
  set(OpenCRG_SHARED FALSE)
else()
  set(OpenCRG_SHARED TRUE)
endif()

set(OpenCRG_FOUND TRUE)

# provide alias target
if(OpenCRG_FOUND AND NOT TARGET OpenCRG::OpenCRG)
  if (OpenCRG_SHARED)
    add_library(OpenCRG::OpenCRG SHARED IMPORTED)
  else()
    add_library(OpenCRG::OpenCRG STATIC IMPORTED)
  endif()
  set_target_properties(OpenCRG::OpenCRG PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OpenCRG_INCLUDE_DIR}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_RELEASE "${OpenCRG_LIBRARY_RELEASE}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_DEBUG "${OpenCRG_LIBRARY_DEBUG}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_RELWITHDEBINFO "${OpenCRG_LIBRARY_RELWITHDEBINFO}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_MINSIZEREL "${OpenCRG_LIBRARY_MINSIZEREL}")
endif()