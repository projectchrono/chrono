
#-------------------------------------------------------------------------------
# Find OpenCRG
# This find script requires the following input variables:
# OpenCRG_INCLUDE_DIR: location of headers
# OpenCRG_LIBRARY: location of import library
# OpenCRG_DLL: location of dll (Win only)
# This find script checks the existence of OpenCRG library and sets the following:
# OpenCRG_FOUND: if OpenCRG has been found
# OpenCRG::OpenCRG: imported target
#-------------------------------------------------------------------------------




# Try to find OpenCRG library in default paths + user specified paths
find_path(OpenCRG_INCLUDE_DIR_INTERNAL NAMES crgBaseLib.h PATHS "${OpenCRG_INCLUDE_DIR}")
find_library(OpenCRG_LIBRARY_INTERNAL NAMES OpenCRG PATHS "${OpenCRG_LIBRARY}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  find_file(OpenCRG_DLL_INTERNAL NAMES OpenCRG.dll PATHS "${OpenCRG_DLL}")
endif()

# check if library is found
if((${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND NOT OpenCRG_DLL_INTERNAL) OR NOT OpenCRG_INCLUDE_DIR_INTERNAL OR NOT OpenCRG_LIBRARY_INTERNAL)
  message(WARNING "OpenCRG library cannot be found. Please set OpenCRG_INCLUDE_DIR, OpenCRG_LIBRARY and OpenCRG_DLL variables to proper locations.")
  set(OpenCRG_FOUND FALSE)
  return()
endif()

set(OpenCRG_FOUND TRUE)

# provide alias target
if(OpenCRG_FOUND AND NOT TARGET OpenCRG::OpenCRG)
  add_library(OpenCRG::OpenCRG SHARED IMPORTED)
  set_target_properties(OpenCRG::OpenCRG PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OpenCRG_INCLUDE_DIR}")
endif()