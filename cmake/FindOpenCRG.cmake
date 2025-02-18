
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

# Try to find OpenCRG library in default paths + user specified paths
find_path(OpenCRG_INCLUDE_DIR_INTERNAL NAMES crgBaseLib.h PATHS "${OpenCRG_INCLUDE_DIR}")

if(NOT OpenCRG_INCLUDE_DIR_INTERNAL)
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "The provided OpenCRG include directory does not contain crgBaseLib.h")
  endif()
  set(OpenCRG_FOUND FALSE)
  return()
else()
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Found crgBaseLib.h in provided include directory")
  endif()
endif()

if(NOT EXISTS "${OpenCRG_LIBRARY}")
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "The provided OpenCRG library does not exist")
  endif()
  set(OpenCRG_FOUND FALSE)
  return()
else()
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Found OpenCRG library file")
  endif()
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  find_file(OpenCRG_DLL_INTERNAL NAMES OpenCRG.dll PATHS "${OpenCRG_DLL}")
  if(NOT OpenCRG_DLL_INTERNAL)
    if(NOT OpenCRG_FIND_QUIETLY)
      message(STATUS "OpenCRG DLL not found; assuming static OpenCRG library")
    endif()
    set(OpenCRG_SHARED FALSE)
  else()
    if(NOT OpenCRG_FIND_QUIETLY)
      message(STATUS "OpenCRG DLL found; assuming shared OpenCRG library")
    endif()
    set(OpenCRG_SHARED TRUE)
  endif()
endif()

mark_as_advanced(FORCE OpenCRG_INCLUDE_DIR_INTERNAL OpenCRG_DLL_INTERNAL)

# Try to locate libraries for multiple configurations
get_filename_component(OpenCRG_LIBRARY_DIR ${OpenCRG_LIBRARY} DIRECTORY)

find_library(OpenCRG_LIBRARY_RELEASE NAMES OpenCRG PATHS "${OpenCRG_LIBRARY_DIR}")
find_library(OpenCRG_LIBRARY_DEBUG NAMES OpenCRG_d PATHS "${OpenCRG_LIBRARY_DIR}" PATHS ${OpenCRG_LIBRARY_DIR})
find_library(OpenCRG_LIBRARY_MINSIZEREL NAMES OpenCRG_s PATHS "${OpenCRG_LIBRARY_DIR}" PATHS ${OpenCRG_LIBRARY_DIR})
find_library(OpenCRG_LIBRARY_RELWITHDEBINFO NAMES OpenCRG_rd PATHS "${OpenCRG_LIBRARY_DIR}" PATHS ${OpenCRG_LIBRARY_DIR})

if(OpenCRG_LIBRARY_DEBUG)
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Found Debug OpenCRG library")
  endif()
else()
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Debug OpenCRG library not found; using Release library")
  endif()
  set(OpenCRG_LIBRARY_DEBUG ${OpenCRG_LIBRARY_RELEASE})
endif()

if(OpenCRG_LIBRARY_MINSIZEREL)
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Found MinSizeRel OpenCRG library")
  endif()
else()
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "MinSizeRel OpenCRG library not found; using Release library")
  endif()
  set(OpenCRG_LIBRARY_MINSIZEREL ${OpenCRG_LIBRARY_RELEASE})
endif()

if(OpenCRG_LIBRARY_RELWITHDEBINFO)
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Found RelWithDebInfo OpenCRG library")
  endif()
else()
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "RelWithDebInfo OpenCRG library not found; using Release library")
  endif()
  set(OpenCRG_LIBRARY_RELWITHDEBINFO ${OpenCRG_LIBRARY_RELEASE})
endif()

mark_as_advanced(FORCE OpenCRG_LIBRARY_RELEASE OpenCRG_LIBRARY_DEBUG OpenCRG_LIBRARY_MINSIZEREL OpenCRG_LIBRARY_RELWITHDEBINFO)

# Set OpenCRG as found
set(OpenCRG_FOUND TRUE)

# Create target

if(OpenCRG_FOUND AND NOT TARGET OpenCRG::OpenCRG)
  if(NOT OpenCRG_FIND_QUIETLY)
    message(STATUS "Create target OpenCRG::OpenCRG")
  endif()

  if (OpenCRG_SHARED)
    add_library(OpenCRG::OpenCRG SHARED IMPORTED)
  else()
    add_library(OpenCRG::OpenCRG STATIC IMPORTED)
  endif()

  set_target_properties(OpenCRG::OpenCRG PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OpenCRG_INCLUDE_DIR}")

  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION "${OpenCRG_LIBRARY}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_RELEASE "${OpenCRG_LIBRARY_RELEASE}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_DEBUG "${OpenCRG_LIBRARY_DEBUG}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_RELWITHDEBINFO "${OpenCRG_LIBRARY_RELWITHDEBINFO}")
  set_target_properties(OpenCRG::OpenCRG PROPERTIES IMPORTED_LOCATION_MINSIZEREL "${OpenCRG_LIBRARY_MINSIZEREL}")
endif()
