#-------------------------------------------------------------------------------
# Find Irrlicht
#
# Input variables
#    Irrlicht_ROOT: location of irrlicht root directory
#    Irrlicht_INCLUDE_DIR: full path to the Irrlicht include directory
#    Irrlicht_LIBRARY: full path to the Irrlicht library
#    Irrlicht_DLL: full path to the Irrlicht DLL (windows only)
# Use first Irrlicht_INCLUDE_DIR, Irrlicht_LIBRARY, and Irrlicht_DLL if defined.
# Otherwise, if Irrlicht_ROOT is provided, look for the Irrlicht header and library
# in Irrlicht_ROOT and standard installation directories.
# If Irrlicht still not found, ask the caller to provide Irrlicht_ROOT and, if
# still needed, Irrlicht_INCLUDE_DIR, Irrlicht_LIBRARY, and Irrlicht_DLL.
#
# Output variables
#    Irrlicht_FOUND: true, if Irrlicht has been found
#    Irrlicht::Irrlicht target
# Additional output variables
#    IRRLICHT_INCLUDE_DIR
#    IRRLICHT_LIBRARY
#    IRRLICHT_DLL
#-------------------------------------------------------------------------------

mark_as_advanced(IRRLICHT_LIBRARY_PATHS IRRLICHT_INCLUDE_PATHS)

set(Irrlicht_FOUND FALSE)

# These are output variables thus should not be set by the user
unset(IRRLICHT_INCLUDE_DIR)
unset(IRRLICHT_LIBRARY)
unset(IRRLICHT_DLL)

# ----------------------------------------------------------------------------------------------------
# 1. Look for Irrlicht using Irrlicht_INCLUDE_DIR and Irrlicht_LIBRARY if both provided
# ----------------------------------------------------------------------------------------------------
message(STATUS "Looking for Irrlicht...")
if(NOT "${Irrlicht_INCLUDE_DIR}" STREQUAL "" AND NOT "${Irrlicht_LIBRARY}" STREQUAL "")
  find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h PATHS ${Irrlicht_INCLUDE_DIR} NO_CACHE NO_PACKAGE_ROOT_PATH)

  if(IRRLICHT_INCLUDE_DIR AND EXISTS ${Irrlicht_LIBRARY})
    set(IRRLICHT_LIBRARY ${Irrlicht_LIBRARY})
    set(Irrlicht_FOUND TRUE)
  endif()
endif()

if(NOT "${Irrlicht_DLL}" STREQUAL "")
  set(IRRLICHT_DLL ${Irrlicht_DLL})
endif()

if(Irrlicht_FOUND)
  if(NOT Irrlicht_FIND_QUIETLY)
    message(STATUS "Found Irrlicht using Irrlicht_INCLUDE_DIR and Irrlicht_LIBRARY")
  endif()
else()

  # ----------------------------------------------------------------------------------------------------
  # 2. Look for Irrlicht using Irrlicht_ROOT and/or in known locations
  # ----------------------------------------------------------------------------------------------------

  set(Irrlicht_FOUND TRUE)
  set(IRRLICHT_DLL "")
  unset(IRRLICHT_LIBRARY)

  # Windows
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(IRRLICHT_INCLUDE_PATHS "${Irrlicht_ROOT}/include")
  
    get_property(CURRENT_ENABLED_LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)
    list(FIND CURRENT_ENABLED_LANGUAGES CSharp LANG_INDEX)
    mark_as_advanced(FORCE LANG_INDEX)
    mark_as_advanced(FORCE CURRENT_ENABLED_LANGUAGES)
  
    if(MSVC OR LANG_INDEX GREATER_EQUAL 0)
      if (CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/lib/Win32-visualstudio")
        set(IRRLICHT_DLL "${Irrlicht_ROOT}/bin/Win32-VisualStudio/Irrlicht.dll")
      else()
        set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/lib/Win64-visualStudio")
        set(IRRLICHT_DLL "${Irrlicht_ROOT}/bin/Win64-VisualStudio/Irrlicht.dll")
      endif()
    elseif(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
      if (CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/lib/Win32-gcc")
      else()
        set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/lib/Win64-gcc")
      endif()
    elseif(DEFINED ENV{CONDA_BUILD})
      set(IRRLICHT_DLL "$ENV{PREFIX}/Library/bin/Irrlicht.dll")
    endif()
  
  # Linux
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  
    set(IRRLICHT_INCLUDE_PATHS "${Irrlicht_ROOT}" "${Irrlicht_ROOT}/include" "/usr/include/irrlicht" "/usr/local/include/irrlicht")
    set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/lib/Linux")
  
  # MacOS
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  
    set(IRRLICHT_INCLUDE_PATHS "${Irrlicht_ROOT}" "${Irrlicht_ROOT}/include" "${Irrlicht_ROOT}/include/irrlicht" "/usr/include/irrlicht" "/usr/local/include/irrlicht")
    set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/lib" "/usr/local/lib")
  
  # Other
  else()
  
    set(IRRLICHT_INCLUDE_PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
    set(IRRLICHT_LIBRARY_PATHS "${Irrlicht_ROOT}/../../lib")
  
  endif()
  
  # Find IRRLICHT headers
  find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h PATHS ${IRRLICHT_INCLUDE_PATHS} NO_CACHE)
  if (IRRLICHT_INCLUDE_DIR MATCHES IRRLICHT_INCLUDE_DIR-NOTFOUND)
    set(Irrlicht_FOUND FALSE)
  endif()
  
  # Find IRRLICHT library
  find_library(IRRLICHT_LIBRARY NAMES Irrlicht PATHS ${IRRLICHT_LIBRARY_PATHS} NO_CACHE)
  if (IRRLICHT_LIBRARY MATCHES IRRLICHT_LIBRARY-NOTFOUND)
    set(Irrlicht_FOUND FALSE)
  endif()
  
  if(Irrlicht_FOUND)
    if(NOT Irrlicht_FIND_QUIETLY)
      if("${Irrlicht_ROOT}" STREQUAL "")
        message(STATUS "Found Irrlicht in standard locations")
      else()
        message(STATUS "Found Irrlicht using Irrlicht_ROOT")
      endif()
    endif()
  endif()

endif()

# ----------------------------------------------------------------------------------------------------
# 3. If Irrlicht not found and Irrlicht_ROOT not provided, ask for Irrlicht_ROOT
# ----------------------------------------------------------------------------------------------------

if(NOT Irrlicht_FOUND AND "${Irrlicht_ROOT}" STREQUAL "")
  if(NOT Irrlicht_FIND_QUIETLY)
    message(STATUS "Could not find Irrlicht; set Irrlicht_ROOT")
  endif()
  set(Irrlicht_ROOT "" CACHE PATH "Path to root of Irrlicht SDK installation.")
  return()
endif()

# ----------------------------------------------------------------------------------------------------
# 4. If Irrlicht still not found, ask for Irrlicht_INCLUDE_DIR, Irrlicht_LIBRARY, and Irrlicht_DLL
# ----------------------------------------------------------------------------------------------------

if(NOT Irrlicht_FOUND)
  if(NOT Irrlicht_FIND_QUIETLY)
    message(STATUS "Could not find Irrlicht; set Irrlicht_INCLUDE_DIR and Irrlicht_LIBRARY")
  endif()
  set(Irrlicht_INCLUDE_DIR "" CACHE PATH "Irrlicht include directory")
  set(Irrlicht_LIBRARY "" CACHE PATH "Irrlicht library")
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(Irrlicht_DLL "" CACHE PATH "Irrlicht DLL")
  endif()
  return()
endif()

# ----------------------------------------------------------------------------------------------------
# Prepare target properties
# ----------------------------------------------------------------------------------------------------

# Set known dependencies
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(IRRLICHT_DEPENDENCY_LIBS "")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(IRRLICHT_DEPENDENCY_LIBS "Xxf86vm;glut;X11;GL")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  find_library(COCOA_LIBRARY Cocoa NO_CACHE)
  find_library(OPENGL_LIBRARY OpenGL NO_CACHE)
  find_library(IOKIT_LIBRARY IOKit NO_CACHE)
  if (DEFINED COCOA_LIBRARY-NOTFOUND)
    if(NOT Irrlicht_FIND_QUIETLY)
      message(STATUS "Could not find Irrlicht dependency library: Cocoa.")
    endif()
    return()
  endif()
  if (DEFINED OPENGL_LIBRARY-NOTFOUND)
    if(NOT Irrlicht_FIND_QUIETLY)
      message(STATUS "Could not find Irrlicht dependency library: OpenGL.")
    endif()
    return()
  endif()
  if (DEFINED IOKIT_LIBRARY-NOTFOUND)
    if(NOT Irrlicht_FIND_QUIETLY)
      message(STATUS "Could not find Irrlicht dependency library: IOKit.")
    endif()
    return()
  endif()
  set(IRRLICHT_DEPENDENCY_LIBS "${COCOA_LIBRARY};${OPENGL_LIBRARY};${IOKIT_LIBRARY}")
else()
  set(IRRLICHT_DEPENDENCY_LIBS "")
endif()

if(NOT Irrlicht_FIND_QUIETLY)
  message(STATUS "  include dir:  ${IRRLICHT_INCLUDE_DIR}")
  message(STATUS "  library:      ${IRRLICHT_LIBRARY}")
  message(STATUS "  dependencies: ${IRRLICHT_DEPENDENCY_LIBS}")
  message(STATUS "  DLL:          ${IRRLICHT_DLL}")
endif()

# ----------------------------------------------------------------------------------------------------
# 5. Create Irrlicht::Irrlicht target
# ----------------------------------------------------------------------------------------------------

if(Irrlicht_FOUND AND NOT TARGET Irrlicht::Irrlicht)

  add_library(Irrlicht::Irrlicht SHARED IMPORTED)
  set_target_properties(Irrlicht::Irrlicht PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${IRRLICHT_INCLUDE_DIR}")

  if(MSVC)
    set_property(TARGET Irrlicht::Irrlicht PROPERTY IMPORTED_LOCATION ${IRRLICHT_DLL})
    set_property(TARGET Irrlicht::Irrlicht PROPERTY INTERFACE_COMPILE_OPTIONS "/wd4275")
  else()
    set_property(TARGET Irrlicht::Irrlicht PROPERTY IMPORTED_LOCATION ${IRRLICHT_LIBRARY})
  endif()

  set_property(TARGET Irrlicht::Irrlicht PROPERTY IMPORTED_IMPLIB ${IRRLICHT_LIBRARY})
  set_property(TARGET Irrlicht::Irrlicht PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES ${IRRLICHT_DEPENDENCY_LIBS})
               
endif()
