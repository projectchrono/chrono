
#-------------------------------------------------------------------------------
# Find IRRLICHT
# This find script requires the following input variables:
# IRRLICHT_DIR: location of irrlicht root directory
# This find script provides the following output variables:
# IRRLICHT_FOUND: if Irrlicht has been found
# IRRLICHT_INCLUDE_DIRS: full path to include directories for Irrlicht
# IRRLICHT_LIBRARIES: full path to Irrlicht libraries and its dependencies
# IRRLICHT_DLL: full path to Irrlicht DLLs (Win only)
# IRRLICHT_C_FLAGS | IRRLICHT_CXX_FLAGS: compiler flags for Irrlicht
#
# other variable are intended only for internal use
#-------------------------------------------------------------------------------

mark_as_advanced(IRRLICHT_LIBRARY_PATHS IRRLICHT_INCLUDE_PATHS)

# Setting search paths for include and libraries
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(IRRLICHT_INCLUDE_PATHS "${IRRLICHT_DIR}/include")

  get_property(CURRENT_ENABLED_LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)
  list(FIND CURRENT_ENABLED_LANGUAGES CSharp LANG_INDEX)
  mark_as_advanced(FORCE LANG_INDEX)
  mark_as_advanced(FORCE CURRENT_ENABLED_LANGUAGES)

  if(MSVC OR LANG_INDEX GREATER_EQUAL 0)
    if (CMAKE_SIZEOF_VOID_P MATCHES 4)
      set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/lib/Win32-visualstudio")
      set(IRRLICHT_DLL "${IRRLICHT_DIR}/bin/Win32-VisualStudio/Irrlicht.dll")
    else()
      set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/lib/Win64-visualStudio")
      set(IRRLICHT_DLL "${IRRLICHT_DIR}/bin/Win64-VisualStudio/Irrlicht.dll")
    endif()
  elseif(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    if (CMAKE_SIZEOF_VOID_P MATCHES 4)
      set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/lib/Win32-gcc")
    else()
      set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/lib/Win64-gcc")
    endif()
  else()
    if(DEFINED ENV{CONDA_BUILD})
      set(IRRLICHT_DLL "$ENV{PREFIX}/Library/bin/Irrlicht.dll")
    endif()
  endif()
  set(IRRLICHT_DEPENDENCY_LIBS "")
  set(IRRLICHT_DEPENDENCY_INCLUDE_DIRS "")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

  set(IRRLICHT_INCLUDE_PATHS "${IRRLICHT_DIR}/include" "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/lib/Linux")
  set(IRRLICHT_DEPENDENCY_INCLUDE_DIRS "")
  set(IRRLICHT_DEPENDENCY_LIBS "Xxf86vm;glut;X11;GL")

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

  set(IRRLICHT_INCLUDE_PATHS "${IRRLICHT_DIR}/include" "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/lib/Linux" "/usr/local/lib")
  set(IRRLICHT_DEPENDENCY_INCLUDE_DIRS "/System/Library/Frameworks")

  find_library(COCOA_LIBRARY Cocoa NO_CACHE)
  find_library(OPENGL_LIBRARY OpenGL NO_CACHE)
  find_library(IOKIT_LIBRARY IOKit NO_CACHE)
  if (DEFINED COCOA_LIBRARY-NOTFOUND)
    message(FATAL_ERROR "Could not find IRRLICHT dependency library: Cocoa.")
    return()
  endif()
  if (DEFINED OPENGL_LIBRARY-NOTFOUND)
    message(FATAL_ERROR "Could not find IRRLICHT dependency library: OpenGL.")
    return()
  endif()
  if (DEFINED IOKIT_LIBRARY-NOTFOUND)
    message(FATAL_ERROR "Could not find IRRLICHT dependency library: IOKit.")
    return()
  endif()
  set(IRRLICHT_DEPENDENCY_LIBS "${COCOA_LIBRARY};${OPENGL_LIBRARY};${IOKIT_LIBRARY}")

else()

  set(IRRLICHT_INCLUDE_PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  set(IRRLICHT_LIBRARY_PATHS "${IRRLICHT_DIR}/../../lib")
  set(IRRLICHT_DEPENDENCY_LIBS "")
  set(IRRLICHT_DEPENDENCY_INCLUDE_DIRS "")

endif()

# Find IRRLICHT headers
find_path(IRRLICHT_INCLUDE_DIR NAMES irrlicht.h PATHS ${IRRLICHT_INCLUDE_PATHS} NO_CACHE)
if (IRRLICHT_INCLUDE_DIR MATCHES IRRLICHT_INCLUDE_DIR-NOTFOUND)
  #message(FATAL_ERROR "Could not find 'irrlicht.h' in any of the following directories: ${IRRLICHT_INCLUDE_PATHS}.\nSet IRRLICHT_DIR to the Irrlicht SDK installation directory.")
  return()
endif()
set(IRRLICHT_INCLUDE_DIRS ${IRRLICHT_INCLUDE_DIR} ${IRRLICHT_DEPENDENCY_INCLUDE_DIRS})

# Find IRRLICHT library
find_library(IRRLICHT_LIBRARY NAMES Irrlicht PATHS ${IRRLICHT_LIBRARY_PATHS} NO_CACHE)
if (IRRLICHT_LIBRARY MATCHES IRRLICHT_LIBRARY-NOTFOUND)
  #message(FATAL_ERROR "Could not find 'irrlicht${CMAKE_IMPORT_LIBRARY_SUFFIX}' in any of the following directories: ${IRRLICHT_LIBRARY_PATHS}.\nSet IRRLICHT_DIR to the Irrlicht SDK installation directory.")
  return()
endif()
set(IRRLICHT_LIBRARIES ${IRRLICHT_LIBRARY} ${IRRLICHT_DEPENDENCY_LIBS})

set(IRRLICHT_FOUND TRUE)


# IRRLICHT-specific compiler flags
set(IRRLICHT_CXX_FLAGS "")
set(IRRLICHT_C_FLAGS "")
if(MSVC)
  # If using MSVC, disable warning 4275 (non-DLL-interface class used as base for DLL-interface class)
  ###add_compile_options(/wd4275)
  set(IRRLICHT_CXX_FLAGS "${IRRLICHT_CXX_FLAGS} /wd4275")
  set(IRRLICHT_C_FLAGS "${IRRLICHT_C_FLAGS} /wd4275")
endif()

if(IRRLICHT_FOUND AND NOT TARGET Irrlicht::Irrlicht)
  add_library(Irrlicht::Irrlicht SHARED IMPORTED)
  set_target_properties(Irrlicht::Irrlicht PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${IRRLICHT_INCLUDE_DIRS}")
  
  set_property(TARGET Irrlicht::Irrlicht PROPERTY
              IMPORTED_LOCATION ${IRRLICHT_DLL})
  set_property(TARGET Irrlicht::Irrlicht PROPERTY
              IMPORTED_IMPLIB ${IRRLICHT_LIBRARY})
  set_property(TARGET Irrlicht::Irrlicht PROPERTY
              IMPORTED_LINK_DEPENDENT_LIBRARIES ${IRRLICHT_DEPENDENCY_LIBS})
  set_property(TARGET Irrlicht::Irrlicht PROPERTY
               INTERFACE_COMPILE_OPTIONS "/wd4275")

  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set_property(TARGET Irrlicht::Irrlicht PROPERTY
                 INTERFACE_COMPILE_OPTIONS "-framework IOKit -framework Cocoa -framework OpenGL")
  endif()
               
endif()