
#-------------------------------------------------------------------------------
# Find IrrKlang
# # Input variables:
# - IRRKLANG_ROOT: root folder of IrrKlang SDK
# - IRRKLANG_LIBDIR: folder containing IrrKlang libraries
# # Output variables:
# - IRRKLANG_INCLUDE_DIR: folder containing IrrKlang headers
# - IRRKLANG_LIBRARY: path to IrrKlang library
# # Output targets
# - IrrKlang::IrrKlang: CMake target
#-------------------------------------------------------------------------------

# Irrklang

set(IRRKLANG_FOUND FALSE)

find_path(IRRKLANG_INCLUDE_DIR
          NAMES irrKlang.h
          PATHS "/usr/include/irrklang" "/usr/local/include/irrklang" "${IRRKLANG_ROOT}/include")

if (NOT IRRKLANG_INCLUDE_DIR)
  message(SEND_ERROR "Cannot find IrrKlang include folder. Set IRRKLANG_ROOT to the IrrKlang SDK root directory (should contain 'include/irrKlang.h').")
  return()
endif()

if (NOT IRRKLANG_LIBDIR OR IRRKLANG_LIBDIR MATCHES "")
  if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    set(IRRKLANG_LIBDIR "${IRRKLANG_INCLUDE_DIR}/../lib/Win32-VisualStudio/")
  elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    set(IRRKLANG_LIBDIR "${IRRKLANG_INCLUDE_DIR}/../lib/Winx64-VisualStudio/")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(IRRKLANG_LIBDIR "${IRRKLANG_INCLUDE_DIR}/../lib/Linux/")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(IRRKLANG_LIBDIR "${IRRKLANG_INCLUDE_DIR}/../lib/MacOSX/")
  endif()
endif(NOT IRRKLANG_LIBDIR OR IRRKLANG_LIBDIR MATCHES "")

message(STATUS "IRRKLANG_LIBDIR: ${IRRKLANG_LIBDIR}")

find_library(IRRKLANG_LIBRARY
             NAMES "Irrklang" "irrKlang"
             PATHS ${IRRKLANG_LIBDIR} "/usr/local/lib")

if (NOT IRRKLANG_LIBRARY)
  message(SEND_ERROR "Cannot find IrrKlang library. Set IRRKLANG_LIBDIR to the appropriate irrKlang library directory.")
  return()
endif()


if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    set(IRRKLANG_DLL "${IRRKLANG_INCLUDE_DIR}/../bin/Win32-VisualStudio/irrKlang.dll")
  elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    set(IRRKLANG_DLL "${IRRKLANG_INCLUDE_DIR}/../bin/Winx64-VisualStudio/irrKlang.dll")
  endif()
endif()

mark_as_advanced(CLEAR IRRKLANG_INCLUDE_DIR)
mark_as_advanced(CLEAR IRRKLANG_LIBRARY)

message(STATUS "IrrKlang include dir: ${IRRKLANG_INCLUDE_DIR}")
message(STATUS "IrrKlang library:     ${IRRKLANG_LIBRARY}")

set(IRRKLANG_FOUND TRUE)



if(IRRKLANG_FOUND AND NOT TARGET IrrKlang::IrrKlang)
  add_library(IrrKlang::IrrKlang SHARED IMPORTED)
  set_target_properties(IrrKlang::IrrKlang PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${IRRKLANG_INCLUDE_DIR}")
  
  set_property(TARGET IrrKlang::IrrKlang PROPERTY
              IMPORTED_LOCATION ${IRRKLANG_DLL})
  set_property(TARGET IrrKlang::IrrKlang PROPERTY
              IMPORTED_IMPLIB ${IRRKLANG_LIBRARY})
               
endif()

