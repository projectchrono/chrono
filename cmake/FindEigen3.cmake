# Try to find Eigen3 lib
#
# This module supports requiring a minimum version, e.g. you can do
#   find_package(Eigen3 3.1.2)
# to require version 3.1.2 or newer of Eigen3.
#
# If successful, this will define
#
#  Eigen3_FOUND - system has Eigen3 lib with correct version
#  EIGEN3_INCLUDE_DIR - the Eigen3 include directory
#  EIGEN3_VERSION - Eigen3 version
#
# and the following imported target:
#
#  Eigen3::Eigen - The header-only Eigen library
#
# This module reads hints about search locations from the following environment variables:
#   EIGEN3_ROOT
#   EIGEN3_ROOT_DIR

# Based on:
# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Copyright (c) 2008, 2009 Gael Guennebaud, <g.gael@free.fr>
# Copyright (c) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.


if(NOT Eigen3_FIND_VERSION)
  if(NOT Eigen3_FIND_VERSION_MAJOR)
    set(Eigen3_FIND_VERSION_MAJOR 3)
  endif()
  if(NOT Eigen3_FIND_VERSION_MINOR)
    set(Eigen3_FIND_VERSION_MINOR 3)
  endif()
  if(NOT Eigen3_FIND_VERSION_PATCH)
    set(Eigen3_FIND_VERSION_PATCH 0)
  endif()

  set(Eigen3_FIND_VERSION "${Eigen3_FIND_VERSION_MAJOR}.${Eigen3_FIND_VERSION_MINOR}.${Eigen3_FIND_VERSION_PATCH}")
endif()

# ------------------------------------------------------------------------------

macro(_eigen3_check_version)
  if(EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Version")
      # Eigen version >= 5.0.0
      file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/Version" _eigen3_version_header)
      
      string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
      set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")
      string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
      set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")
      string(REGEX MATCH "define[ \t]+EIGEN_PATCH_VERSION[ \t]+([0-9]+)" _eigen3_patch_version_match "${_eigen3_version_header}")
      set(EIGEN3_PATCH_VERSION "${CMAKE_MATCH_1}")
      
      set(EIGEN3_VERSION ${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION}.${EIGEN3_PATCH_VERSION})
  elseif(EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
      # Eigen version <= 3.4.1
      file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen3_version_header)
      
      string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}")
      set(EIGEN3_WORLD_VERSION "${CMAKE_MATCH_1}")
      string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
      set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")
      string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
      set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")
      
      set(EIGEN3_VERSION ${EIGEN3_WORLD_VERSION}.${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION})
  endif()

  if(EIGEN3_VERSION)
      if(${EIGEN3_VERSION} VERSION_LESS ${Eigen3_FIND_VERSION})
          set(EIGEN3_VERSION_OK FALSE)
          message("WARNING: Eigen3 version ${EIGEN3_VERSION} found in ${EIGEN3_INCLUDE_DIR}, "
                  "but at least version ${Eigen3_FIND_VERSION} is required")
      else()
          set(EIGEN3_VERSION_OK TRUE)
      endif()
  else()
      message("ERROR: cannot find Eigen3 version information")
  endif()

endmacro()

# ------------------------------------------------------------------------------

if(EIGEN3_INCLUDE_DIR)

    if(NOT Eigen3_FIND_QUIETLY)
      message(STATUS "  EIGEN3_INCLUDE_DIR found in cache.")
      message(STATUS "  Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
    endif()

    _eigen3_check_version()
    set(EIGEN3_FOUND ${EIGEN3_VERSION_OK})
    set(Eigen3_FOUND ${EIGEN3_VERSION_OK})

else()

    if(NOT Eigen3_FIND_QUIETLY)
      message(STATUS "  EIGEN3_INCLUDE_DIR NOT found in cache.")
      message(STATUS "  Looking for Eigen3Config.cmake.")
    endif()
  
    # Check if Eigen3Config.cmake is available
    if(Eigen3_FIND_QUIETLY)
      find_package(Eigen3 ${Eigen3_FIND_VERSION} NO_MODULE QUIET)
    else()
      find_package(Eigen3 ${Eigen3_FIND_VERSION} NO_MODULE)
    endif()

    if(Eigen3_FOUND)

        get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

        if(NOT Eigen3_FIND_QUIETLY)
          message(STATUS "  Eigen3 found with Eigen3Config.cmake.")
          message(STATUS "  Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
        endif()
  
    else()
  
        if(NOT Eigen3_FIND_QUIETLY)
          message(STATUS "  Eigen3 NOT found through Eigen3Config.cmake.")
          message(STATUS "  Searching paths EIGEN3_ROOT, EIGEN3_ROOT_DIR.")
        endif()

        find_path(EIGEN3_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
        HINTS
        ENV Eigen3_ROOT 
        ENV Eigen3_ROOT_DIR
        PATHS
        ${CMAKE_INSTALL_PREFIX}/include
        ${KDE4_INCLUDE_DIR}
        PATH_SUFFIXES eigen3 eigen
        )

        if(NOT EIGEN3_INCLUDE_DIR)
           if(NOT Eigen3_FIND_QUIETLY)
             message(STATUS "  EIGEN3_INCLUDE_DIR NOT found through find_path.")
           endif()
        endif()

    endif()

    if(EIGEN3_INCLUDE_DIR)
      _eigen3_check_version()
    endif()

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR EIGEN3_VERSION_OK)

    mark_as_advanced(EIGEN3_INCLUDE_DIR)

endif()

# ------------------------------------------------------------------------------

if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
                          INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
