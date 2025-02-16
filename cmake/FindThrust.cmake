##=============================================================================
##
##  Copyright (c) Kitware, Inc.
##  All rights reserved.
##  See LICENSE.txt for details.
##
##  This software is distributed WITHOUT ANY WARRANTY; without even
##  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
##  PURPOSE.  See the above copyright notice for more information.
##
##  Copyright 2012 Sandia Corporation.
##  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
##  the U.S. Government retains certain rights in this software.
##
##=============================================================================

#
# FindThrust
#
# This module finds the Thrust header files and extrats their version.  It
# sets the following variables.
#
# THRUST_INCLUDE_DIR -  Include directory for thrust header files.  (All header
#                       files will actually be in the thrust subdirectory.)
# THRUST_VERSION -      Version of thrust in the form "major.minor.patch".
#

find_path( THRUST_INCLUDE_DIR
  HINTS
    ${Thrust_ROOT}  
    ${CUDAToolkit_LIBRARY_ROOT}/include
    ${CUDA_INCLUDE_DIRS}
    /usr/include/cuda
    /usr/local/
    /usr/local/include
    /usr/local/cuda/include
    /opt/cuda/include
    $ENV{CUDA_PATH}/include
  NAMES thrust/version.h
  DOC "Thrust headers"
  )
if( THRUST_INCLUDE_DIR )
  list( REMOVE_DUPLICATES THRUST_INCLUDE_DIR )
endif( THRUST_INCLUDE_DIR )

if(NOT THRUST_INCLUDE_DIR)
  return()
endif()

# Find thrust version
file( STRINGS ${THRUST_INCLUDE_DIR}/thrust/version.h
  version
  REGEX "#define THRUST_VERSION[ \t]+([0-9x]+)"
  )
string( REGEX REPLACE
  "#define THRUST_VERSION[ \t]+([0-9x]+)(.*)"
  "\\1"
  version
  "${version}"
)

math(EXPR major "${version} / 100000")
math(EXPR minor "(${version} / 100) % 1000")
math(EXPR version "${version} % 100")
set( THRUST_VERSION "${major}.${minor}.${version}")
set( THRUST_MAJOR_VERSION "${major}")
set( THRUST_MINOR_VERSION "${minor}")

# Check for required components
include( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Thrust
  REQUIRED_VARS THRUST_INCLUDE_DIR
  VERSION_VAR THRUST_VERSION
  )


set(THRUST_INCLUDE_DIRS ${THRUST_INCLUDE_DIR})
mark_as_advanced(THRUST_INCLUDE_DIR)

if(THRUST_INCLUDE_DIR AND NOT TARGET Thrust::Thrust)
  add_library(Thrust::Thrust INTERFACE IMPORTED)
  set_target_properties(Thrust::Thrust PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${THRUST_INCLUDE_DIR}")
endif()
