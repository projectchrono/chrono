# CMake script to detect Intel(R) Math Kernel Library (MKL)
#
# This will try to find Intel MKL libraries, and include path by automatic
# search through typical install locations and if failed it will
# examine MKLROOT environment variable.
# Note, MKLROOT is not set by IPP installer, it should be set manually.
#
# Usage example:
#   set(MKL_USE_STATIC_LIBS ON)
#   find_package(MKL)
#   if (MKL_FOUND)
#      include_directories(${MKL_INCLUDE_DIRS})
#      link_directories(${MKL_LIBRARY_DIRS})
#      add_executable(foo foo.cc)
#      target_link_libraries(foo ${MKL_LIBRARIES})
#   endif()
#
# Variables used by this module, they can change the default behaviour and
# need to be set before calling find_package:
#
#   MKL_ADDITIONAL_VERSIONS      A list of version numbers to use for searching
#                                the MKL include directory.
#
#   MKL_USE_STATIC_LIBS          Can be set to ON to force the use of the static
#                                boost libraries. Defaults to OFF.
#
#   MKL_FIND_DEBUG               Set this to TRUE to enable debugging output
#                                of FindMKL.cmake if you are having problems.
#
# On return this will define:
#   MKL_FOUND                   Indicates whether MKL was found (True/False)
#   MKL_INCLUDE_DIRS            MKL include folder
#   MKL_LIBRARY_DIRS            MKL libraries folder
#   MKL_LIBRARIES               MKL libraries names
#
# NOTE: this script has only been tested with Intel(R) Parallel Studio XE 2011
# and may need changes for compatibility with older versions.
#
# Adapted from OpenCV IPP detection script
#   https://code.ros.org/trac/opencv/browser/trunk/opencv/OpenCVFindIPP.cmake
# Many portions taken from FindBoost.cmake

# TODO:
# - caller needs to link with libiomp5md.lib or /Qopenmp...
# - runtime DLLs:
#   <Composer XE directory> -> C:\Program Files\Intel\ComposerXE-2011
#     redist\ia32\mkl
#     redist\intel64\mkl

set(_MKL_IA32 FALSE)
set(_MKL_INTEL64 FALSE)
if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(_MKL_IA32 TRUE)
elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(_MKL_INTEL64 TRUE)
else()
    message(FATAL_ERROR "Unsupported 'void *' size (${SIZEOF_VOID_P})")
endif()

# Versions should be listed is decreasing order of preference
set(_MKL_TEST_VERSIONS ${MKL_ADDITIONAL_VERSIONS}
    "2011"
    # alternative form: "2011.xxx.y"
    # (y is the release-update number and xxx is the package number)
)

if (MKL_FIND_VERSION AND NOT MKL_FIND_QUIETLY)
    message(WARNING "Requesting a specific version of Intel(R) MKL is not supported")
endif()

# Use environment variables from Intel build scripts, if available
if (NOT MKL_ROOT AND NOT $ENV{MKLROOT} STREQUAL "")
  set(MKL_ROOT $ENV{MKLROOT})
endif()

if (MKL_ROOT)
  file(TO_CMAKE_PATH ${MKL_ROOT} MKL_ROOT)
endif()

if (NOT INTEL_ROOT AND NOT $ENV{INTELROOT} STREQUAL "")
  set(INTEL_ROOT $ENV{INTELROOT})
endif()

if (INTEL_ROOT)
  file(TO_CMAKE_PATH ${INTEL_ROOT} INTEL_ROOT)
endif()

if (MKL_FIND_DEBUG)
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "_MKL_TEST_VERSIONS = ${_MKL_TEST_VERSIONS}")
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "MKL_ADDITIONAL_VERSIONS = ${MKL_ADDITIONAL_VERSIONS}")
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "MKL_USE_STATIC_LIBS = ${MKL_USE_STATIC_LIBS}")
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "MKL_ROOT = ${MKL_ROOT}")
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "INTEL_ROOT = ${INTEL_ROOT}")
endif()

# Find MKL include directory

set(_MKL_ROOT_SEARCH_DIRS
  ${MKL_ROOT}
)

foreach (_MKL_VER ${_MKL_TEST_VERSIONS})
    if (WIN32)
        list(APPEND _MKL_ROOT_SEARCH_DIRS "$ENV{ProgramFiles}/Intel/Composer XE/mkl")
    else()
        list(APPEND _MKL_ROOT_SEARCH_DIRS "/opt/intel/composerxe-${_MKL_VER}/mkl")
    endif()
endforeach()

if (MKL_FIND_DEBUG)
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "_MKL_ROOT_SEARCH_DIRS = ${_MKL_ROOT_SEARCH_DIRS}")
endif()

find_path(MKL_INCLUDE_DIR
    NAMES mkl.h
    PATHS ${_MKL_ROOT_SEARCH_DIRS}
    PATH_SUFFIXES include
    DOC "The path to Intel(R) MKL header files"
)

if (MKL_INCLUDE_DIR)
    if (MKL_FIND_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                       "location of mkl.h: ${MKL_INCLUDE_DIR}/mkl.h")
    endif()
else()
    if (MKL_FIND_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                       "unable to find Intel(R) MKL header files. Please set MKLROOT"
                       " to the root directory containing MKL.")
    endif()
endif()

# Find MKL library directory

set(_INTEL_LIBRARY_DIR_SUFFIXES "lib")
if (_MKL_IA32)
    list(APPEND _INTEL_LIBRARY_DIR_SUFFIXES "lib/ia32")
elseif (_MKL_INTEL64)
    list(APPEND _INTEL_LIBRARY_DIR_SUFFIXES "lib/intel64")
else()
    message(FATAL_ERROR "unreachable")
endif()

set(_MKL_LIBRARY_SEARCH_DIRS ${_MKL_ROOT_SEARCH_DIRS})
if (MKL_INCLUDE_DIR)
    list(APPEND _MKL_LIBRARY_SEARCH_DIRS "${MKL_INCLUDE_DIR}/..")
endif()

if (MKL_FIND_DEBUG)
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "_INTEL_LIBRARY_DIR_SUFFIXES = ${_INTEL_LIBRARY_DIR_SUFFIXES}")
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "_MKL_LIBRARY_SEARCH_DIRS = ${_MKL_LIBRARY_SEARCH_DIRS}")
endif()

set(MKL_LIB_PREFIX "mkl_")
if (MKL_USE_STATIC_LIBS)
    if (_MKL_IA32)
        if (WIN32)
            set(_MKL_LIBRARIES intel_c)
        else()
            set(_MKL_LIBRARIES intel)
        endif()
    elseif (_MKL_INTEL64)
        set(_MKL_LIBRARIES intel_lp64)
    else()
        message(FATAL_ERROR "unreachable")
    endif()

    list(APPEND _MKL_LIBRARIES intel_thread)
    list(APPEND _MKL_LIBRARIES core)
else()
    set(_MKL_LIBRARIES rt)
endif()

set(_MKL_MISSING_LIBRARIES "")
set(MKL_LIBRARIES "")
set(MKL_LIBRARY_DIRS "")
# Find MKL libraries
foreach (_MKL_LIB_RAW ${_MKL_LIBRARIES})
    set(_MKL_LIB ${MKL_LIB_PREFIX}${_MKL_LIB_RAW})
    string(TOUPPER ${_MKL_LIB} _MKL_LIB_UPPER)

    find_library(${_MKL_LIB_UPPER}_LIBRARY
        NAMES ${_MKL_LIB}
        PATHS ${_MKL_LIBRARY_SEARCH_DIRS}
        PATH_SUFFIXES ${_INTEL_LIBRARY_DIR_SUFFIXES}
        DOC "The path to Intel(R) MKL ${_MKL_LIB_RAW} library"
    )
    mark_as_advanced(${_MKL_LIB_UPPER}_LIBRARY)

    if (NOT ${_MKL_LIB_UPPER}_LIBRARY)
        list(APPEND _MKL_MISSING_LIBRARIES ${_MKL_LIB})
    else()
        list(APPEND MKL_LIBRARIES ${${_MKL_LIB_UPPER}_LIBRARY})
        if (MKL_FIND_DEBUG)
            message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                           "Found ${_MKL_LIB}: ${${_MKL_LIB_UPPER}_LIBRARY}")
        endif()

        get_filename_component(_MKL_LIB_PATH "${${_MKL_LIB_UPPER}_LIBRARY}" PATH)
        list(APPEND MKL_LIBRARY_DIRS ${_MKL_LIB_PATH})
    endif()
endforeach()

## Find OpenMP, pthread and math libraries

set(_INTEL_LIBRARY_SEARCH_DIRS
  ${INTEL_ROOT}
  ${INTEL_ROOT}/compiler
)

foreach(_MKL_DIR ${_MKL_ROOT_SEARCH_DIRS})
    list(APPEND _INTEL_LIBRARY_SEARCH_DIRS "${_MKL_DIR}/..")
    list(APPEND _INTEL_LIBRARY_SEARCH_DIRS "${_MKL_DIR}/../compiler")
endforeach()

if (MKL_FIND_DEBUG)
    message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                   "_INTEL_LIBRARY_SEARCH_DIRS = ${_INTEL_LIBRARY_SEARCH_DIRS}")
endif()

if (NOT WIN32)
    find_library(PTHREAD_LIBRARY pthread DOC "Path to POSIX threads library")
endif()

set(_IOMP5_LIB iomp5)
if (WIN32)
  if (MKL_USE_STATIC_LIBS)
      list(APPEND _IOMP5_LIB libiomp5mt.lib)
  else()
      list(APPEND _IOMP5_LIB libiomp5md.lib)
  endif()
endif()

find_library(IOMP5_LIBRARY
    NAMES ${_IOMP5_LIB}
    PATHS ${_INTEL_LIBRARY_SEARCH_DIRS}
    PATH_SUFFIXES ${_INTEL_LIBRARY_DIR_SUFFIXES}
    DOC "Path to OpenMP runtime library"
)

if (NOT IOMP5_LIBRARY)
    # we could instead fallback to default library (via FindOpenMP.cmake)
    list(APPEND _MKL_MISSING_LIBRARIES IOMP5)
else()
    list(APPEND MKL_LIBRARIES ${IOMP5_LIBRARY})
    if (MKL_FIND_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                       "Found IOMP5_LIBRARY: ${IOMP5_LIBRARY}")
    endif()

    get_filename_component(_MKL_LIB_PATH "${IOMP5_LIBRARY}" PATH)
    list(APPEND MKL_LIBRARY_DIRS ${_MKL_LIB_PATH})
endif()

# Optimized math library (optional)
set(_MATH_LIB imf)  # linked by default with Intel compiler
if (WIN32)
  if (MKL_USE_STATIC_LIBS)
    list(APPEND _MATH_LIB libmmds.lib)  # assumes (/MD) otherwise libmmt.lib (for /MT)
  else()
      list(APPEND _MATH_LIB libmmd.lib)
  endif()
endif()

find_library(MATH_LIBRARY
    NAMES ${_MATH_LIB}
    PATHS ${_INTEL_LIBRARY_SEARCH_DIRS}
    PATH_SUFFIXES ${_INTEL_LIBRARY_DIR_SUFFIXES}
    DOC "Path to optimized math library"
)

if (NOT MATH_LIBRARY)
    # we could instead fallback to default library (via FindOpenMP.cmake)
    list(APPEND _MKL_MISSING_LIBRARIES MATH)
else()
    list(APPEND MKL_LIBRARIES ${MATH_LIBRARY})
    if (MKL_FIND_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                       "Found MATH_LIBRARY: ${MATH_LIBRARY}")
    endif()

    get_filename_component(_MKL_LIB_PATH "${MATH_LIBRARY}" PATH)
    list(APPEND MKL_LIBRARY_DIRS ${_MKL_LIB_PATH})
endif()

# Check all required libraries are available
list(REMOVE_DUPLICATES MKL_LIBRARY_DIRS)

set(MKL_INCLUDE_DIRS
    ${MKL_INCLUDE_DIR}
)

set(MKL_FOUND TRUE)
if (NOT MKL_INCLUDE_DIR)
    set(MKL_FOUND FALSE)
    if (MKL_FIND_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                       "MKL not found - MKL_INCLUDE_DIR was empty")
    endif()
elseif (_MKL_MISSING_LIBRARIES)
    set(MKL_FOUND FALSE)
    if (MKL_FIND_DEBUG)
        message(STATUS "[ ${CMAKE_CURRENT_LIST_FILE}:${CMAKE_CURRENT_LIST_LINE} ] "
                       "MKL not found - the following libraries are missing: "
                       "${_MKL_MISSING_LIBRARIES}")
    endif()
endif()

if (MKL_FOUND)
    if (NOT MKL_FIND_QUIETLY OR MKL_FIND_DEBUG)
        message(STATUS
            "Intel(R) MKL was found:\n"
            "  MKL_INCLUDE_DIRS: ${MKL_INCLUDE_DIRS}\n"
            "  MKL_LIBRARY_DIRS: ${MKL_LIBRARY_DIRS}\n"
            "  MKL_LIBRARIES: ${MKL_LIBRARIES}"
        )
    endif()
else()
    if (MKL_FIND_REQUIRED)
        message(SEND_ERROR "Intel(R) MKL could not be found.")
    else()
        message(STATUS "Intel(R) MKL could not be found.")
    endif()
endif()

mark_as_advanced(
    MKL_INCLUDE_DIR
    MKL_INCLUDE_DIRS
    MKL_LIBRARY_DIRS
)
