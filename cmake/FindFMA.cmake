# This script checks for the highest level of FMA support on the host
# by compiling and running small C++ programs that uses FMA intrinsics.
#
# You can invoke this module using the following command:
#
#   FIND_PACKAGE(FMA [QUIET|REQUIRED])
#
# If any  FMA support is detected, the following variables are set:
#
#   FMA_FOUND   = 1
#   FMA_FLAGS = compile flags for the version of FMA found
# 
# If  FMA is not supported on the host platform, these variables are
# not set. If QUIET is true, the module does not print a message if
#  FMA if missing. If REQUIRED is true, the module produces a fatal
# error if  FMA support is missing.
# 
set(FMA_FLAGS)
set(FMA_FOUND)
set(DETECTED_FMA)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE GCC_VERSION_STRING)
  if(GCC_VERSION_STRING VERSION_GREATER 4.2 AND NOT APPLE AND NOT CMAKE_CROSSCOMPILING)
    SET(FMA_FLAGS "${FMA_FLAGS} -march=native")
    message(STATUS "Using CPU native flags for FMA optimization: ${FMA_FLAGS}")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT CMAKE_CROSSCOMPILING)
    SET(FMA_FLAGS "${FMA_FLAGS} -march=native")
    message(STATUS "Using CPU native flags for FMA optimization: ${FMA_FLAGS}")
endif()


include(CheckCXXSourceRuns)
set(CMAKE_REQUIRED_FLAGS)

# Generate a list of FMA versions to test.
set(_FMA_TEST 1)

# Check for FMA support.
if(_FMA_TEST)
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_REQUIRED_FLAGS "-mavx2 -mfma")
  elseif(MSVC AND NOT CMAKE_CL_64)
    set(CMAKE_REQUIRED_FLAGS "/arch:AVX2")
  endif()
  check_cxx_source_runs("
  #include <immintrin.h>
  int main()
    {
    __m256d a = _mm256_set_pd (-1, 2, -3, 4);
    __m256d b = _mm256_set_pd (-2, 3, -4, 1);
    __m256d c = _mm256_set_pd (-11, 6, 4, -1);

    __m256d result =  _mm256_fmsub_pd (a, b, c);
    return 0;
    }" DETECTED_FMA)
endif()

set(CMAKE_REQUIRED_FLAGS)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(DETECTED_FMA)
      SET(FMA_FLAGS "${FMA_FLAGS} -mfma")
      SET(FMA_FOUND 1)
  endif()
elseif(MSVC)
 if(DETECTED_FMA)
      SET(FMA_FLAGS "${FMA_FLAGS} /arch:AVX2")
      SET(FMA_FOUND 1)
  endif()
endif()

if(FMA_FOUND)
  message(STATUS "  Found FMA extensions, using flags: ${FMA_FLAGS}")
else()
  message(STATUS "  No FMA support found")
  set(FMA_FLAGS "")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${FMA_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${FMA_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${FMA_FLAGS}")

return()
#-----------------------

# If no FMA support is found, print an error message.
if(FMA_FIND_REQUIRED)
  set(_FMA_ERROR_MESSAGE "FMA support is not found on this architecture")
endif()

if(FMA_FIND_REQUIRED)
  message(FATAL_ERROR "${_FMA_ERROR_MESSAGE}")
elseif(NOT FMA_FIND_QUIETLY)
  message(STATUS "${_FMA_ERROR_MESSAGE}")
endif()
