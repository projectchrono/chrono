# This script checks for the highest level of SSE support on the host
# by compiling and running small C++ programs that uses SSE intrinsics.
#
# You can invoke this module using the following command:
#
#   FIND_PACKAGE(SSE [major[.minor]] [EXACT] [QUIET|REQUIRED])
#
# where the version string is one of:
#
#   1.0 for SSE support
#   2.0 for SSE2 support
#   3.0 for SSE3 support
#   3.1 for SSSE3 support
#   4.1 for SSE 4.1 support
#   4.2 for SSE 4.2 support
#
# Note that any ".0" in the above version string is optional.
#
# If any SSE support is detected, the following variables are set:
#
#   SSE_FOUND   = 1
#   SSE_VERSION = the requested version, if EXACT is true, or
#                 the highest SSE version found.
#   SSE_FLAGS = compile flags for the version of SSE found
# 
# If SSE is not supported on the host platform, these variables are
# not set. If QUIET is true, the module does not print a message if
# SSE if missing. If REQUIRED is true, the module produces a fatal
# error if SSE support is missing.
# 
set(SSE_FLAGS)
set(SSE_FOUND)
set(DETECTED_SSE_41)
set(DETECTED_SSE_42)
set(DETECTED_SSE_10)
set(DETECTED_SSE_20)
set(DETECTED_SSE_30)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE GCC_VERSION_STRING)
  if(GCC_VERSION_STRING VERSION_GREATER 4.2 AND NOT APPLE AND NOT CMAKE_CROSSCOMPILING)
    SET(SSE_FLAGS "${SSE_FLAGS} -march=native")
    message(STATUS "Using CPU native flags for SSE optimization: ${SSE_FLAGS}")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT CMAKE_CROSSCOMPILING)
    SET(SSE_FLAGS "${SSE_FLAGS} -march=native")
    message(STATUS "Using CPU native flags for SSE optimization: ${SSE_FLAGS}")
endif()

include(CheckCXXSourceRuns)
set(CMAKE_REQUIRED_FLAGS)


# Generate a list of SSE versions to test.
if(SSE_FIND_VERSION_EXACT)
  if(SSE_FIND_VERSION VERSION_EQUAL "4.2")
    set(_SSE_TEST_42 1)
  elseif(SSE_FIND_VERSION VERSION_EQUAL "4.1")
    set(_SSE_TEST_41 1)
  elseif(SSE_FIND_VERSION VERSION_EQUAL "3.0")
    set(_SSE_TEST_30 1)
  elseif(SSE_FIND_VERSION VERSION_EQUAL "2.0")
    set(_SSE_TEST_20 1)
  elseif(SSE_FIND_VERSION VERSION_EQUAL "1.0")
    set(_SSE_TEST_10 1)
  endif()
else()
  if(NOT SSE_FIND_VERSION VERSION_GREATER "4.2")
    set(_SSE_TEST_42 1)
  endif()
  if(NOT SSE_FIND_VERSION VERSION_GREATER "4.1")
    set(_SSE_TEST_41 1)
  endif()
  if(NOT SSE_FIND_VERSION VERSION_GREATER "3.0")
    set(_SSE_TEST_30 1)
  endif()
  if(NOT SSE_FIND_VERSION VERSION_GREATER "2.0")
    set(_SSE_TEST_20 1)
  endif()
  if(NOT SSE_FIND_VERSION VERSION_GREATER "1.0")
    set(_SSE_TEST_10 1)
  endif()  
endif()


# Check for SSE 4.2 support.
if(_SSE_TEST_42)
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_REQUIRED_FLAGS "-msse4.2")
  endif()
  check_cxx_source_runs("
  #include <emmintrin.h>
  #include <nmmintrin.h>
  int main()
  {
    long long a[2] = {  1, 2 };
    long long b[2] = { -1, 3 };
    long long c[2];
    __m128i va = _mm_loadu_si128((__m128i*)a);
    __m128i vb = _mm_loadu_si128((__m128i*)b);
    __m128i vc = _mm_cmpgt_epi64(va, vb);

    _mm_storeu_si128((__m128i*)c, vc);
    if (c[0] == -1LL && c[1] == 0LL)
      return 0;
    else
      return 1;
  }" 
  DETECTED_SSE_42)
endif()

# Check for SSE 4.1 support.
if(_SSE_TEST_41)
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_REQUIRED_FLAGS "-msse4.1")
  endif()
  check_cxx_source_runs("
  #include <emmintrin.h>
  #include <smmintrin.h>
  int main()
  {
    long long a[2] = {  1, 2 };
    long long b[2] = { -1, 2 };
    long long c[2];
    __m128i va = _mm_loadu_si128((__m128i*)a);
    __m128i vb = _mm_loadu_si128((__m128i*)b);
    __m128i vc = _mm_cmpeq_epi64(va, vb);

    _mm_storeu_si128((__m128i*)c, vc);
    if (c[0] == 0LL && c[1] == -1LL)
      return 0;
    else
      return 1;
  }" DETECTED_SSE_41)
endif()

# Check for SSE 3 support.
if(_SSE_TEST_30)
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_REQUIRED_FLAGS "-msse3")
  endif()
  check_cxx_source_runs("
  #include <emmintrin.h>
  #ifdef _WIN32
    #include <intrin.h>
  #else
    #include <x86intrin.h>
  #endif

  int main()
  {
    float a[4] = { 1.0f, 2.0f, 3.0f, 4.0f };
    float b[4] = { 3.0f, 5.0f, 7.0f, 9.0f };
    float c[4];

    __m128 va = _mm_loadu_ps(a);
    __m128 vb = _mm_loadu_ps(b);
    __m128 vc = _mm_hadd_ps(va, vb);

    _mm_storeu_ps(c, vc);
    if (c[0] == 3.0f && c[1] == 7.0f && c[2] == 8.0f && c[3] == 16.0f)
      return 0;
    else
      return 1;
  }" DETECTED_SSE_30)
endif()

# Check for SSE2 support.
if(_SSE_TEST_20)
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_REQUIRED_FLAGS "-msse2")
  elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(CMAKE_REQUIRED_FLAGS "/arch:SSE2")
  endif()
  check_cxx_source_runs("
  #include <emmintrin.h>
  int main()
  {
    int a[4] = { 1, 2,  3,  4 };
    int b[4] = { 3, 6, -4, -4 };
    int c[4];

    __m128i va = _mm_loadu_si128((__m128i*)a);
    __m128i vb = _mm_loadu_si128((__m128i*)b);
    __m128i vc = _mm_add_epi32(va, vb);

    _mm_storeu_si128((__m128i*)c, vc);
    if (c[0] == 4 && c[1] == 8 && c[2] == -1 && c[3] == 0)
      return 0;
    else
      return 1;
  }" DETECTED_SSE_20)
endif()

# Check for SSE support.
if(_SSE_TEST_10)
  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_REQUIRED_FLAGS "-msse")
  elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(CMAKE_REQUIRED_FLAGS "/arch:SSE")
  endif()
  check_cxx_source_runs("
  #include <emmintrin.h>
  int main()
  {
    float a[4] = { 1.0f, 2.0f, 3.0f, 4.0f };
    float b[4] = { 2.0f, 3.0f, 4.0f, 5.0f };
    float c[4];
    __m128 va = _mm_loadu_ps(a);
    __m128 vb = _mm_loadu_ps(b);
    __m128 vc = _mm_add_ps(va, vb);

    _mm_storeu_ps(c, vc);
    if (c[0] == 3.0f && c[1] == 5.0f && c[2] == 7.0f && c[3] == 9.0f)
      return 0;
    else
      return 1;
  }" DETECTED_SSE_10)
endif()

set(CMAKE_REQUIRED_FLAGS)

if(DETECTED_SSE_42)
   set(SSE_VERSION "4.2")
   set(SSE_STR "4_2")
   set(SSE_FOUND 1)
elseif(DETECTED_SSE_41)
   set(SSE_VERSION "4.1")
   set(SSE_STR "4_1")
   set(SSE_FOUND 1)
elseif(DETECTED_SSE_30)
   set(SSE_VERSION "3.0")
   set(SSE_STR "3_0")
   set(SSE_FOUND 1)
elseif(DETECTED_SSE_20)
   set(SSE_VERSION "2.0")
   set(SSE_STR "2_0")
   set(SSE_FOUND 1)
elseif(DETECTED_SSE_10)
   set(SSE_VERSION "1.0")
   set(SSE_STR "1_0")
   set(SSE_FOUND 1)
endif()


if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(DETECTED_SSE_42)
     set(SSE_FLAGS "${SSE_FLAGS} -msse4.2 -mfpmath=sse")
  elseif(DETECTED_SSE_41)
     set(SSE_FLAGS "${SSE_FLAGS} -msse4.1 -mfpmath=sse")
  elseif(DETECTED_SSE_30)
     set(SSE_FLAGS "${SSE_FLAGS} -msse3 -mfpmath=sse")
  elseif(DETECTED_SSE_20)
     set(SSE_FLAGS "${SSE_FLAGS} -msse2 -mfpmath=sse")
  elseif(DETECTED_SSE_10)
     set(SSE_FLAGS "${SSE_FLAGS} -msse -mfpmath=sse")
  else()
     # Setting -ffloat-store to alleviate 32bit vs 64bit discrepancies on non-SSE platforms.
     set(SSE_FLAGS "-ffloat-store")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
  set(SSE_FLAGS "-xHost")
elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
  if(DETECTED_SSE_20)
     set(SSE_FLAGS "${SSE_FLAGS} /arch:SSE2")
  elseif(DETECTED_SSE_10)
     set(SSE_FLAGS "${SSE_FLAGS} /arch:SSE")
  endif()
endif()

if(SSE_FOUND)
  message(STATUS "  Found SSE ${SSE_VERSION} extensions, using flags: ${SSE_FLAGS}")
else()
  message(STATUS "  No SSE support found")
  set(SSE_FLAGS "")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${SSE_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${SSE_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${SSE_FLAGS}")

return()
#-------------------------------------

# If no SSE support is found, print an error message.
if(SSE_FIND_VERSION)
  set(_SSE_ERROR_MESSAGE "SSE ${SSE_FIND_VERSION} support is not found on this architecture")
else()
  set(_SSE_ERROR_MESSAGE "SSE support is not found on this architecture")
endif()

if(SSE_FIND_REQUIRED)
  message(FATAL_ERROR "${_SSE_ERROR_MESSAGE}")
elseif(NOT SSE_FIND_QUIETLY)
  message(STATUS "${_SSE_ERROR_MESSAGE}")
endif()
