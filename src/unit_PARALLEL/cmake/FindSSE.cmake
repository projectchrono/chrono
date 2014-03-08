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
# 
# If SSE is not supported on the host platform, these variables are
# not set. If QUIET is true, the module does not print a message if
# SSE if missing. If REQUIRED is true, the module produces a fatal
# error if SSE support is missing.
# 

include(CheckCXXSourceRuns)

# Generate a list of SSE versions to test.
if(SSE_FIND_VERSION_EXACT)
  if(SSE_FIND_VERSION VERSION_EQUAL "4.2")
    set(_SSE_TEST_42 1)
  elseif(SSE_FIND_VERSION VERSION_EQUAL "4.1")
    set(_SSE_TEST_41 1)
  elseif(SSE_FIND_VERSION VERSION_EQUAL "3.1")
    set(_SSE_TEST_31 1)
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
  if(NOT SSE_FIND_VERSION VERSION_GREATER "3.1")
    set(_SSE_TEST_31 1)
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

# Set compiler flag to generate instructions for the host architecture.
set(CMAKE_REQUIRED_FLAGS "-march=native")

# Check for SSE 4.2 support.
if(_SSE_TEST_42)
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
  }" SSE_42_DETECTED)
  if(SSE_42_DETECTED)
    set(SSE_VERSION "4.2")
    set(SSE_FOUND 1)
    return()
  endif()
endif()

# Check for SSE 4.1 support.
if(_SSE_TEST_41)
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
  }" SSE_41_DETECTED)
  if(SSE_41_DETECTED)
    set(SSE_VERSION "4.1")
    set(SSE_FOUND 1)
    return()
  endif()
endif()

# Check for SSSE 3 support.
if(_SSE_TEST_31)
  check_cxx_source_runs("
  #include <emmintrin.h>
  #include <tmmintrin.h>
  int main()
  {

    int a[4] = { 1, 0, -3, -2 };
    int b[4];
    __m128i va = _mm_loadu_si128((__m128i*)a);
    __m128i vb = _mm_abs_epi32(va);

    _mm_storeu_si128((__m128i*)b, vb);
    if (b[0] == 1 && b[1] == 0 && b[2] == 3 && b[3] == 2)
      return 0;
    else
      return 1;
  }" SSE_31_DETECTED)
  if(SSE_31_DETECTED)
    set(SSE_VERSION "3.1")
    set(SSE_FOUND 1)
    return()
  endif()
endif()

# Check for SSE 3 support.
if(_SSE_TEST_30)
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
  }" SSE_30_DETECTED)
  if(SSE_30_DETECTED)
    set(SSE_VERSION "3.0")
    set(SSE_FOUND 1)
    return()
  endif()
endif()

# Check for SSE2 support.
if(_SSE_TEST_20)
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
  }" SSE_20_DETECTED)
  if(SSE_20_DETECTED)
    set(SSE_VERSION "2.0")
    set(SSE_FOUND 1)
    return()
  endif()
endif()

# Check for SSE support.
if(_SSE_TEST_10)
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
  }" SSE_10_DETECTED)
  if(SSE_10_DETECTED)
    set(SSE_VERSION "1.0")
    set(SSE_FOUND 1)
    return()
  endif()
endif()

# If no SSE support is found, print an error message.
if(NOT SSE_FOUND)
  
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

endif()