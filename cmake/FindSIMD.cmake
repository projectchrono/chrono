# SPDX-License-Identifier: BSD-3-Clause
#
#	This script combines the search routines for various SIMD technologies into one place.
#


# This script checks for the highest level of SSE support on the host
# by compiling and running small C++ programs that uses SSE intrinsics.
#
# If any SSE support is detected, the following variables are set:
#
#   SSE_FOUND   = 1
#   SSE_VERSION = the highest SSE version found.
#   SSE_FLAGS = compile flags for the version of SSE found
# 
# If SSE is not supported on the host platform, these variables are
# not set.  
#
# NOTE: 64-bit x86 architectures provide SSE 2.0 or support by default so it is not tested here.

function (test_sse_availability)

	set(SSE_FLAGS)
	set(SSE_FOUND)
	set(DETECTED_SSE_41)
	set(DETECTED_SSE_42)
	set(DETECTED_SSE_30)

	include(CheckCXXSourceRuns)
	set(CMAKE_REQUIRED_FLAGS)

	set(_SSE_TEST_42 1)
	set(_SSE_TEST_41 1)
	set(_SSE_TEST_30 1)

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
	endif()


	if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	  	if(DETECTED_SSE_42)
		  	set(SSE_FLAGS "${SSE_FLAGS} -msse4.2 -mfpmath=sse")
	  	endif()
	  	if(DETECTED_SSE_41)
			set(SSE_FLAGS "${SSE_FLAGS} -msse4.1 -mfpmath=sse")
		endif()
	  	if(DETECTED_SSE_30)
		  set(SSE_FLAGS "${SSE_FLAGS} -msse3 -mfpmath=sse")
		endif()
	elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		# clang does not require "-mfpmath" which is automatic
		if(DETECTED_SSE_42)
			set(SSE_FLAGS "${SSE_FLAGS} -msse4.2")
		endif()
		if(DETECTED_SSE_41)
			set(SSE_FLAGS "${SSE_FLAGS} -msse4.1")
		endif()
		if(DETECTED_SSE_30)
			set(SSE_FLAGS "${SSE_FLAGS} -msse3")
		endif()
	elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
		set(SSE_FLAGS "-xHost")
	elseif(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
	endif()

	# Export flags to caller scope 
	if(SSE_FOUND)
		set(SSE_FOUND TRUE PARENT_SCOPE)
		set(SSE_FLAGS "${SSE_FLAGS}" PARENT_SCOPE)
		set(SSE_VERSION "${SSE_VERSION}" PARENT_SCOPE)
	else()
		set(SSE_FOUND FALSE PARENT_SCOPE)
	  	set(SSE_FLAGS "")
	endif()

	return()

endfunction()

# This script checks for the highest level of FMA support on the host
# by compiling and running small C++ programs that uses FMA intrinsics.

# If any  FMA support is detected, the following variables are set:
#
#   FMA_FOUND   = 1
#   FMA_FLAGS = compile flags for the version of FMA found
# 
# If FMA is not supported on the host platform, these variables are
# not set.  

function (test_fma_availability)
	set(FMA_FLAGS)
	set(FMA_FOUND)
	set(DETECTED_FMA)

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

	if(DETECTED_FMA)
	  SET(FMA_FOUND 1)
	  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		SET(FMA_FLAGS "${FMA_FLAGS} -mfma")
	  endif()
	endif()

	if(FMA_FOUND)
	  	set(FMA_FOUND TRUE PARENT_SCOPE)
		set(FMA_FLAGS "${FMA_FLAGS}" PARENT_SCOPE)
	else()
		set(FMA_FOUND FALSE PARENT_SCOPE)
		set(FMA_FLAGS "")
	endif()


	return()
endfunction()


# This script checks for the highest level of AVX support on the host
# by compiling and running small C++ programs that use AVX intrinsics.
#
# If any AVX support is detected, the following variables are set:
#
#   AVX_FOUND   = 1
#   AVX_VERSION = the requested version, if EXACT is true, or
#                 the highest AVX version found.
#   AVX_FLAGS = compile flags for the version of AVX found
# 
# If AVX is not supported on the host platform, these variables are
# not set.  

function(test_avx_availability)

	set(AVX_FLAGS)
	set(AVX_FOUND)
	set(DETECTED_AVX_10)
	set(DETECTED_AVX_20)

	include(CheckCXXSourceRuns)
	set(CMAKE_REQUIRED_FLAGS)

	set(_AVX_TEST_20 1)
	set(_AVX_TEST_10 1)

# Check for AVX2 support.
	if(_AVX_TEST_20)
	  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		set(CMAKE_REQUIRED_FLAGS "-mavx2")
	  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
		set(CMAKE_REQUIRED_FLAGS "-xHost")
	  elseif(MSVC AND NOT CMAKE_CL_64)
		set(CMAKE_REQUIRED_FLAGS "/arch:AVX2")
	  endif()
	  check_cxx_source_runs("
	  #include <immintrin.h>
	  int main()
		{
		__m256i a = _mm256_set_epi32 (-1, 2, -3, 4, -1, 2, -3, 4);
		__m256i result = _mm256_abs_epi32 (a);
		return 0;
		}" DETECTED_AVX_20)
	endif()

# Check for AVX support.
	if(_AVX_TEST_10)
	  if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		set(CMAKE_REQUIRED_FLAGS "-mavx")
	  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
		set(CMAKE_REQUIRED_FLAGS "-xHost")
	  elseif(MSVC AND NOT CMAKE_CL_64)
		set(CMAKE_REQUIRED_FLAGS "/arch:AVX")
	  endif()
	  check_cxx_source_runs("
	  #include <immintrin.h>
	  int main()
		{
		__m256 a = _mm256_set_ps (-1.0f, 2.0f, -3.0f, 4.0f, -1.0f, 2.0f, -3.0f, 4.0f);
		__m256 b = _mm256_set_ps (1.0f, 2.0f, 3.0f, 4.0f, 1.0f, 2.0f, 3.0f, 4.0f);
		__m256 result = _mm256_add_ps (a, b);
		return 0;
		}" DETECTED_AVX_10)
	endif()

	set(CMAKE_REQUIRED_FLAGS)


	if(DETECTED_AVX_20)
		set(AVX_VERSION "2.0")
		set(AVX_STR "2_0")
		set(AVX_FOUND 1)
	elseif(DETECTED_AVX_10)
		set(AVX_VERSION "1.0")
		set(AVX_STR "1_0")
		set(AVX_FOUND 1)
	endif()


	if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		if(DETECTED_AVX_20)
			SET(AVX_FLAGS "${AVX_FLAGS} -mavx2")
		endif()
	  	if(DETECTED_AVX_10)
			SET(AVX_FLAGS "${AVX_FLAGS} -mavx")
	  	endif()
	elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
		set(AVX_FLAGS "-xHost")
	elseif(MSVC)
		if(DETECTED_AVX_20)
			SET(AVX_FLAGS "${AVX_FLAGS} /arch:AVX2")
		endif()
		if(DETECTED_AVX_10)
			SET(AVX_FLAGS "${AVX_FLAGS} /arch:AVX")
		endif()
	endif()

	if(AVX_FOUND)
		set(AVX_FOUND TRUE PARENT_SCOPE)
		set(AVX_FLAGS "${AVX_FLAGS}" PARENT_SCOPE)
		set(AVX_VERSION "${AVX_VERSION}" PARENT_SCOPE)
	else()
		set(AVX_FOUND FALSE PARENT_SCOPE)
		set(AVX_FLAGS "")
	endif()


	return()

endfunction()

# This script checks for the highest level of NEON support on the host
# by compiling and running small C++ programs that uses NEON intrinsics.
#
# If any NEON support is detected, the following variables are set:
#
#   NEON_FOUND   = 1
#   NEON_VERSION = 2_0 (assumes Advanced SIMD 2.0)
#   NEON_FLAGS = compile flags for the version of NEON found
# 
# If NEON is not supported on the host platform, these variables are
# not set.
# 

function(test_neon_availability)

	set(NEON_FLAGS)
	set(NEON_FOUND)
	set(DETECTED_NEON)

	include(CheckCXXSourceRuns)

	set(CMAKE_REQUIRED_FLAGS "-march=armv8-a")
# Check for NEON support.
	check_cxx_source_runs("
#include <arm_neon.h>
	int main()
	{
	  float64_t a[2] = {  1., 2. };
	  float64_t b[2] = { -1., 3. };
	  float64_t c[2];

	  float64x2_t va = vld1q_f64(&a[0]);
	  float64x2_t vb = vld1q_f64(&b[0]);
	  float64x2_t vc = vaddq_f64(va, vb);
	  vst1q_f64(&c[0], vc);

	  if (c[0] == 0. && c[1] == 5.)
		return 0;
	  else
		return 0;
	}
	" DETECTED_NEON)

	set(CMAKE_REQUIRED_FLAGS)

	if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	  if(DETECTED_NEON)
		  SET(NEON_FLAGS "-march=armv8-a")
		  set(NEON_STR "2_0")
		  SET(NEON_FOUND 1)
		else()
		  # Setting -ffloat-store to alleviate 32bit vs 64bit discrepancies on non-SIMD platforms.
		  set(NEON_FLAGS "-ffloat-store")
	  endif()
	endif()

	if(NEON_FOUND)
		set(NEON_FOUND TRUE PARENT_SCOPE)
		set(NEON_FLAGS "${NEON_FLAGS}" PARENT_SCOPE)
	else()
		set(NEON_FOUND FALSE PARENT_SCOPE)
		set(NEON_FLAGS "")
	endif()

	return()

endfunction()



###
#
#	Perform SIMD checks as defined above
#
###

set(SIMD_FLAGS "")

set(SIMD_SSE "FALSE" CACHE STRING "Any detected SSE SIMD version, else FALSE")
set(SIMD_AVX "FALSE" CACHE STRING "Any detected AVX SIMD version, else FALSE")
set(SIMD_FMA  ${FMA_FOUND}  CACHE BOOL "Whether AVX2 FMA extensions were a detected SIMD feature")
set(SIMD_NEON ${NEON_FOUND} CACHE BOOL "Whether NEON was a detected SIMD feature")

# Check availability of SSE instructions
test_sse_availability()
if (SSE_FOUND)
	if (NOT ${SIMD_FIND_QUIETLY})
		message(STATUS "Target supports SSE instructions")
	endif()

	set(SIMD_FLAGS "${SIMD_FLAGS} ${SSE_FLAGS}")
	set(SIMD_SSE "${SSE_VERSION}")
endif()

test_avx_availability()
if (AVX_FOUND) 
	if (NOT ${SIMD_FIND_QUIETLY})
		message(STATUS "Target supports AVX instructions") 
	endif()

	set(SIMD_FLAGS "${SIMD_FLAGS} ${AVX_FLAGS}") 
	set(SIMD_AVX "${AVX_VERSION}")
endif()

test_fma_availability()
if (FMA_FOUND) 
	if (NOT ${SIMD_FIND_QUIETLY})
		message(STATUS "Target supports AVX2 FMA instructions") 
	endif()

	set(SIMD_FLAGS "${SIMD_FLAGS} ${FMA_FLAGS}")
endif()

test_neon_availability()
if (NEON_FOUND) 
	if (NOT ${SIMD_FIND_QUIETLY})
		message(STATUS "Target supports NEON instructions") 
	endif()

	set(SIMD_FLAGS "${SIMD_FLAGS} ${NEON_FLAGS}")
endif()

# Determine whether to use SIMD flags or automatic detection 
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE GCC_VERSION_STRING)
	if(GCC_VERSION_STRING VERSION_GREATER 4.2 AND NOT APPLE AND NOT CMAKE_CROSSCOMPILING)
		SET(SIMD_FLAGS "-march=native")
		if (NOT SIMD_FIND_QUIETLY)
			message(STATUS "Using automatic native flag for SIMD optimization")
		endif()
	endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT CMAKE_CROSSCOMPILING)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE CLANG_VERSION_STRING)
	if(CLANG_VERSION_STRING VERSION_GREATER_EQUAL 15.0 AND NOT CMAKE_CROSSCOMPILING)
		SET(SIMD_FLAGS "-march=native")
		if (NOT SIMD_FIND_QUIETLY)
			message(STATUS "Using automatic native flag for SIMD optimization")
		endif()
	elseif(CMAKE_HOST_UNIX)
		execute_process(COMMAND uname -m OUTPUT_VARIABLE UNIX_MACHINE_ARCH)
		if(UNIX_MACHINE_ARCH MATCHES "x86_64|x86|amd64")
			SET(SIMD_FLAGS "-march=native")
			if (NOT SIMD_FIND_QUIETLY)
				message(STATUS "Using automatic native flag for SIMD optimization")
			endif()
		endif()
	endif()
endif()


set(SIMD_C_FLAGS "${SIMD_FLAGS}" CACHE STRING "Flags used for compiling C programs with SIMD support")
set(SIMD_CXX_FLAGS "${SIMD_FLAGS}" CACHE STRING "Flags used for compiling C++ programs with SIMD support")

mark_as_advanced(SIMD_SSE SIMD_AVX SIMD_FMA SIMD_NEON SIMD_C_FLAGS SIMD_CXX_FLAGS)

