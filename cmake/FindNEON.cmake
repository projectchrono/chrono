# This script checks for the highest level of NEON support on the host
# by compiling and running small C++ programs that uses NEON intrinsics.
#
# You can invoke this module using the following command:
#
#   FIND_PACKAGE(NEON)
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
set(NEON_FLAGS)
set(NEON_FOUND)
set(DETECTED_NEON)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_CXX_COMPILER} "-dumpversion" OUTPUT_VARIABLE GCC_VERSION_STRING)
  if(GCC_VERSION_STRING VERSION_GREATER 4.2 AND NOT APPLE AND NOT CMAKE_CROSSCOMPILING)
    SET(NEON_FLAGS "${NEON_FLAGS} -march=native")
    message(STATUS "Using CPU native flags for NEON optimization: ${NEON_FLAGS}")
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT CMAKE_CROSSCOMPILING)
    SET(NEON_FLAGS "${NEON_FLAGS} -march=native")
    message(STATUS "Using CPU native flags for NEON optimization: ${NEON_FLAGS}")
endif()

include(CheckCXXSourceRuns)
set(CMAKE_REQUIRED_FLAGS)
#set(CMAKE_REQUIRED_INCLUDES arm_neon.h)

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
      SET(NEON_FLAGS "${NEON_FLAGS}")
      set(NEON_STR "2_0")
      SET(NEON_FOUND 1)
    else()
      # Setting -ffloat-store to alleviate 32bit vs 64bit discrepancies on non-SIMD platforms.
      set(NEON_FLAGS "-ffloat-store")
  endif()
endif()

if(NEON_FOUND)
  message(STATUS "  Found NEON extensions, using flags: ${NEON_FLAGS}")
else()
  message(STATUS "  No NEON support found")
  set(NEON_FLAGS "")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${NEON_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${NEON_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${NEON_FLAGS}")

return()
