#=============================================================================
# Chrono GPU backend detection -- orchestrator
#
# This file is include()d from src/CMakeLists.txt and DELIBERATELY EXECUTES AT
# FILE SCOPE rather than wrapping its body in a function.
#
# That is not a style choice. enable_language() is only valid at file scope:
# called from inside a function it configures without complaint and then fails
# at generate time with "required internal CMake variable not set ...
# CMAKE_CUDA_COMPILE_WHOLE_COMPILATION", and CMAKE_CUDA_ARCHITECTURES_ALL_MAJOR
# never gets defined. include() runs in the includer's scope, so the language
# enabling below lands in src/ directory scope exactly as it did when this code
# was inline.
#
# Everything that does NOT touch language state lives in pure functions in
# ChronoGPUVendor.cmake (Layer 0) and ChronoGPUToolchains.cmake (Layer 1).
#
# Publishes into src/ directory scope, and therefore into every module:
#   CHRONO_GPU_VENDOR_RESOLVED   NVIDIA | AMD | NONE
#   CHRONO_GPU_HOST_VENDOR       NVIDIA | AMD | BOTH | NONE | UNKNOWN
#   CHRONO_GPU_CROSS_TARGET      TRUE | FALSE
#   CHRONO_CUDA_FOUND            independent fact
#   CHRONO_HIP_FOUND             independent fact
#   CHRONO_HIP_PLATFORM          amd | nvidia | ""
#=============================================================================

include(CheckLanguage)
include(ChronoGPUToolchains)
include(ChronoGPUVendor)

mark_as_advanced(FORCE CMAKE_CUDA_ARCHITECTURES)
mark_as_advanced(FORCE CMAKE_HIP_ARCHITECTURES)

message(STATUS "Searching for GPU backends")

#-----------------------------------------------------------------------------
# Layer 0 -- vendor
#-----------------------------------------------------------------------------

chrono_resolve_gpu_vendor()

#-----------------------------------------------------------------------------
# Layer 1 -- toolchains
#
# Note the shape here: these are two independent searches, not an if/else. On
# NVIDIA, CUDA is searched and HIP MAY ALSO be searched; the outcome of one
# does not suppress the other. The old code gated the CUDA search behind
# "if(NOT CHRONO_HIP_FOUND)", which is what allowed a stray ROCm install to
# hide a perfectly good CUDA toolkit.
#-----------------------------------------------------------------------------

set(CHRONO_CUDA_FOUND FALSE)
set(CHRONO_HIP_FOUND FALSE)
set(CHRONO_HIP_PLATFORM "")
set(CHRONO_ROCM_VERSION "")
set(_chrono_hip_platform_request "")

#--- CUDA -------------------------------------------------------------------
# Searched only for NVIDIA. On an AMD build a CUDA toolkit that happens to be
# installed tells us nothing useful, and probing for it would only emit a
# confusing "CUDA not found" diagnostic about something we never wanted.

if(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "NVIDIA")

  message(STATUS "Searching for CUDA")
  check_language(CUDA)

  if(CMAKE_CUDA_COMPILER)

    enable_language(CUDA)
    find_package(CUDAToolkit)
    message(STATUS "  CUDA compiler:           ${CMAKE_CUDA_COMPILER}")
    message(STATUS "  CUDA toolkit version:    ${CUDAToolkit_VERSION}")
    message(STATUS "  CUDA toolkit root dir:   ${CUDAToolkit_LIBRARY_ROOT}")

    if(CHRONO_CUDA_ARCHITECTURES STREQUAL "")
      # all-major is a fat-binary default and so is already correct for
      # distribution builds; CUDA needs no cross-target special case here,
      # unlike HIP below.
      if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.23")
        set(CHRONO_CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES_ALL_MAJOR}"
            CACHE STRING "Chrono CUDA architectures" FORCE)
        message(STATUS "  CUDA archs (set to):     ${CHRONO_CUDA_ARCHITECTURES}")
      endif()
    else()
      message(STATUS "  CUDA archs (from cache): ${CHRONO_CUDA_ARCHITECTURES}")
    endif()

    chrono_guard_cross_target_arch("CUDA" CHRONO_CUDA_ARCHITECTURES)

    # Architecture 50 has no double-precision atomicAdd.
    list(REMOVE_ITEM CHRONO_CUDA_ARCHITECTURES "50" "50-real")
    message(STATUS "  CUDA archs (filtered):   ${CHRONO_CUDA_ARCHITECTURES}")

    if(CHRONO_CUDA_ARCHITECTURES STREQUAL "")
      message(WARNING "  CUDA architectures not found. Set CHRONO_CUDA_ARCHITECTURES")
    else()
      set(CHRONO_CUDA_FOUND TRUE)
      set(CMAKE_CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES} CACHE STRING "CUDA architectures" FORCE)
      if(CMAKE_SYSTEM_NAME MATCHES "Windows")
        set(CUDA_SEPARABLE_COMPILATION OFF)
      endif()
      message(STATUS "  CUDA found and enabled.")
    endif()

  else()

    mark_as_advanced(FORCE CHRONO_CUDA_ARCHITECTURES)
    message(STATUS "  CUDA not found. CUDA features will be disabled.")

  endif()

  #--- HIP on NVIDIA (secondary, opt-in) ------------------------------------
  if(CHRONO_ENABLE_HIP_ON_NVIDIA)
    # CMake gained HIP language support in 3.21, but for the AMD platform only.
    # Driving the HIP language with CMAKE_HIP_PLATFORM=nvidia needs 3.28.
    # Refusing here is better than letting check_language(HIP) fail in a way
    # that reads like a broken ROCm install.
    if(CMAKE_VERSION VERSION_LESS "3.28")
      message(WARNING
        "CHRONO_ENABLE_HIP_ON_NVIDIA requires CMake >= 3.28 (have ${CMAKE_VERSION}); "
        "HIP on NVIDIA will not be enabled.")
    else()
      set(_chrono_hip_platform_request "nvidia")
    endif()
  endif()

elseif(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "AMD")

  set(_chrono_hip_platform_request "amd")

else()

  message(STATUS "  No GPU vendor resolved; skipping GPU toolchain search.")

endif()

#--- HIP --------------------------------------------------------------------

if(_chrono_hip_platform_request)

  message(STATUS "Searching for HIP (platform: ${_chrono_hip_platform_request})")

  chrono_find_rocm(_chrono_rocm_root _chrono_rocm_ver)

  if(NOT _chrono_rocm_root)

    message(STATUS "  ROCm not found. HIP features will be disabled.")

  elseif(_chrono_rocm_ver VERSION_LESS CHRONO_HIP_MIN_VERSION)

    message(WARNING
      "  Found ROCm ${_chrono_rocm_ver} at ${_chrono_rocm_root}, but Chrono requires "
      ">= ${CHRONO_HIP_MIN_VERSION}. HIP features will be disabled.")

  else()

    list(PREPEND CMAKE_PREFIX_PATH "${_chrono_rocm_root}")
    if(NOT DEFINED CMAKE_HIP_COMPILER_ROCM_ROOT)
      set(CMAKE_HIP_COMPILER_ROCM_ROOT "${_chrono_rocm_root}" CACHE PATH "ROCm root path")
    endif()
    if(NOT DEFINED CMAKE_HIP_PLATFORM)
      set(CMAKE_HIP_PLATFORM "${_chrono_hip_platform_request}" CACHE STRING "HIP platform")
    endif()

    # On the NVIDIA platform the HIP compiler IS nvcc: HIP compiles to CUDA
    # there, and ROCm ships no compiler that targets NVIDIA. check_language(HIP)
    # searches for hipcc/clang, finds neither usable, and reports "no HIP
    # compiler" -- which reads like a broken ROCm install rather than a missing
    # setting. We already located nvcc when CUDA was enabled above, so use it
    # instead of making every user pass -DCMAKE_HIP_COMPILER by hand.
    #
    # Only a default: an explicit CMAKE_HIP_COMPILER always wins.
    if(_chrono_hip_platform_request STREQUAL "nvidia"
       AND NOT DEFINED CMAKE_HIP_COMPILER
       AND CMAKE_CUDA_COMPILER)
      set(CMAKE_HIP_COMPILER "${CMAKE_CUDA_COMPILER}" CACHE FILEPATH "HIP compiler")
      message(STATUS "  HIP compiler defaulted to the CUDA compiler: ${CMAKE_HIP_COMPILER}")
    endif()

    check_language(HIP)

    if(NOT CMAKE_HIP_COMPILER)

      message(STATUS "  No HIP compiler was found. HIP features will be disabled.")

    else()

      enable_language(HIP)

      if(CHRONO_HIP_ARCHITECTURES STREQUAL "")
        # Prefer a concrete gfx target discovered by rocminfo over "native": it
        # is equally accurate here and, unlike "native", it survives being
        # copied into a preset or a CI script that runs somewhere else.
        if(CHRONO_GPU_HOST_GFX)
          set(CHRONO_HIP_ARCHITECTURES "${CHRONO_GPU_HOST_GFX}"
              CACHE STRING "Chrono HIP architectures" FORCE)
        elseif(NOT CHRONO_GPU_CROSS_TARGET AND CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
          set(CHRONO_HIP_ARCHITECTURES "native" CACHE STRING "Chrono HIP architectures" FORCE)
        endif()
      endif()

      chrono_guard_cross_target_arch("HIP" CHRONO_HIP_ARCHITECTURES)

      if(NOT CHRONO_HIP_ARCHITECTURES STREQUAL "")
        set(CMAKE_HIP_ARCHITECTURES ${CHRONO_HIP_ARCHITECTURES} CACHE STRING "HIP architectures" FORCE)
      endif()

      set(CHRONO_HIP_FOUND TRUE)
      set(CHRONO_HIP_PLATFORM "${_chrono_hip_platform_request}")
      set(CHRONO_ROCM_VERSION "${_chrono_rocm_ver}")
      set(CHRONO_ROCM_ROOT "${_chrono_rocm_root}" CACHE PATH "Explicit ROCm root for HIP builds" FORCE)

      message(STATUS "  HIP compiler:            ${CMAKE_HIP_COMPILER}")
      message(STATUS "  ROCm version:            ${CHRONO_ROCM_VERSION}")
      message(STATUS "  ROCm root dir:           ${CHRONO_ROCM_ROOT}")
      if(NOT CHRONO_HIP_ARCHITECTURES STREQUAL "")
        message(STATUS "  HIP archs:               ${CHRONO_HIP_ARCHITECTURES}")
      endif()

    endif()

  endif()

endif()

unset(_chrono_hip_platform_request)

#-----------------------------------------------------------------------------
# Summary and diagnostics
#-----------------------------------------------------------------------------

chrono_report_missing_gpu_toolchain()

# Report a global preference that cannot be honoured.
#
# CHRONO_GPU_BACKEND is documented as a preference, so quietly using the other
# backend is the correct OUTCOME -- but doing it silently is not. Someone who
# passed -DCHRONO_GPU_BACKEND=HIP and got a CUDA build with no mention of HIP
# anywhere in the log has no way to tell whether their request was honoured,
# ignored, or misspelled.
#
# Reported once here, where the toolchain facts are known, rather than once per
# feature in Layer 2: the cause is global, so repeating it per module would be
# noise around a single piece of information.
if(NOT CHRONO_GPU_BACKEND STREQUAL "AUTO")
  set(_requested_available FALSE)
  if(CHRONO_GPU_BACKEND STREQUAL "CUDA" AND CHRONO_CUDA_FOUND)
    set(_requested_available TRUE)
  elseif(CHRONO_GPU_BACKEND STREQUAL "HIP" AND CHRONO_HIP_FOUND)
    set(_requested_available TRUE)
  endif()

  if(_requested_available)
    message(STATUS "  Preferred GPU backend:   ${CHRONO_GPU_BACKEND} (available)")
  else()
    set(_hint "")
    if(CHRONO_GPU_BACKEND STREQUAL "HIP" AND CHRONO_GPU_VENDOR_RESOLVED STREQUAL "NVIDIA")
      set(_hint
          "  This build targets NVIDIA. To use HIP on NVIDIA hardware, configure with "
          "-DCHRONO_ENABLE_HIP_ON_NVIDIA=ON (requires CMake >= 3.28); to build for AMD "
          "hardware instead, use -DCHRONO_GPU_VENDOR=AMD.")
    elseif(CHRONO_GPU_BACKEND STREQUAL "CUDA" AND CHRONO_GPU_VENDOR_RESOLVED STREQUAL "AMD")
      set(_hint "  This build targets AMD, where CUDA is not searched for. Use "
                "-DCHRONO_GPU_VENDOR=NVIDIA to build for NVIDIA hardware.")
    endif()
    string(REPLACE ";" "" _hint "${_hint}")

    message(WARNING
      "CHRONO_GPU_BACKEND=${CHRONO_GPU_BACKEND} was requested, but ${CHRONO_GPU_BACKEND} is "
      "not available in this configuration (CUDA=${CHRONO_CUDA_FOUND}, HIP=${CHRONO_HIP_FOUND}). "
      "Modules will use whichever backend they can.\n"
      "${_hint}")
  endif()
endif()

# Retained for backward compatibility with anything reading a single "is there
# a GPU" flag. Under independent facts this is simply the disjunction; it must
# NOT be used to decide which backend anything compiles as.
if(CHRONO_CUDA_FOUND OR CHRONO_HIP_FOUND)
  set(CHRONO_GPU_FOUND TRUE)
else()
  set(CHRONO_GPU_FOUND FALSE)
endif()

# Hide the knobs that are irrelevant for the resolved vendor.
if(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "NVIDIA")
  mark_as_advanced(FORCE CHRONO_HIP_ARCHITECTURES)
  mark_as_advanced(FORCE CHRONO_HIP_MIN_VERSION)
  mark_as_advanced(FORCE CHRONO_ROCM_ROOT)
elseif(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "AMD")
  mark_as_advanced(FORCE CHRONO_CUDA_ARCHITECTURES)
endif()

message(STATUS "  GPU toolchains available: CUDA=${CHRONO_CUDA_FOUND} HIP=${CHRONO_HIP_FOUND}")
