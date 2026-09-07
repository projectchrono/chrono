#=============================================================================
# Chrono GPU backend resolution -- Layer 1: toolchains (helpers)
#
# Answers: WHICH GPU SDKs ARE USABLE FOR THE RESOLVED VENDOR?
#
# The important property of this layer, and the one the whole redesign rests
# on, is that its outputs are INDEPENDENT FACTS rather than a single exclusive
# choice:
#
#   CHRONO_CUDA_FOUND    TRUE | FALSE
#   CHRONO_HIP_FOUND     TRUE | FALSE
#   CHRONO_HIP_PLATFORM  amd | nvidia | ""
#
# Both found-flags may be TRUE at once. The previous design could not represent
# that state, which is precisely why an NVIDIA machine that happened to have
# ROCm installed (for PyTorch, or pulled in transitively, or in a multi-SDK CI
# image) would silently configure a HIP/AMD build and then fail to compile.
#
# Consumers must NOT read these flags as "which backend am I". They say only
# "this SDK is available". Deciding what any given module actually compiles as
# is Layer 2's job -- see ChronoGPUModule.cmake.
#
# This file defines PURE FUNCTIONS only. The detection that actually enables
# languages lives in ChronoGPUDetect.cmake, because enable_language() is only
# valid at file scope -- calling it inside a function configures without
# complaint and then fails at generate time with a missing internal variable.
#=============================================================================

include_guard(GLOBAL)

set(CHRONO_CUDA_ARCHITECTURES "" CACHE STRING "Chrono CUDA architectures")
set(CHRONO_HIP_ARCHITECTURES "" CACHE STRING "Chrono HIP architectures")
set(CHRONO_ROCM_ROOT "" CACHE PATH "Explicit ROCm root for HIP builds")
set(CHRONO_HIP_MIN_VERSION "5.7.0" CACHE STRING "Minimum HIP version required by Chrono")

# HIP targeting NVIDIA hardware (CMAKE_HIP_PLATFORM=nvidia).
#
# Ships ON. This shipped OFF when the resolution model was introduced, as a
# staging decision: the model could represent the state, but nothing in-tree
# used it, and the default was to be flipped by the first module whose GPU
# sources are HIP-only, so that it would be reviewed against a real consumer.
#
# Chrono::Vehicle's SCM GPU backend is that consumer. Its kernels exist only in
# HIP, so on NVIDIA hardware "is HIP available?" and "can this feature use the
# GPU at all?" are the same question. Leaving this OFF would mean a machine
# with a GPU and a working toolchain silently building the CPU path until the
# user discovered an option they had no reason to look for.
#
# Turning it ON only widens the search; it does not change what any existing
# module compiles as. Chrono::DEM and Chrono::FSI::SPH still PREFER CUDA, so on
# NVIDIA they resolve to CUDA exactly as before -- HIP merely becomes an
# available fact rather than an unasked question. The search itself is safe by
# construction: it is version-guarded (CMake >= 3.28), gated on finding a ROCm
# new enough to satisfy CHRONO_HIP_MIN_VERSION, and ends in check_language(HIP),
# so anything missing or too old leaves CHRONO_HIP_FOUND FALSE instead of
# failing the configure.
#
# Set OFF to skip the search on a machine where ROCm is present but unwanted.
option(CHRONO_ENABLE_HIP_ON_NVIDIA
       "Also look for HIP targeting NVIDIA hardware (requires CMake >= 3.28)" ON)

#-----------------------------------------------------------------------------
# ROCm discovery
#
# Moved verbatim from src/CMakeLists.txt. Also used by Layer 0's SDK-inference
# fallback, so that the two layers cannot disagree about where ROCm is.
#-----------------------------------------------------------------------------

function(chrono_read_hip_version hip_header out_var)
  if(NOT EXISTS "${hip_header}")
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()

  file(STRINGS "${hip_header}" _major REGEX "^#define[ \t]+HIP_VERSION_MAJOR[ \t]+[0-9]+")
  file(STRINGS "${hip_header}" _minor REGEX "^#define[ \t]+HIP_VERSION_MINOR[ \t]+[0-9]+")
  file(STRINGS "${hip_header}" _patch REGEX "^#define[ \t]+HIP_VERSION_PATCH[ \t]+[0-9]+")

  if(NOT _major OR NOT _minor OR NOT _patch)
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()

  string(REGEX REPLACE ".*HIP_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" _maj "${_major}")
  string(REGEX REPLACE ".*HIP_VERSION_MINOR[ \t]+([0-9]+).*" "\\1" _min "${_minor}")
  string(REGEX REPLACE ".*HIP_VERSION_PATCH[ \t]+([0-9]+).*" "\\1" _pat "${_patch}")
  set(${out_var} "${_maj}.${_min}.${_pat}" PARENT_SCOPE)
endfunction()

function(chrono_find_rocm out_root out_version)
  set(_roots)
  if(CHRONO_ROCM_ROOT)
    list(APPEND _roots "${CHRONO_ROCM_ROOT}")
  endif()
  if(DEFINED ROCM_PATH AND NOT ROCM_PATH STREQUAL "")
    list(APPEND _roots "${ROCM_PATH}")
  endif()
  if(DEFINED ENV{ROCM_PATH} AND NOT "$ENV{ROCM_PATH}" STREQUAL "")
    list(APPEND _roots "$ENV{ROCM_PATH}")
  endif()
  if(DEFINED ENV{ROCM_HOME} AND NOT "$ENV{ROCM_HOME}" STREQUAL "")
    list(APPEND _roots "$ENV{ROCM_HOME}")
  endif()
  if(DEFINED ENV{HIP_PATH} AND NOT "$ENV{HIP_PATH}" STREQUAL "")
    list(APPEND _roots "$ENV{HIP_PATH}")
  endif()
  list(APPEND _roots /opt/rocm /usr/lib/rocm /usr/local/rocm)

  foreach(_root IN LISTS _roots)
    if(NOT _root)
      continue()
    endif()
    set(_header "")
    if(EXISTS "${_root}/include/hip/hip_version.h")
      set(_header "${_root}/include/hip/hip_version.h")
    elseif(EXISTS "${_root}/hip/include/hip/hip_version.h")
      set(_header "${_root}/hip/include/hip/hip_version.h")
    endif()
    if(_header)
      chrono_read_hip_version("${_header}" _ver)
      if(_ver)
        set(${out_root} "${_root}" PARENT_SCOPE)
        set(${out_version} "${_ver}" PARENT_SCOPE)
        return()
      endif()
    endif()
  endforeach()

  set(${out_root} "" PARENT_SCOPE)
  set(${out_version} "" PARENT_SCOPE)
endfunction()

#-----------------------------------------------------------------------------
# Architecture policy
#
# "native" asks the compiler to detect the GPU installed in THIS machine. In a
# cross-target build there is no such GPU, so the request is not merely
# suboptimal -- it is unanswerable, and both nvcc and hipcc respond to it with
# errors that point nowhere near the actual cause.
#
# This is the highest-value guard in the redesign for anyone doing distribution
# builds: it turns a confusing mid-compile failure into a configure-time error
# that names the variable to set.
#-----------------------------------------------------------------------------

function(chrono_guard_cross_target_arch lang var_name)
  if(NOT CHRONO_GPU_CROSS_TARGET)
    return()
  endif()

  set(_context
      "Chrono is being built for ${CHRONO_GPU_VENDOR_RESOLVED}, but no "
      "${CHRONO_GPU_VENDOR_RESOLVED} GPU was detected on this machine "
      "(host GPU: ${CHRONO_GPU_HOST_VENDOR}).")
  string(REPLACE ";" "" _context "${_context}")

  if("${${var_name}}" STREQUAL "native")
    # Genuinely unanswerable: "native" asks the compiler to interrogate a local
    # device that by definition is not here. Refusing beats emitting a compiler
    # error that points nowhere near the cause.
    message(FATAL_ERROR
      "${_context}\n"
      "  ${lang} 'native' architecture detection cannot work in a cross-target build.\n"
      "  Set ${var_name} explicitly, for example -D${var_name}=\"<arch>\".")
  elseif("${${var_name}}" STREQUAL "")
    # Merely unspecified, which is NOT the same thing. Building GPU code inside
    # a container with no GPU attached is an entirely normal CI pattern, and it
    # worked before this change, so this must not become a hard failure. Each
    # caller keeps its own existing handling for empty architectures.
    message(WARNING
      "${_context}\n"
      "  No ${lang} architectures are set, and none can be detected locally. "
      "Set ${var_name} explicitly to choose what this build targets.")
  endif()
endfunction()

#-----------------------------------------------------------------------------
# chrono_report_missing_gpu_toolchain()
#
# Signpost the dead end.
#
# Resolving a vendor and then finding no matching toolchain produces a build
# with every GPU module silently absent. That is the CORRECT outcome under a
# hardware-first policy -- binaries built with the other vendor's SDK could not
# run on this machine -- but "correct" and "obvious" are different things, and
# the legitimate version of this scenario (building here to deploy elsewhere)
# is exactly what the explicit override exists for. Say so, by name.
#-----------------------------------------------------------------------------

function(chrono_report_missing_gpu_toolchain)
  if(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "NONE")
    return()
  endif()
  if(CHRONO_CUDA_FOUND OR CHRONO_HIP_FOUND)
    return()
  endif()

  if(CHRONO_GPU_VENDOR_RESOLVED STREQUAL "NVIDIA")
    set(_other "AMD")
  else()
    set(_other "NVIDIA")
  endif()

  message(WARNING
    "Building for ${CHRONO_GPU_VENDOR_RESOLVED}, but no usable "
    "${CHRONO_GPU_VENDOR_RESOLVED} GPU toolchain was found. All GPU-dependent "
    "Chrono modules will be disabled.\n"
    "  If you meant to build for ${_other} hardware (for example, to deploy "
    "elsewhere), configure with -DCHRONO_GPU_VENDOR=${_other}.")
endfunction()
