#=============================================================================
# Chrono GPU backend resolution -- Layer 0: vendor
#
# Answers exactly one question: WHICH HARDWARE VENDOR ARE WE COMPILING FOR?
#
# This is deliberately a "declared-or-inferred" fact rather than a strictly
# probed one, because build-time and run-time hardware visibility legitimately
# differ (CI runners, containers, cross-compilation, distribution builds).
#
# Two distinct facts are published:
#
#   CHRONO_GPU_VENDOR_RESOLVED  what we COMPILE FOR.  Authoritative.
#   CHRONO_GPU_HOST_VENDOR      what is PHYSICALLY PRESENT on the build host.
#                               Advisory / diagnostic only -- it must never
#                               gate a module.
#
# When they disagree we are producing binaries that cannot run here. That is a
# legitimate and supported mode (building on an NVIDIA workstation to ship for
# an AMD cluster), but it has consequences -- notably that "native" GPU
# architecture detection is meaningless -- so it is named explicitly:
#
#   CHRONO_GPU_CROSS_TARGET     TRUE when the target vendor is not known to be
#                               present on this machine.
#
# Resolution order (first match wins):
#   1. explicit CHRONO_GPU_VENDOR  -> used verbatim, no detection runs at all
#   2. build-host hardware probe   -> skipped when cross-compiling
#   3. installed SDK inference     -> weak evidence, used only as a fallback
#   4. NONE
#=============================================================================

include_guard(GLOBAL)

# For chrono_find_rocm(), reused by the SDK-inference fallback below so that
# Layer 0 and Layer 1 cannot disagree about where ROCm is.
include(ChronoGPUToolchains)

set(CHRONO_GPU_VENDOR "AUTO" CACHE STRING
    "GPU hardware vendor to build for: AUTO, NVIDIA, AMD, NONE")
set_property(CACHE CHRONO_GPU_VENDOR PROPERTY STRINGS AUTO NVIDIA AMD NONE)

#-----------------------------------------------------------------------------
# Hardware probes
#
# Non-negotiable property: a probe can never break a build. Every one of them
# is best-effort (ERROR_QUIET, an explicit TIMEOUT, return codes checked), and
# a wrong answer can only ever produce a wrong DEFAULT -- never a hard failure,
# because an explicit CHRONO_GPU_VENDOR bypasses all of this.
#
# Results are cached as INTERNAL so repeat configures do not re-spawn
# subprocesses. Blow away the build tree to re-probe.
#-----------------------------------------------------------------------------

function(_chrono_probe_nvidia_hardware out_found)
  if(DEFINED CHRONO_PROBED_NVIDIA_HW)
    set(${out_found} ${CHRONO_PROBED_NVIDIA_HW} PARENT_SCOPE)
    return()
  endif()

  set(_found FALSE)

  # Tier A -- driver device nodes. Free: no subprocess, no SDK, no filesystem
  # walk. These exist if and only if the NVIDIA kernel driver is loaded, which
  # includes containers started with `--gpus`.
  if(EXISTS "/dev/nvidiactl" OR EXISTS "/dev/nvidia0")
    set(_found TRUE)
  endif()

  # Tier B -- vendor tool. The TIMEOUT is mandatory, not defensive padding:
  # nvidia-smi can block indefinitely against a wedged driver or an
  # unreachable MIG configuration, and that would hang `cmake` itself.
  if(NOT _found)
    find_program(_chrono_nvidia_smi NAMES nvidia-smi)
    mark_as_advanced(_chrono_nvidia_smi)
    if(_chrono_nvidia_smi)
      execute_process(COMMAND "${_chrono_nvidia_smi}" -L
                      RESULT_VARIABLE _rc
                      OUTPUT_VARIABLE _txt
                      ERROR_QUIET
                      TIMEOUT 5)
      if(_rc EQUAL 0 AND _txt MATCHES "GPU [0-9]+:")
        set(_found TRUE)
      endif()
    endif()
  endif()

  # Tier C -- driver library, for containers where the device nodes are masked
  # but the driver is injected.
  #
  # CRITICAL: reject the CUDA toolkit's stubs/libcuda.so. That file exists
  # purely to satisfy the linker on machines with no driver at all, so treating
  # it as evidence of hardware is exactly backwards. Chrono's own build scripts
  # put the stubs directory on CMAKE_LIBRARY_PATH, so this is a live risk here
  # rather than a theoretical one.
  if(NOT _found)
    find_library(_chrono_libcuda NAMES cuda libcuda.so.1)
    mark_as_advanced(_chrono_libcuda)
    if(_chrono_libcuda AND NOT _chrono_libcuda MATCHES "/stubs/")
      set(_found TRUE)
    endif()
  endif()

  set(CHRONO_PROBED_NVIDIA_HW ${_found} CACHE INTERNAL "NVIDIA hardware probe result")
  set(${out_found} ${_found} PARENT_SCOPE)
endfunction()

function(_chrono_probe_amd_hardware out_found out_gfx)
  if(DEFINED CHRONO_PROBED_AMD_HW)
    set(${out_found} ${CHRONO_PROBED_AMD_HW} PARENT_SCOPE)
    set(${out_gfx} "${CHRONO_PROBED_AMD_GFX}" PARENT_SCOPE)
    return()
  endif()

  set(_found FALSE)
  set(_gfx "")

  # Tier A -- /dev/kfd is the amdgpu compute (KFD) device: specific, decisive,
  # and present exactly when ROCm compute is usable.
  #
  # Deliberately NOT used: /dev/dri/renderD*. Intel and NVIDIA create renderD
  # nodes too, so it is a false-positive generator, not a signal.
  if(EXISTS "/dev/kfd")
    set(_found TRUE)
  endif()

  # Tier B -- rocminfo. Also reports the gfx target, which is a far better seed
  # for CHRONO_HIP_ARCHITECTURES than "native".
  if(NOT _found OR _gfx STREQUAL "")
    find_program(_chrono_rocminfo NAMES rocminfo)
    mark_as_advanced(_chrono_rocminfo)
    if(_chrono_rocminfo)
      execute_process(COMMAND "${_chrono_rocminfo}"
                      RESULT_VARIABLE _rc
                      OUTPUT_VARIABLE _txt
                      ERROR_QUIET
                      TIMEOUT 5)
      if(_rc EQUAL 0 AND _txt MATCHES "(gfx[0-9a-f]+)")
        set(_found TRUE)
        set(_gfx "${CMAKE_MATCH_1}")
      endif()
    endif()
  endif()

  set(CHRONO_PROBED_AMD_HW ${_found} CACHE INTERNAL "AMD hardware probe result")
  set(CHRONO_PROBED_AMD_GFX "${_gfx}" CACHE INTERNAL "AMD gfx target probe result")
  set(${out_found} ${_found} PARENT_SCOPE)
  set(${out_gfx} "${_gfx}" PARENT_SCOPE)
endfunction()

#-----------------------------------------------------------------------------
# SDK inference -- the fallback evidence source
#
# This answers a different and weaker question than the hardware probe: not
# "what can run here" but "what can this toolchain compile for". It is used
# only when the hardware probe finds nothing or is inapplicable.
#
# Kept deliberately shallow (existence checks only, no version parsing, no
# compiler invocation) because Layer 1 does the real toolchain validation and
# is entitled to disagree with the guess made here.
#-----------------------------------------------------------------------------

function(_chrono_infer_vendor_from_sdks out_vendor)
  set(_have_cuda FALSE)
  set(_have_rocm FALSE)

  find_program(_chrono_nvcc NAMES nvcc HINTS ENV CUDA_PATH ENV CUDA_HOME
               PATH_SUFFIXES bin)
  mark_as_advanced(_chrono_nvcc)
  if(_chrono_nvcc OR DEFINED ENV{CUDA_PATH} OR EXISTS "/usr/local/cuda/bin/nvcc")
    set(_have_cuda TRUE)
  endif()

  # Reuse Layer 1's ROCm search so the two layers cannot disagree about where
  # ROCm is or whether it is usable at all.
  chrono_find_rocm(_rocm_root _rocm_ver)
  if(_rocm_root AND _rocm_ver)
    set(_have_rocm TRUE)
  endif()

  if(_have_cuda AND _have_rocm)
    set(${out_vendor} "BOTH" PARENT_SCOPE)
  elseif(_have_cuda)
    set(${out_vendor} "NVIDIA" PARENT_SCOPE)
  elseif(_have_rocm)
    set(${out_vendor} "AMD" PARENT_SCOPE)
  else()
    set(${out_vendor} "NONE" PARENT_SCOPE)
  endif()
endfunction()

#-----------------------------------------------------------------------------
# chrono_resolve_gpu_vendor()
#
# Publishes, in the caller's scope:
#   CHRONO_GPU_VENDOR_RESOLVED   NVIDIA | AMD | NONE
#   CHRONO_GPU_HOST_VENDOR       NVIDIA | AMD | BOTH | NONE | UNKNOWN
#   CHRONO_GPU_CROSS_TARGET      TRUE | FALSE
#   CHRONO_GPU_HOST_GFX          probed AMD gfx target, or ""
#-----------------------------------------------------------------------------

function(chrono_resolve_gpu_vendor)
  if(NOT CHRONO_GPU_VENDOR MATCHES "^(AUTO|NVIDIA|AMD|NONE)$")
    message(FATAL_ERROR
      "Invalid CHRONO_GPU_VENDOR='${CHRONO_GPU_VENDOR}'. Expected AUTO, NVIDIA, AMD, or NONE.")
  endif()

  #---------------------------------------------------------------------------
  # Host probe.
  #
  # Run unconditionally -- even when the vendor was set explicitly -- because
  # its purpose is not to decide the vendor but to tell us whether what we are
  # about to build can actually RUN here. That question still has an answer
  # (and still matters) when the vendor was declared by hand.
  #
  # Skipped only when cross-compiling: then the build host's GPU is not weak
  # evidence, it is actively misleading. A CPU-only build server cross-
  # compiling for an MI300X cluster would confidently report the wrong thing.
  #---------------------------------------------------------------------------
  set(_host "UNKNOWN")
  set(_gfx "")

  if(CMAKE_CROSSCOMPILING)
    message(STATUS "  Cross-compiling: skipping build-host GPU hardware probe.")
  elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    # No CUDA and no ROCm on macOS. Probing would only produce noise.
    set(_host "NONE")
  else()
    _chrono_probe_nvidia_hardware(_nv_hw)
    _chrono_probe_amd_hardware(_amd_hw _gfx)

    if(_nv_hw AND _amd_hw)
      set(_host "BOTH")
    elseif(_nv_hw)
      set(_host "NVIDIA")
    elseif(_amd_hw)
      set(_host "AMD")
    else()
      set(_host "NONE")
    endif()
  endif()

  #---------------------------------------------------------------------------
  # Target vendor resolution.
  #---------------------------------------------------------------------------
  set(_resolved "NONE")
  set(_how "")

  if(NOT CHRONO_GPU_VENDOR STREQUAL "AUTO")

    set(_resolved "${CHRONO_GPU_VENDOR}")
    set(_how "explicitly requested")

  else()

    if(CMAKE_CROSSCOMPILING)
      message(WARNING
        "Cross-compiling with CHRONO_GPU_VENDOR=AUTO. The target vendor will be "
        "guessed from which SDKs are installed, which is thin evidence. Set "
        "-DCHRONO_GPU_VENDOR=<NVIDIA|AMD|NONE> explicitly.")
    endif()

    if(_host STREQUAL "NVIDIA" OR _host STREQUAL "AMD")
      set(_resolved "${_host}")
      set(_how "detected hardware")
    elseif(_host STREQUAL "BOTH")
      # Both vendors physically present. There is no defensible automatic
      # answer here -- either choice silently discards half the machine's
      # capability -- and since the explicit override is a documented,
      # first-class path, demanding it costs the user almost nothing and
      # removes a silent wrong guess.
      message(FATAL_ERROR
        "Both NVIDIA and AMD GPUs were detected on this machine. Chrono builds "
        "for one vendor per build tree, and there is no safe default here.\n"
        "  Configure with -DCHRONO_GPU_VENDOR=NVIDIA or -DCHRONO_GPU_VENDOR=AMD.")
    else()
      # No usable hardware evidence: fall back to what the toolchain can
      # compile for.
      _chrono_infer_vendor_from_sdks(_sdk)
      if(_sdk STREQUAL "BOTH")
        set(_resolved "NVIDIA")
        set(_how "installed SDKs (ambiguous)")
        message(WARNING
          "No GPU hardware was detected, and BOTH the CUDA toolkit and ROCm are "
          "installed. Defaulting to NVIDIA. Set -DCHRONO_GPU_VENDOR explicitly to "
          "remove this ambiguity.")
      elseif(NOT _sdk STREQUAL "NONE")
        set(_resolved "${_sdk}")
        set(_how "installed SDKs")
      else()
        set(_resolved "NONE")
        set(_how "no GPU hardware or SDK found")
      endif()
    endif()

  endif()

  #---------------------------------------------------------------------------
  # Cross-target determination.
  #
  # TRUE means: we are building for a vendor we cannot demonstrate is present.
  # Layer 1 uses this to refuse "native" architecture detection, and the test
  # suite uses it to avoid registering GPU tests that cannot possibly pass.
  #
  # UNKNOWN host counts as cross-target. That is the conservative direction:
  # the cost is having to name an architecture explicitly, whereas trusting an
  # unverified host means emitting a confusing compiler error much later.
  #---------------------------------------------------------------------------
  set(_cross FALSE)
  if(NOT _resolved STREQUAL "NONE")
    if(_host STREQUAL "BOTH" OR _host STREQUAL "${_resolved}")
      set(_cross FALSE)
    else()
      set(_cross TRUE)
    endif()
  endif()

  set(CHRONO_GPU_VENDOR_RESOLVED "${_resolved}" PARENT_SCOPE)
  set(CHRONO_GPU_HOST_VENDOR "${_host}" PARENT_SCOPE)
  set(CHRONO_GPU_CROSS_TARGET ${_cross} PARENT_SCOPE)
  set(CHRONO_GPU_HOST_GFX "${_gfx}" PARENT_SCOPE)

  message(STATUS "  GPU target vendor:       ${_resolved} (${_how})")
  message(STATUS "  GPU hardware on host:    ${_host}")
  if(_cross)
    message(STATUS "  Cross-target build:      YES -- binaries will not run on this machine")
  endif()
endfunction()
