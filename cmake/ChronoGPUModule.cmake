#=============================================================================
# Chrono GPU backend resolution -- Layer 2: per-feature selection
#
# Answers: WHICH BACKEND DOES THIS PARTICULAR FEATURE COMPILE AS?
#
# Granularity is per FEATURE, not per module. That distinction is load-bearing
# rather than pedantic: Chrono::Sensor requires no GPU backend at all (its
# default renderer is Vulkan RT, which is vendor-neutral), while its optional
# OptiX renderer requires CUDA specifically. A module-level "Sensor requires
# CUDA" rule would wrongly disable the whole module on AMD hardware, where it
# builds and runs perfectly well.
#
# Layer 2 cannot observe HOW the vendor was decided in Layer 0, and must not
# try to. Preference engages whenever two or more toolchain facts satisfy a
# feature's REQUIRES -- a function of Layer 1's output alone. That keeps this
# layer testable by injecting fact combinations, with no hardware involved.
#=============================================================================

include_guard(GLOBAL)

# Whether an explicitly requested feature that cannot get a backend is a fatal
# error or a loud warning.
#
# Defaults OFF so this is not a breaking change for matrix builds that sweep
# -DCH_ENABLE_MODULE_*=ON across heterogeneous machines and rely on modules
# quietly dropping out. Chrono's own CI should turn it ON.
option(CHRONO_STRICT_MODULE_REQUIREMENTS
       "Treat an unsatisfiable GPU requirement for an explicitly enabled module as a fatal error" OFF)

# Global backend preference, applied when a feature could use either backend.
#
# This variable previously named the single exclusive backend for the entire
# build. It now expresses a PREFERENCE. The spelling and accepted values are
# unchanged, so anyone who was setting it explicitly keeps getting what they
# asked for; what changed is that it no longer suppresses the search for the
# backend it did not name.
set(CHRONO_GPU_BACKEND "AUTO" CACHE STRING
    "Preferred GPU backend when a module could use either: AUTO, CUDA, HIP")
set_property(CACHE CHRONO_GPU_BACKEND PROPERTY STRINGS AUTO CUDA HIP)

# Validate here rather than at the point of use. Preference is only consulted
# when two or more backends are available, so a typo would otherwise pass
# unnoticed on every single-backend machine and only surface for the one user
# who happens to have both SDKs installed.
if(NOT CHRONO_GPU_BACKEND MATCHES "^(AUTO|CUDA|HIP)$")
  message(FATAL_ERROR
    "Invalid CHRONO_GPU_BACKEND='${CHRONO_GPU_BACKEND}'. Expected AUTO, CUDA, or HIP.")
endif()

#-----------------------------------------------------------------------------
# chrono_select_gpu_backend(<out_var>
#                           FEATURE  <human readable name>
#                           REQUIRES <CUDA|HIP|CUDA_OR_HIP>
#                           [NAME    <SHORTNAME>]
#                           [PREFER  <CUDA|HIP>])
#
# Sets <out_var> to CUDA, HIP, or NONE.
#
# Precedence:
#   1. CHRONO_<SHORTNAME>_GPU_BACKEND   per-feature override  -> FATAL if unmet
#   2. CHRONO_GPU_BACKEND               global preference
#   3. PREFER                           the feature's own declared preference
#   4. the single satisfying fact
#   5. NONE
#
# A per-feature override is deliberately NOT registered as a documented cache
# entry. The mechanism exists (passing -DCHRONO_DEM_GPU_BACKEND=HIP works and
# is honoured), but every advertised cache variable is a permanent
# compatibility obligation, and nothing in-tree needs one yet. Promote it to a
# real option() when a module actually diverges.
#-----------------------------------------------------------------------------

function(chrono_select_gpu_backend out_var)
  cmake_parse_arguments(ARG "" "FEATURE;REQUIRES;NAME;PREFER;HIP_PLATFORM_NOTE" "HIP_PLATFORMS" ${ARGN})

  if(NOT ARG_FEATURE OR NOT ARG_REQUIRES)
    message(FATAL_ERROR "chrono_select_gpu_backend() requires FEATURE and REQUIRES")
  endif()

  # Which facts would satisfy this feature?
  set(_candidates "")
  if(ARG_REQUIRES STREQUAL "CUDA")
    if(CHRONO_CUDA_FOUND)
      list(APPEND _candidates "CUDA")
    endif()
  elseif(ARG_REQUIRES STREQUAL "HIP")
    if(CHRONO_HIP_FOUND)
      list(APPEND _candidates "HIP")
    endif()
  elseif(ARG_REQUIRES STREQUAL "CUDA_OR_HIP")
    if(CHRONO_CUDA_FOUND)
      list(APPEND _candidates "CUDA")
    endif()
    if(CHRONO_HIP_FOUND)
      list(APPEND _candidates "HIP")
    endif()
  else()
    message(FATAL_ERROR
      "chrono_select_gpu_backend(): invalid REQUIRES='${ARG_REQUIRES}'. "
      "Expected CUDA, HIP, or CUDA_OR_HIP.")
  endif()

  # A feature may accept HIP in principle but only on particular HIP platforms.
  # chrono_dem and chrono_fsi/sph are the live example: their HIP support is
  # written against the ROCm ecosystem (hipCUB/rocPRIM, rocThrust), which has no
  # working equivalent when HIP targets NVIDIA. Without this, forcing them onto
  # HIP there configures cleanly and then fails deep inside ROCm headers, which
  # tells the user nothing about what they actually did wrong.
  set(_hip_blocked FALSE)
  if("HIP" IN_LIST _candidates AND ARG_HIP_PLATFORMS)
    if(NOT "${CHRONO_HIP_PLATFORM}" IN_LIST ARG_HIP_PLATFORMS)
      list(REMOVE_ITEM _candidates "HIP")
      set(_hip_blocked TRUE)
      # Say so even though the fallback is correct and automatic. Without this,
      # a build where HIP is genuinely available -- and may even have been asked
      # for globally -- reports this feature using CUDA with no indication that
      # HIP was considered and ruled out, which is indistinguishable from the
      # preference having been ignored.
      message(STATUS
        "  ${ARG_FEATURE}: HIP is available but not usable on HIP platform "
        "'${CHRONO_HIP_PLATFORM}' (supported: ${ARG_HIP_PLATFORMS}); using another backend.")
    endif()
  endif()

  # --- Tier 1: per-feature override -----------------------------------------
  # An explicit request that cannot be satisfied is a user error, not an
  # invitation to silently substitute something else.
  if(ARG_NAME)
    set(_override_var "CHRONO_${ARG_NAME}_GPU_BACKEND")
    if(DEFINED ${_override_var} AND NOT "${${_override_var}}" STREQUAL "AUTO")
      set(_want "${${_override_var}}")
      if(NOT _want MATCHES "^(CUDA|HIP)$")
        message(FATAL_ERROR "Invalid ${_override_var}='${_want}'. Expected AUTO, CUDA, or HIP.")
      endif()
      if(NOT "${_want}" IN_LIST _candidates)
        if(_want STREQUAL "HIP" AND _hip_blocked)
          # Distinguish "HIP is not available" from "HIP is available but this
          # feature cannot use it on this platform". Reporting the first for the
          # second sends the reader off to fix a ROCm install that was never the
          # problem.
          message(FATAL_ERROR
            "${_override_var}=HIP was requested for ${ARG_FEATURE}, but its HIP support "
            "only works with CMAKE_HIP_PLATFORM in '${ARG_HIP_PLATFORMS}' "
            "(this build has '${CHRONO_HIP_PLATFORM}').\n"
            "  ${ARG_HIP_PLATFORM_NOTE}")
        endif()
        message(FATAL_ERROR
          "${_override_var}=${_want} was requested for ${ARG_FEATURE}, but ${_want} is not "
          "available in this configuration (CUDA=${CHRONO_CUDA_FOUND}, HIP=${CHRONO_HIP_FOUND}, "
          "vendor=${CHRONO_GPU_VENDOR_RESOLVED}).")
      endif()
      set(${out_var} "${_want}" PARENT_SCOPE)
      return()
    endif()
  endif()

  list(LENGTH _candidates _n)

  if(_n EQUAL 0)
    set(${out_var} "NONE" PARENT_SCOPE)
    return()
  endif()

  if(_n EQUAL 1)
    # Only one backend can satisfy this feature, so there is nothing to choose
    # and a global preference simply does not apply here. Say so when the user
    # expressed one, because this is the case where a build legitimately mixes
    # backends -- asking for HIP globally still leaves Chrono::Sensor's OptiX
    # renderer on CUDA, which is correct and worth stating rather than leaving
    # the user to wonder whether their request was dropped.
    if(NOT CHRONO_GPU_BACKEND STREQUAL "AUTO" AND NOT "${_candidates}" STREQUAL "${CHRONO_GPU_BACKEND}")
      message(STATUS
        "  ${ARG_FEATURE}: requires ${ARG_REQUIRES}, so it uses ${_candidates} rather than the "
        "preferred ${CHRONO_GPU_BACKEND}.")
    endif()
    set(${out_var} "${_candidates}" PARENT_SCOPE)
    return()
  endif()

  # --- Tier 2: global preference --------------------------------------------
  if(NOT CHRONO_GPU_BACKEND STREQUAL "AUTO")
    if(NOT CHRONO_GPU_BACKEND MATCHES "^(CUDA|HIP)$")
      message(FATAL_ERROR
        "Invalid CHRONO_GPU_BACKEND='${CHRONO_GPU_BACKEND}'. Expected AUTO, CUDA, or HIP.")
    endif()
    if("${CHRONO_GPU_BACKEND}" IN_LIST _candidates)
      set(${out_var} "${CHRONO_GPU_BACKEND}" PARENT_SCOPE)
      return()
    endif()
  endif()

  # --- Tier 3: the feature's declared preference ----------------------------
  if(ARG_PREFER AND "${ARG_PREFER}" IN_LIST _candidates)
    set(${out_var} "${ARG_PREFER}" PARENT_SCOPE)
    return()
  endif()

  list(GET _candidates 0 _first)
  set(${out_var} "${_first}" PARENT_SCOPE)
endfunction()

#-----------------------------------------------------------------------------
# chrono_gpu_feature_unavailable(FEATURE <name>
#                                [OPTION <cache var to force OFF>]
#                                [CLASS  EXPLICIT|IMPLIED])
#
# Reports a feature that cannot get the backend it needs, at a severity that
# depends on how the feature came to be enabled:
#
#   EXPLICIT  the user set option(... OFF)-defaulted switch to ON by hand.
#             Silently handing them a build missing exactly what they asked
#             for is the wrong behaviour -> WARNING, or FATAL under
#             CHRONO_STRICT_MODULE_REQUIREMENTS.
#
#   IMPLIED   enabled automatically via cmake_dependent_option(... ON).
#             The user never asked; quietly dropping it is correct -> STATUS.
#
# The module declares its own class, so no "was this explicit?" guesswork is
# needed -- the defaults already in the tree encode it.
#-----------------------------------------------------------------------------

function(chrono_gpu_feature_unavailable)
  cmake_parse_arguments(ARG "" "FEATURE;OPTION;CLASS;REQUIRES" "" ${ARGN})

  if(NOT ARG_CLASS)
    set(ARG_CLASS "EXPLICIT")
  endif()

  set(_detail
      "${ARG_FEATURE} requires a GPU backend")
  if(ARG_REQUIRES)
    set(_detail "${ARG_FEATURE} requires ${ARG_REQUIRES}")
  endif()
  set(_detail
      "${_detail}, but none is available "
      "(vendor=${CHRONO_GPU_VENDOR_RESOLVED}, CUDA=${CHRONO_CUDA_FOUND}, HIP=${CHRONO_HIP_FOUND}). "
      "Disabling ${ARG_FEATURE}.")
  string(REPLACE ";" "" _detail "${_detail}")

  if(ARG_CLASS STREQUAL "IMPLIED")
    message(STATUS "${_detail}")
  elseif(CHRONO_STRICT_MODULE_REQUIREMENTS)
    message(FATAL_ERROR
      "${_detail}\n"
      "  (CHRONO_STRICT_MODULE_REQUIREMENTS is ON, so this is fatal rather than a warning.)")
  else()
    message(WARNING "${_detail}")
  endif()

  if(ARG_OPTION)
    set(${ARG_OPTION} OFF CACHE BOOL "" FORCE)
  endif()
endfunction()

#-----------------------------------------------------------------------------
# chrono_set_gpu_source_language(<backend> <sources...>)
#
# Relabels .cu sources for the selected backend. Chrono's GPU modules are NOT
# dual-source: DEM and FSI::SPH ship a single set of .cu files that are simply
# compiled as HIP instead of CUDA, so "choosing a backend" here means choosing
# a compiler for the same sources.
#
# MUST be called before add_library(). Source-file properties are directory
# scoped and the LANGUAGE property in particular (CMP0118) is only honoured
# when set in the same directory that defines the target, so this is kept as a
# separate call rather than folded into chrono_apply_gpu_backend() below --
# which necessarily runs after the target exists.
#-----------------------------------------------------------------------------

function(chrono_set_gpu_source_language backend)
  if(NOT ARGN)
    return()
  endif()
  if(backend STREQUAL "HIP")
    set_source_files_properties(${ARGN} PROPERTIES LANGUAGE HIP)
  endif()
  # CUDA needs no relabelling: .cu is already CUDA by default, and forcing the
  # property would override any per-file choice a module made deliberately.
endfunction()

#-----------------------------------------------------------------------------
# chrono_apply_gpu_backend(<target> <backend>
#                          [DEFINE_VISIBILITY <PUBLIC|PRIVATE|INTERFACE>]
#                          [LINK_VISIBILITY   <PUBLIC|PRIVATE|INTERFACE>])
#
# Applies everything that is MECHANICALLY IDENTICAL between GPU modules:
# source language relabelling, the CHRONO_USE_* define, architectures, and the
# runtime libraries. Module-specific choices (Thrust device-system defines,
# extra INTERFACE defines) deliberately stay in the module.
#
# Before this existed, chrono_dem and chrono_fsi/sph carried
# character-identical copies of the HIP_ARCHITECTURES workaround below --
# including its comment. That workaround becomes MORE important once both
# languages can be enabled in a single project, so having one copy of it
# matters more than the line count saved.
#-----------------------------------------------------------------------------

function(chrono_apply_gpu_backend target backend)
  cmake_parse_arguments(ARG "" "DEFINE_VISIBILITY;LINK_VISIBILITY" "" ${ARGN})

  if(NOT ARG_DEFINE_VISIBILITY)
    set(ARG_DEFINE_VISIBILITY PUBLIC)
  endif()
  if(NOT ARG_LINK_VISIBILITY)
    set(ARG_LINK_VISIBILITY PUBLIC)
  endif()

  if(backend STREQUAL "CUDA")

    target_compile_definitions(${target} ${ARG_DEFINE_VISIBILITY} CHRONO_USE_CUDA)
    target_compile_options(${target} PRIVATE
                           $<$<COMPILE_LANGUAGE:CUDA>:-Wno-deprecated-gpu-targets>)

    target_link_libraries(${target} ${ARG_LINK_VISIBILITY}
                          CUDA::cudart_static
                          CUDA::nvrtc
                          CUDA::cuda_driver
                          CUDA::cublas
                          CUDA::cusparse)

    set_target_properties(${target} PROPERTIES CUDA_ARCHITECTURES "${CHRONO_CUDA_ARCHITECTURES}")

  elseif(backend STREQUAL "HIP")

    # Emit the platform macro that matches the platform HIP was actually
    # configured for, instead of hardcoding AMD. Defining both
    # __HIP_PLATFORM_AMD__ and __HIP_PLATFORM_NVIDIA__ is a hard error inside
    # HIP's own headers, so a hardcoded AMD here would make HIP-on-NVIDIA
    # unbuildable no matter what the rest of the build system decided.
    #
    # With CHRONO_ENABLE_HIP_ON_NVIDIA OFF, CHRONO_HIP_PLATFORM is always
    # "amd", so this is behaviour-identical to the previous hardcoding today.
    if(CHRONO_HIP_PLATFORM STREQUAL "nvidia")
      set(_platform_define __HIP_PLATFORM_NVIDIA__)
    else()
      set(_platform_define __HIP_PLATFORM_AMD__)
    endif()

    target_compile_definitions(${target} ${ARG_DEFINE_VISIBILITY} CHRONO_USE_HIP ${_platform_define})

    if(CHRONO_HIP_PLATFORM STREQUAL "nvidia")

      # Deliberately do NOT link hip::host here.
      #
      # ROCm's exported hip::host target carries
      # INTERFACE_COMPILE_DEFINITIONS "__HIP_PLATFORM_AMD__=1" unconditionally --
      # it is baked into the ROCm package and does not consult
      # CMAKE_HIP_PLATFORM. Linking it on the NVIDIA platform therefore puts
      # BOTH platform macros on the command line, which HIP's own headers reject
      # outright. The headers we actually need come from the ROCm include
      # directory below, and the runtime comes from CUDA.
      if(CHRONO_ROCM_ROOT AND EXISTS "${CHRONO_ROCM_ROOT}/include")
        target_include_directories(${target} ${ARG_DEFINE_VISIBILITY} "${CHRONO_ROCM_ROOT}/include")
      endif()

      # hip/nvidia_detail includes <cuda.h> and <cuda_runtime.h>. HIP translation
      # units are compiled by nvcc and find those implicitly; ordinary CXX
      # sources do not. Derive the location from the HIP compiler, which on this
      # platform IS nvcc, rather than assuming a fixed CUDA install path.
      #
      # Same visibility as the ROCm include directory above: these two travel
      # together, since any translation unit that reaches a HIP header needs the
      # CUDA headers that HIP header itself includes.
      get_filename_component(_hip_cc_bin "${CMAKE_HIP_COMPILER}" DIRECTORY)
      get_filename_component(_hip_cc_root "${_hip_cc_bin}" DIRECTORY)
      if(EXISTS "${_hip_cc_root}/include")
        target_include_directories(${target} ${ARG_DEFINE_VISIBILITY} "${_hip_cc_root}/include")
      endif()

    else()

      find_package(hip QUIET CONFIG HINTS "${CHRONO_ROCM_ROOT}")
      if(TARGET hip::host)
        target_link_libraries(${target} ${ARG_LINK_VISIBILITY} hip::host)
      endif()
      if(CHRONO_ROCM_ROOT AND EXISTS "${CHRONO_ROCM_ROOT}/include")
        target_include_directories(${target} ${ARG_DEFINE_VISIBILITY} "${CHRONO_ROCM_ROOT}/include")
      endif()

    endif()

    # Mixed CXX/HIP targets can leak HIP_ARCHITECTURES into ordinary CXX
    # compilations (for example stb_image.cpp), which makes the host compiler
    # see --offload-arch. Keep the target-level property disabled and pass the
    # GPU arch flags only to real HIP translation units.
    set_target_properties(${target} PROPERTIES HIP_ARCHITECTURES OFF)
    if(NOT CHRONO_HIP_ARCHITECTURES STREQUAL "")
      foreach(_arch IN LISTS CHRONO_HIP_ARCHITECTURES)
        if(CHRONO_HIP_PLATFORM STREQUAL "nvidia")
          # --offload-arch is a clang/hipcc spelling. On the NVIDIA platform the
          # HIP compiler is nvcc, which does not understand it and silently
          # forwards it to the host compiler, producing
          # "gcc: error: unrecognized command-line option '--offload-arch=...'".
          # Translate to a CUDA gencode spec instead.
          if(_arch STREQUAL "native")
            # "native" is a keyword, not an architecture, so it must not be
            # pasted into a gencode spec -- that yields compute_native and
            # "nvcc fatal: Unsupported gpu architecture 'compute_native'".
            # nvcc spells the same request -arch=native (CUDA >= 11.5), which
            # is also what CHRONO_HIP_ARCHITECTURES defaults to, so this is the
            # path taken whenever the user does not set an architecture.
            #
            # Cross-target builds never reach here: chrono_guard_cross_target_arch()
            # rejects "native" at configure time, since there is no local GPU to
            # detect.
            target_compile_options(${target} PRIVATE
              $<$<COMPILE_LANGUAGE:HIP>:-arch=native>)
          else()
            string(REGEX REPLACE "^sm_" "" _cc "${_arch}")
            target_compile_options(${target} PRIVATE
              $<$<COMPILE_LANGUAGE:HIP>:--generate-code=arch=compute_${_cc},code=sm_${_cc}>)
          endif()
        else()
          target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:HIP>:--offload-arch=${_arch}>)
          target_link_options(${target} PRIVATE $<$<LINK_LANGUAGE:HIP>:--offload-arch=${_arch}>)
        endif()
      endforeach()
    endif()

  elseif(NOT backend STREQUAL "NONE")

    message(FATAL_ERROR "chrono_apply_gpu_backend(): unknown backend '${backend}'")

  endif()
endfunction()
