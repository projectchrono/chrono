#=============================================================================
# CMake configuration file for the Chrono::SPH FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

cmake_dependent_option(CH_ENABLE_MODULE_FSI_SPH "Enable the Chrono::SPH FSI module" ON
                       "CH_ENABLE_MODULE_FSI" ON)
                 
if(NOT CH_ENABLE_MODULE_FSI_SPH)
  mark_as_advanced(FORCE CH_USE_SPH_DOUBLE)
  return()
else()
  mark_as_advanced(CLEAR CH_USE_SPH_DOUBLE)
endif()

message(STATUS "SPH-based Chrono::FSI submodule")

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI")
    set(CH_ENABLE_MODULE_FSI_SPH OFF CACHE BOOL "Enable the Chrono::SPH FSI module" FORCE)
    return()
endif()

# Select the GPU backend for this submodule.
#
# CLASS IMPLIED below: unlike the other GPU modules, this one is enabled
# automatically via cmake_dependent_option(... ON) whenever Chrono::FSI is on.
# The user never explicitly asked for it, so dropping it quietly when no GPU
# backend exists is the correct behavior rather than a warning they cannot act
# on.
# HIP_PLATFORMS AMD NVIDIA: Thrust's device system is selected from the HIP
# platform (rocThrust on AMD, CUDA's own Thrust when HIP targets NVIDIA), so
# neither platform is excluded. The NVIDIA platform was verified to build; the
# AMD platform is the preexisting, unchanged path.
chrono_select_gpu_backend(CHRONO_FSISPH_BACKEND
                          FEATURE  "Chrono::FSI::SPH"
                          NAME     FSI_SPH
                          REQUIRES CUDA_OR_HIP
                          PREFER   CUDA
                          HIP_PLATFORMS amd nvidia)

if(CHRONO_FSISPH_BACKEND STREQUAL "NONE")
    chrono_gpu_feature_unavailable(FEATURE  "Chrono::FSI::SPH"
                                   REQUIRES "CUDA or HIP"
                                   OPTION   CH_ENABLE_MODULE_FSI_SPH
                                   CLASS    IMPLIED)
    return()
endif()

# Thrust's device system follows the HIP PLATFORM, not merely the fact that the
# backend is HIP. THRUST_DEVICE_SYSTEM_HIP means rocThrust, which exists only in
# the ROCm stack; when HIP targets NVIDIA the translation units are compiled by
# nvcc and the correct device system is CUDA. Selecting HIP there sends the CUDA
# toolkit's own CCCL looking for a system it does not ship.
#
# Computed once here because every target that compiles SPH device code must
# agree on it -- Chrono_fsisph and the VSG plugin both do, and having the VSG
# plugin decide separately is exactly how it ended up out of step.
if(CHRONO_HIP_PLATFORM STREQUAL "nvidia")
  set(CHRONO_FSISPH_THRUST_DEVICE THRUST_DEVICE_SYSTEM_CUDA)
else()
  set(CHRONO_FSISPH_THRUST_DEVICE THRUST_DEVICE_SYSTEM_HIP)
endif()

message(STATUS "  Chrono::FSI::SPH GPU backend: ${CHRONO_FSISPH_BACKEND}")
if(CHRONO_FSISPH_BACKEND STREQUAL "HIP")
    message(STATUS "  Chrono HIP architectures: ${CHRONO_HIP_ARCHITECTURES}")
else()
    message(STATUS "  Chrono CUDA architectures: ${CHRONO_CUDA_ARCHITECTURES}")
endif()

#-----------------------------------------------------------------------------

# Additional options for this module
option(CH_USE_SPH_DOUBLE "Compile Chrono::SPH with double precision math" OFF)
if(CH_USE_SPH_DOUBLE)
  set(CHRONO_SPH_USE_DOUBLE "#define CHRONO_SPH_USE_DOUBLE")
endif()

message(STATUS "  Use double precision? ${CH_USE_SPH_DOUBLE}")

#-----------------------------------------------------------------------------

find_package(Splashsurf QUIET)
if(SPLASHSURF_FOUND)
    message(STATUS "  Splashsurf executable: ${SPLASHSURF_EXECUTABLE}")
    message(STATUS "  Splashsurf version:    ${SPLASHSURF_VERSION_STRING}")

    # Prepare replacement variables
    set(CHRONO_HAS_SPLASHSURF "#define CHRONO_HAS_SPLASHSURF")
    set(CHRONO_SPLASHSURF_EXECUTABLE "#define SPLASHSURF_EXECUTABLE \"${SPLASHSURF_EXECUTABLE}\"")
else()
    message(STATUS "  Splashsurf NOT found. Surface reconstruction utility disabled.")

    set(CHRONO_HAS_SPLASHSURF "#undef CHRONO_HAS_SPLASHSURF")
    set(CHRONO_SPLASHSURF_EXECUTABLE "#undef SPLASHSURF_EXECUTABLE")
endif()

#-----------------------------------------------------------------------------
# List all files for the SPH-based Chrono::FSI library
#-----------------------------------------------------------------------------

# --------------- SPH FSI FILES

set(FSISPH_BASE_FILES
    ChFsiDataTypesSPH.h
    ChFsiDefinitionsSPH.h
    ChFsiParamsSPH.h
    ChFsiProblemSPH.h
    ChFsiSystemSPH.h
    ChFsiInterfaceSPH.h
    ChFsiFluidSystemSPH.h
    ChFsiPrintUtilsSPH.h
    ChFsiSplashsurfSPH.h
    #
    ChFsiProblemSPH.cpp
    ChFsiSystemSPH.cpp
    ChFsiInterfaceSPH.cpp
    ChFsiFluidSystemSPH.cpp
    ChFsiSplashsurfSPH.cpp
)
source_group("" FILES ${FSISPH_BASE_FILES})

set(FSISPH_PHYSICS_FILES
    physics/SphBceManager.cuh
    physics/SphCollisionSystem.cuh
    physics/SphDataManager.cuh
    physics/SphFluidDynamics.cuh
    physics/SphForce.cuh    
    physics/SphForceWCSPH.cuh
    physics/SphForceISPH.cuh
    physics/SphFsiInterface.cuh
    physics/SphGeneral.cuh
    physics/SphMarkerType.cuh
    physics/SphParticleRelocator.cuh
    #
    physics/SphBceManager.cu
    physics/SphCollisionSystem.cu
    physics/SphDataManager.cu
    physics/SphFluidDynamics.cu
    physics/SphForce.cu
    physics/SphForceWCSPH.cu
    physics/SphForceISPH.cu
    physics/SphFsiInterface.cu
    physics/SphGeneral.cu
    physics/SphParticleRelocator.cu
)
source_group("physics" FILES ${FSISPH_PHYSICS_FILES})

set(FSISPH_MATH_FILES
    math/SphLinearSolver.h
    #math/SphLinearSolverBiCGStab.h
    #math/SphLinearSolverGMRES.h
    #
    math/SphCustomMath.cuh
    math/SphExactLinearSolvers.cuh
    #
    #math/SphLinearSolverBiCGStab.cpp
    #math/SphLinearSolverGMRES.cpp
)
source_group("math" FILES ${FSISPH_MATH_FILES})

set(FSISPH_UTILS_FILES
    utils/SphUtilsDevice.cuh
    utils/SphUtilsDevice.cu
    utils/SphUtilsTypeConvert.cuh
    utils/SphUtilsPrint.cuh
    utils/SphUtilsPrint.cu
    utils/SphUtilsLogging.cuh
)
source_group("utils" FILES ${FSISPH_UTILS_FILES})

set(FSISPH_GPU_FILES
    ../../chrono/gpu/ChGpuRuntime.h
    ../../chrono/gpu/ChGpuPrimitives.h
)
source_group("gpu" FILES ${FSISPH_GPU_FILES})

# ------------------------------------------

set(FSISPH_GPU_SOURCE_FILES
    ChFsiProblemSPH.cpp
    ChFsiSystemSPH.cpp
    ChFsiInterfaceSPH.cpp
    ChFsiFluidSystemSPH.cpp
    ChFsiSplashsurfSPH.cpp
    physics/SphBceManager.cu
    physics/SphCollisionSystem.cu
    physics/SphDataManager.cu
    physics/SphFluidDynamics.cu
    physics/SphForce.cu
    physics/SphForceWCSPH.cu
    physics/SphForceISPH.cu
    physics/SphFsiInterface.cu
    physics/SphGeneral.cu
    physics/SphParticleRelocator.cu
    utils/SphUtilsDevice.cu
    utils/SphUtilsPrint.cu
)
chrono_set_gpu_source_language(${CHRONO_FSISPH_BACKEND} ${FSISPH_GPU_SOURCE_FILES})

# --------------- 3rd party STB FILES

set(FSISPH_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("stb" FILES ${FSISPH_STB_FILES})

# Disable all warnings from external code (see cmake/ChronoWarnings.cmake)
ch_disable_warnings_on_sources(${FSISPH_STB_FILES})

# --------------- SPH FSI VSG INTERFACE

if(CH_ENABLE_MODULE_VSG)
    set(FSISPH_VSG_FILES
        visualization/ChSphVisualizationVSG.h
        visualization/ChSphVisualizationVSG.cpp
    )
else()
    set(FSISPH_VSG_FILES "")
endif()
source_group("" FILES ${FSISPH_VSG_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_FSI_SPH "FSI")
if(CH_ENABLE_MODULE_VSG)
  list (APPEND DEPENDENCIES_FSI_SPH "VSG")
endif()

set(DEPENDENCIES_FSI_SPH ${DEPENDENCIES_FSI_SPH} PARENT_SCOPE)

# Export the resolved backend for ChronoConfig.cmake (see cmake/ChronoConfig.cmake.in).
set(CHRONO_FSISPH_BACKEND ${CHRONO_FSISPH_BACKEND} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Generate and install configuration file
#-----------------------------------------------------------------------------

set(FSISPH_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono_fsi/sph/ChFsiConfigSPH.h)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChFsiConfigSPH.h.in ${FSISPH_CONFIG_FILE} @ONLY)

source_group("" FILES ${FSISPH_CONFIG_FILE})

install(FILES ${FSISPH_CONFIG_FILE} DESTINATION include/chrono_fsi/sph)

#-----------------------------------------------------------------------------
# Create the Chrono_fsisph library
#-----------------------------------------------------------------------------

message(STATUS "  Add Chrono_fsisph library")

add_library(Chrono_fsisph
    ${FSISPH_BASE_FILES}
    ${FSISPH_PHYSICS_FILES}
    ${FSISPH_MATH_FILES}
    ${FSISPH_UTILS_FILES}
    ${FSISPH_GPU_FILES}
    ${FSISPH_STB_FILES}
    ${FSISPH_CONFIG_FILE}
)
add_library(Chrono::fsisph ALIAS Chrono_fsisph)

set_target_properties(Chrono_fsisph PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if (CH_STATIC)
  set_target_properties(Chrono_fsisph PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)               # missing DLL interface
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4100>)               # unreferenced formal parameter
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4251>)   # missing DLL interface (forward to host compiler for .cu files)
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4100>)   # unreferenced formal parameter (forward to host compiler for .cu files)
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4324>)   # structure padded due to alignment specifier
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--diag-suppress=1394>) # missing DLL interface (matches /wd4251 for CXX)
  set_target_properties(Chrono_fsisph PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)  # Eigen host constexpr helpers called from __host__ __device__ functions

target_link_libraries(Chrono_fsisph PRIVATE Chrono_core)
target_link_libraries(Chrono_fsisph PUBLIC Chrono_fsi)

target_compile_definitions(Chrono_fsisph PRIVATE CH_API_COMPILE_FSI)

# Visibility of the GPU runtime is backend-dependent for this target, and the
# two cases are not symmetric:
#
#   CUDA  PUBLIC. Public headers switch on CHRONO_USE_CUDA and then include
#         <cuda_runtime.h>, so any consumer that inherits the definition must
#         also inherit the CUDA include directories that come with
#         CUDA::cudart_static. Making the definition public but the link
#         private breaks every downstream target (Chrono_parsers first).
#
#   HIP   PRIVATE. Public consumers are ordinary C++ translation units; the HIP
#         target's compile definitions would force Thrust onto the rocPRIM
#         backend in code that never asked for it.
if(CHRONO_FSISPH_BACKEND STREQUAL "CUDA")
  set(_fsisph_gpu_visibility PUBLIC)
else()
  set(_fsisph_gpu_visibility PRIVATE)
endif()

chrono_apply_gpu_backend(Chrono_fsisph ${CHRONO_FSISPH_BACKEND}
                         DEFINE_VISIBILITY ${_fsisph_gpu_visibility}
                         LINK_VISIBILITY   ${_fsisph_gpu_visibility})

# Module-specific Thrust wiring, which genuinely differs between the two
# backends and so deliberately stays here rather than moving into the helper.
if(CHRONO_FSISPH_BACKEND STREQUAL "CUDA")

  target_link_libraries(Chrono_fsisph PUBLIC Thrust::Thrust)
  target_compile_definitions(Chrono_fsisph PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
  target_compile_definitions(Chrono_fsisph PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

elseif(CHRONO_FSISPH_BACKEND STREQUAL "HIP")

  target_compile_definitions(Chrono_fsisph INTERFACE CHRONO_FSI_SPH_USE_HIP_RUNTIME)

  # Keep the device Thrust backend local to this target, and export NOTHING.
  #
  # No installed Chrono::FSI::SPH header includes <thrust/...>; only the private
  # .cuh sources do. So a consumer never needs a Thrust configuration from us,
  # and any value we exported would be wrong for someone:
  #
  #   THRUST_DEVICE_SYSTEM_HIP  names rocThrust, which a consumer compiled by a
  #                             non-HIP compiler cannot use at all;
  #   THRUST_DEVICE_SYSTEM_CPP  silently demotes a consumer's OWN device Thrust
  #                             to host code. Chrono::Sensor links us PUBLIC and
  #                             calls thrust::device_vector on CUDA device
  #                             pointers; under _CPP that becomes a host memcpy
  #                             from device memory, which segfaults at runtime
  #                             rather than failing to build.
  #
  # Exporting nothing leaves each consumer on Thrust's own default, which is
  # what it gets when Chrono::FSI::SPH is not in the link at all.
  target_compile_definitions(Chrono_fsisph PRIVATE "THRUST_DEVICE_SYSTEM=${CHRONO_FSISPH_THRUST_DEVICE}")
  target_compile_definitions(Chrono_fsisph PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

endif()

install(TARGETS Chrono_fsisph
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_fsi)

#-------------------------------------------------------------------------------
# Add the Chrono_fsisph_vsg library
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)
    message(STATUS "  Add Chrono_fsisph_vsg library")

    add_library(Chrono_fsisph_vsg
        ${FSISPH_VSG_FILES}
    )
    add_library(Chrono::fsisph_vsg ALIAS Chrono_fsisph_vsg)

    set_target_properties(Chrono_fsisph_vsg PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

   
    if (CH_STATIC)
      set_target_properties(Chrono_fsisph_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()
 
    if(MSVC)
      # disable warnings related to missing DLL interface
      target_compile_options(Chrono_fsisph_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
      set_target_properties(Chrono_fsisph_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_compile_definitions(Chrono_fsisph_vsg PRIVATE CH_API_COMPILE_FSI)
    
    target_link_libraries(Chrono_fsisph_vsg PRIVATE Chrono_core Chrono_fsisph)
    target_link_libraries(Chrono_fsisph_vsg PUBLIC Chrono_vsg)

    chrono_apply_gpu_backend(Chrono_fsisph_vsg ${CHRONO_FSISPH_BACKEND}
                             DEFINE_VISIBILITY PRIVATE
                             LINK_VISIBILITY   PRIVATE)

    if(CHRONO_FSISPH_BACKEND STREQUAL "CUDA")
      target_link_libraries(Chrono_fsisph_vsg PRIVATE Thrust::Thrust)
      target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
      target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")
    elseif(CHRONO_FSISPH_BACKEND STREQUAL "HIP")
      # Compile the VSG plugin source with the HIP compiler: it includes FSI
      # data-manager headers that reference rocThrust/rocPRIM device types,
      # which do not compile as plain C++ (same treatment as FSISPH_GPU_SOURCE_FILES).
      set_source_files_properties(visualization/ChSphVisualizationVSG.cpp PROPERTIES LANGUAGE HIP)
      target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_DEVICE_SYSTEM=${CHRONO_FSISPH_THRUST_DEVICE}")
      target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")
    endif()

    install(TARGETS Chrono_fsisph_vsg
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_fsi)

endif()
