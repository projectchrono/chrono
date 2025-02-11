#===============================================================================
# CMake configuration file for the Chrono::GPU library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================


option(ENABLE_MODULE_GPU "Enable the Chrono::GPU module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_GPU)
  return()
endif()

message(STATUS "\n==== Chrono GPU module ====\n")

# Return now if CUDA is not available
if(NOT CHRONO_CUDA_FOUND)
    message("Chrono::GPU requires CUDA, but CUDA was not found; disabling Chrono::GPU")
    set(ENABLE_MODULE_GPU OFF CACHE BOOL "Enable the Chrono::GPU module" FORCE)
    return()
endif()

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message("Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::GPU")
    set(ENABLE_MODULE_GPU OFF CACHE BOOL "Enable the Chrono::GPU module" FORCE)
    return()
endif()

message(STATUS "Chrono CUDA architectures: ${CHRONO_CUDA_ARCHITECTURES}")

# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigGpu.h.in
    ${PROJECT_BINARY_DIR}/chrono_gpu/ChConfigGpu.h
    )

install(FILES "${PROJECT_BINARY_DIR}/chrono_gpu/ChConfigGpu.h"
        DESTINATION include/chrono_gpu)

# ------------------------------------------------------------------------------
# List the files in the Chrono::GPU module
# ------------------------------------------------------------------------------

set(GPU_BASE_FILES
    ChApiGpu.h
    ChGpuDefines.h
    )
source_group("" FILES ${GPU_BASE_FILES})

set(GPU_PHYSICS_FILES
    physics/ChSystemGpu.h
    physics/ChSystemGpu.cpp
    physics/ChSystemGpu_impl.h
    physics/ChSystemGpu_impl.cpp
    physics/ChSystemGpuMesh_impl.h
    physics/ChSystemGpuMesh_impl.cpp
    physics/ChGpuBoundaryConditions.h
    )
source_group(physics FILES ${GPU_PHYSICS_FILES})

set(GPU_CUDA_FILES
    cuda/ChGpu_SMC.cu
    cuda/ChGpu_SMC.cuh
    cuda/ChGpu_SMC_trimesh.cu
    cuda/ChGpu_SMC_trimesh.cuh
    cuda/ChGpuCollision.cuh
    cuda/ChGpuBoundaryConditions.cuh
    cuda/ChGpuHelpers.cuh
    cuda/ChGpuBoxTriangle.cuh
    cuda/ChGpuCUDAalloc.hpp
    cuda/ChCudaMathUtils.cuh
    )
source_group(cuda FILES ${GPU_CUDA_FILES})

set(GPU_UTILITY_FILES
    utils/ChGpuUtilities.h
    utils/ChGpuJsonParser.h
    utils/ChGpuSphereDecomp.h
    )
source_group(utilities FILES ${GPU_UTILITY_FILES})

set(GPU_VIS_FILES
    visualization/ChGpuVisualization.h
    visualization/ChGpuVisualization.cpp
)
source_group(visualization FILES ${GPU_VIS_FILES})

set(GPU_GL_FILES
    visualization/ChGpuVisualizationGL.h
    visualization/ChGpuVisualizationGL.cpp
   )
source_group("" FILES ${GPU_GL_FILES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_gpu library
# ------------------------------------------------------------------------------

message(STATUS "Add ChronoEngine_gpu library")

add_library(ChronoEngine_gpu
            ${GPU_BASE_FILES}
            ${GPU_PHYSICS_FILES}
            ${GPU_CUDA_FILES}
            ${ChronoEngine_GPU_UTILITIES}
            ${GPU_VIS_FILES})

add_library(ChronoEngine::gpu ALIAS ChronoEngine_gpu)

target_compile_definitions(ChronoEngine_gpu PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_GPU>)

target_link_libraries(ChronoEngine_gpu PRIVATE ChronoEngine)

target_link_libraries(ChronoEngine_gpu PUBLIC CUDA::cudart_static)
target_link_libraries(ChronoEngine_gpu PUBLIC CUDA::nvrtc)
target_link_libraries(ChronoEngine_gpu PUBLIC CUDA::cuda_driver)
target_link_libraries(ChronoEngine_gpu PUBLIC CUDA::cublas)
target_link_libraries(ChronoEngine_gpu PUBLIC CUDA::cusparse)

set_target_properties(ChronoEngine_gpu PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})

install(TARGETS ChronoEngine_gpu
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_gpu
)

# ------------------------------------------------------------------------------
# Add the ChronoEngine_gpu_opengl library
# ------------------------------------------------------------------------------

if(ENABLE_MODULE_OPENGL)

    message(STATUS "Add ChronoEngine_gpu_opengl library")

    add_library(ChronoEngine_gpu_opengl ${GPU_GL_FILES})
    add_library(ChronoEngine::gpu_opengl ALIAS ChronoEngine_gpu_opengl)

    target_compile_definitions(ChronoEngine_gpu_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_GPU>)

    target_link_libraries(ChronoEngine_gpu_opengl PRIVATE ChronoEngine ChronoEngine_gpu)
    target_link_libraries(ChronoEngine_gpu_opengl PUBLIC ChronoEngine_opengl)

    target_link_libraries(ChronoEngine_gpu_opengl PRIVATE CUDA::cudart_static)
    target_link_libraries(ChronoEngine_gpu_opengl PRIVATE CUDA::nvrtc)
    target_link_libraries(ChronoEngine_gpu_opengl PRIVATE CUDA::cuda_driver)
    target_link_libraries(ChronoEngine_gpu_opengl PRIVATE CUDA::cublas)
    target_link_libraries(ChronoEngine_gpu_opengl PRIVATE CUDA::cusparse)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        target_compile_options(ChronoEngine_gpu_opengl PUBLIC $<$<COMPILE_LANGUAGE:CXX>:"-framework IOKit -framework Cocoa -framework OpenGL">)
    endif()

        
    install(TARGETS ChronoEngine_gpu_opengl
            EXPORT ChronoEngineTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_gpu)

endif()

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_gpu
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")
