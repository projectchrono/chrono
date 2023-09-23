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

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message("Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::GPU")
    set(ENABLE_MODULE_GPU OFF CACHE BOOL "Enable the Chrono::GPU module" FORCE)
    return()
endif()

# Return now if CUDA is not available
if(NOT CUDA_FOUND)
    message("Chrono::GPU requires CUDA, but CUDA was not found; disabling Chrono::GPU")
    set(ENABLE_MODULE_GPU OFF CACHE BOOL "Enable the Chrono::GPU module" FORCE)
    return()
endif()


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
# Collect all additional include directories necessary for the GPU module
# ------------------------------------------------------------------------------

include_directories(${CUDA_INCLUDE_DIRS})

set(CH_GPU_INCLUDES ${CUDA_INCLUDE_DIRS})
set(CH_GPU_CXX_FLAGS "")
set(CH_GPU_C_FLAGS "")
set(CH_CPU_COMPILE_DEFS "")
set(CH_GPU_LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")
set(CH_GPU_LINKED_LIBRARIES ChronoEngine ${CUDA_FRAMEWORK})

# ------------------------------------------------------------------------------
# Add optional run-time visualization support
# ------------------------------------------------------------------------------

if(ENABLE_MODULE_OPENGL)
  include_directories(${CH_OPENGL_INCLUDES})
  set(CH_GPU_INCLUDES ${CH_GPU_INCLUDES} ${CH_OPENGL_INCLUDES})
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(CH_GPU_LINKER_FLAGS  "${CH_GPU_LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
  list(APPEND CH_GPU_LINKED_LIBRARIES ChronoEngine_opengl)
endif()

# ------------------------------------------------------------------------------
# Add optional HDF5 support
# ------------------------------------------------------------------------------

#if(HDF5_FOUND)
#    include_directories(${HDF5_INCLUDE_DIRS})
#    set(CH_GPU_INCLUDES ${CH_GPU_INCLUDES} ${HDF5_INCLUDE_DIRS})
##    set(CH_CPU_COMPILE_DEFS "${CH_CPU_COMPILE_DEFS} ${HDF5_COMPILE_DEFS} ${H5_BUILT_AS_DYNAMIC_LIB}")
#    set(CH_CPU_COMPILE_DEFS "${CH_CPU_COMPILE_DEFS} ${H5_BUILT_AS_DYNAMIC_LIB}")
#    list(APPEND CH_GPU_LINKED_LIBRARIES ${HDF5_CXX_LIBRARIES})
#    add_definitions(-DUSE_HDF5)
#endif()

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

set(CH_GPU_CXX_FLAGS "${CH_GPU_CXX_FLAGS}" PARENT_SCOPE)
set(CH_GPU_C_FLAGS "${CH_GPU_C_FLAGS}" PARENT_SCOPE)
set(CH_GPU_INCLUDES "${CH_GPU_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List the files in the Chrono::GPU module
# ------------------------------------------------------------------------------

set(ChronoEngine_GPU_BASE
    ChApiGpu.h
    ChGpuDefines.h
    )

source_group("" FILES ${ChronoEngine_GPU_BASE})

set(ChronoEngine_GPU_PHYSICS
    physics/ChSystemGpu.h
    physics/ChSystemGpu.cpp
    physics/ChSystemGpu_impl.h
    physics/ChSystemGpu_impl.cpp
    physics/ChSystemGpuMesh_impl.h
    physics/ChSystemGpuMesh_impl.cpp
    physics/ChGpuBoundaryConditions.h
    )

source_group(physics FILES ${ChronoEngine_GPU_PHYSICS})

set(ChronoEngine_GPU_CUDA
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

source_group(cuda FILES ${ChronoEngine_GPU_CUDA})

set(ChronoEngine_GPU_UTILITIES
    utils/ChGpuUtilities.h
    utils/ChGpuJsonParser.h
    utils/ChGpuSphereDecomp.h
    )

source_group(utilities FILES ${ChronoEngine_GPU_UTILITIES})

set(ChronoEngine_GPU_VISUALIZATION
    utils/ChGpuVisualization.h
    utils/ChGpuVisualization.cpp
   )
source_group(utilities FILES ${ChronoEngine_GPU_VISUALIZATION})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_gpu library
# ------------------------------------------------------------------------------

CUDA_ADD_LIBRARY(ChronoEngine_gpu SHARED
                 ${ChronoEngine_GPU_BASE}
                 ${ChronoEngine_GPU_PHYSICS}
                 ${ChronoEngine_GPU_CUDA}
                 ${ChronoEngine_GPU_UTILITIES}
                 ${ChronoEngine_GPU_VISUALIZATION}
                 )

set_target_properties(ChronoEngine_gpu PROPERTIES
                      LINK_FLAGS "${CH_GPU_LINKER_FLAGS}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_GPU ${CH_CPU_COMPILE_DEFS}")

#if(HDF5_FOUND)
#target_compile_definitions(ChronoEngine_gpu PRIVATE ${HDF5_COMPILE_DEFS})
#endif()

target_link_libraries(ChronoEngine_gpu ${CH_GPU_LINKED_LIBRARIES})
target_include_directories(ChronoEngine_gpu PUBLIC "${CUB_INCLUDE_DIR}/../")

install(TARGETS ChronoEngine_gpu
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_gpu
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp")

mark_as_advanced(FORCE
                 CUDA_BUILD_CUBIN
                 CUDA_BUILD_EMULATION
                 CUDA_SEPARABLE_COMPILATION
                 CUDA_SDK_ROOT_DIR
                 CUDA_VERBOSE_BUILD
                 CUDA_HOST_COMPILER)

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----

option(GPU_VERBOSE_PTXAS "Enable verbose output from ptxas during compilation" OFF)
mark_as_advanced(GPU_VERBOSE_PTXAS)

if(GPU_VERBOSE_PTXAS)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --ptxas-options=-v")
endif()

#message(STATUS "NVCC Flags: ${CUDA_NVCC_FLAGS}")

set(CUDA_SEPARABLE_COMPILATION OFF)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

    set(CUDA_SEPARABLE_COMPILATION ON)
    target_compile_options(ChronoEngine_gpu PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:
                           --generate-line-info
                           --device-debug
                           >)

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

    set(CUDA_NVCC_FLAGS_DEBUG "${CUDA_NVCC_FLAGS_DEBUG} -G -Xcompiler -g")
    set(CUDA_NVCC_FLAGS_RELEASE "${CUDA_NVCC_FLAGS_RELEASE} -Xcompiler -O3 -Xptxas -O3 -Xcompiler -DNDEBUG")
    set(CUDA_NVCC_FLAGS_RELWITHDEBINFO "${CUDA_NVCC_FLAGS_RELWITHDEBINFO} -Xcompiler -O2 -Xcompiler -g -Xcompiler -DNDEBUG")
    set(CUDA_NVCC_FLAGS_MINSIZEREL "${CUDA_NVCC_FLAGS_MINSIZEREL} -Xcompiler -Os -Xcompiler -DNDEBUG")
    message(STATUS "NVCC Release flags are" ${CUDA_NVCC_FLAGS_RELEASE})
       
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
        if(${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER 4.9)
            set(CUDA_PROPAGATE_HOST_FLAGS OFF)
        else()
            message(FATAL_ERROR "Using GCC version ${CMAKE_CXX_COMPILER_VERSION}. GCC <= 4.9 is not supported! Please use a newer compiler")
        endif()
    endif()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC --compiler-options -Wall -lineinfo)

elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    message(FATAL_ERROR "macOS is not supported!")

endif()

message(STATUS "NVCC Flags: ${CUDA_NVCC_FLAGS}")

# Install required chrono_thirdparty headers
if(DEFINED ${CUB_INCLUDE_DIR})
   install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/cub
           DESTINATION include/chrono_thirdparty
           FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
endif()
