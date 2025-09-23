#===============================================================================
# CMake configuration file for the Chrono::DEM library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================


option(CH_ENABLE_MODULE_DEM "Enable the Chrono::DEM module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_DEM)
  return()
endif()

message(STATUS "\n==== Chrono DEM module ====\n")

# Return now if CUDA is not available
if(NOT CHRONO_CUDA_FOUND)
    message("Chrono::DEM requires CUDA, but CUDA was not found; disabling Chrono::DEM")
    set(CH_ENABLE_MODULE_DEM OFF CACHE BOOL "Enable the Chrono::DEM module" FORCE)
    return()
endif()

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message("Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::DEM")
    set(CH_ENABLE_MODULE_DEM OFF CACHE BOOL "Enable the Chrono::DEM module" FORCE)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigDem.h.in
    ${PROJECT_BINARY_DIR}/chrono_dem/ChConfigDem.h
    )

install(FILES "${PROJECT_BINARY_DIR}/chrono_dem/ChConfigDem.h"
        DESTINATION include/chrono_dem)

# ------------------------------------------------------------------------------
# List the files in the Chrono::DEM module
# ------------------------------------------------------------------------------

set(DEM_BASE_FILES
    ChApiDem.h
    ChDemDefines.h
    )
source_group("" FILES ${DEM_BASE_FILES})

set(DEM_PHYSICS_FILES
    physics/ChSystemDem.h
    physics/ChSystemDem.cpp
    physics/ChSystemDem_impl.h
    physics/ChSystemDem_impl.cpp
    physics/ChSystemDemMesh_impl.h
    physics/ChSystemDemMesh_impl.cpp
    physics/ChDemBoundaryConditions.h
    )
source_group(physics FILES ${DEM_PHYSICS_FILES})

set(DEM_CUDA_FILES
    cuda/ChDem_SMC.cu
    cuda/ChDem_SMC.cuh
    cuda/ChDem_SMC_trimesh.cu
    cuda/ChDem_SMC_trimesh.cuh
    cuda/ChDemCollision.cuh
    cuda/ChDemBoundaryConditions.cuh
    cuda/ChDemHelpers.cuh
    cuda/ChDemBoxTriangle.cuh
    cuda/ChDemCUDAalloc.hpp
    cuda/ChCudaMathUtils.cuh
    )
source_group(cuda FILES ${DEM_CUDA_FILES})

set(DEM_UTILITY_FILES
    utils/ChDemUtilities.h
    utils/ChDemJsonParser.h
    utils/ChDemSphereDecomp.h
    )
source_group(utilities FILES ${DEM_UTILITY_FILES})

# --------------- DEM VSG INTERFACE

if(CH_ENABLE_MODULE_VSG)
    set(DEM_VSG_FILES
        visualization/ChDemVisualizationVSG.h
        visualization/ChDemVisualizationVSG.cpp
    )
else()
    set(DEM_VSG_FILES "")
endif()
source_group("" FILES ${DEM_VSG_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_DEM "")
if(CH_ENABLE_MODULE_VSG)
  list (APPEND DEPENDENCIES_DEM "VSG")
endif()

set(DEPENDENCIES_DEM ${DEPENDENCIES_DEM} PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add the Chrono_dem library
# ------------------------------------------------------------------------------

message(STATUS "Add Chrono_dem library")

add_library(Chrono_dem
            ${DEM_BASE_FILES}
            ${DEM_PHYSICS_FILES}
            ${DEM_CUDA_FILES}
            ${Chrono_DEM_UTILITIES})

add_library(Chrono::dem ALIAS Chrono_dem)

set_target_properties(Chrono_dem PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if (CH_STATIC)
  set_target_properties(Chrono_dem PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_dem PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_dem PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_DEM>)
if(MSVC)
  target_compile_options(Chrono_dem PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4996>)  # deprecated function or class member
  target_compile_options(Chrono_dem PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4996>) # CUDA Deprecation: The type will be removed in the next major release
  target_compile_options(Chrono_dem PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4100>) # CUDA unreferenced formal parameter
endif()

target_link_libraries(Chrono_dem PRIVATE Chrono_core)

target_link_libraries(Chrono_dem PUBLIC CUDA::cudart_static)
target_link_libraries(Chrono_dem PUBLIC CUDA::nvrtc)
target_link_libraries(Chrono_dem PUBLIC CUDA::cuda_driver)
target_link_libraries(Chrono_dem PUBLIC CUDA::cublas)
target_link_libraries(Chrono_dem PUBLIC CUDA::cusparse)

set_target_properties(Chrono_dem PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})

install(TARGETS Chrono_dem
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_dem
)

#-------------------------------------------------------------------------------
# Add the Chrono_dem_vsg library
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)
    message(STATUS "  Add Chrono_dem_vsg library")

    add_library(Chrono_dem_vsg
        ${DEM_VSG_FILES}
    )
    add_library(Chrono::dem_vsg ALIAS Chrono_dem_vsg)
   
    if (CH_STATIC)
      set_target_properties(Chrono_dem_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()
 
    if(MSVC)
      # disable warnings related to missing DLL interface
      target_compile_options(Chrono_dem_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
      set_target_properties(Chrono_dem_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_compile_definitions(Chrono_dem_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_DEM>)
    
    target_link_libraries(Chrono_dem_vsg PRIVATE Chrono_core Chrono_dem)
    target_link_libraries(Chrono_dem_vsg PUBLIC Chrono_vsg)

    target_link_libraries(Chrono_dem_vsg PRIVATE CUDA::cudart_static)
    target_link_libraries(Chrono_dem_vsg PRIVATE CUDA::nvrtc)
    target_link_libraries(Chrono_dem_vsg PRIVATE CUDA::cuda_driver)
    target_link_libraries(Chrono_dem_vsg PRIVATE CUDA::cublas)
    target_link_libraries(Chrono_dem_vsg PRIVATE CUDA::cusparse)

    install(TARGETS Chrono_dem_vsg
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_dem)

endif()

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_dem
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")
