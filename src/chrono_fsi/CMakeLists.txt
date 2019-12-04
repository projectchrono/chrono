#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

IF(NOT ENABLE_MODULE_FSI)
  #mark_as_advanced(FORCE USE_FSI_DOUBLE)
  return()
ENDIF()

message(STATUS "==== Chrono FSI module ====")



if(EIGEN3_VERSION VERSION_LESS "3.3.6")
  message("Chrono::FSI requires Eigen version 3.3.6 or better")
  message("Current Eigen version: ${EIGEN3_VERSION}")
  message("Chrono::FSI disabled")
  set(ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
  return()
endif()

# Return now if CUDA is not available
if(NOT CUDA_FOUND)
  message("Chrono::FSI requires CUDA")
  message(STATUS "Chrono::FSI disabled")
  #mark_as_advanced(FORCE USE_FSI_DOUBLE)
  set(ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
  return()
endif()

#mark_as_advanced(CLEAR USE_FSI_DOUBLE)

# ------------------------------------------------------------------------------
# Additional options for this module
# ------------------------------------------------------------------------------

#option(USE_FSI_DOUBLE "Compile Chrono::FSI with double precision math" ON)
#if(USE_FSI_DOUBLE)
  set(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
#endif()

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the FSI module.
# Make some variables visible from parent directory
# ----------------------------------------------------------------------------

set(CH_FSI_INCLUDES "${CUDA_TOOLKIT_ROOT_DIR}/include")
set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES}" PARENT_SCOPE)

list(APPEND ${CUDA_cudadevrt_LIBRARY} LIBRARIES)
list(APPEND LIBRARIES ${CUDA_CUDART_LIBRARY})
list(APPEND LIBRARIES ${CUDA_cusparse_LIBRARY})
list(APPEND LIBRARIES ${CUDA_cublas_LIBRARY})
list(APPEND LIBRARIES ${CUDA_cudart_static_LIBRARY})

message(STATUS "CUDA libraries: ${LIBRARIES}")

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFSI.h.in
               ${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFSI.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFSI.h"
        DESTINATION include/chrono_fsi)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FSI FLUID-SOLID INTERACTION LIBRARY
#-----------------------------------------------------------------------------

set(ChronoEngine_FSI_SOURCES
    ChBce.cu
    ChCollisionSystemFsi.cu
    ChDeviceUtils.cu
    ChFluidDynamics.cu
    ChFsiDataManager.cu
    ChFsiForceExplicitSPH.cu
    ChFsiForceIISPH.cu
    ChFsiForce.cu
    ChFsiGeneral.cu
    ChSphGeneral.cu
    ChFsiLinearSolverBiCGStab.cpp
    ChFsiInterface.cpp
    ChSystemFsi.cpp
    ChFsiTypeConvert.cpp
)

set(ChronoEngine_FSI_HEADERS
    ChBce.cuh
    ChCollisionSystemFsi.cuh
    ChDeviceUtils.cuh
    ChFluidDynamics.cuh
    ChParams.cuh
    ChFsiDataManager.cuh
    ChFsiForceExplicitSPH.cuh
    ChFsiForceIISPH.cuh
    ChFsiForce.cuh
    ChFsiGeneral.cuh
    ChSphGeneral.cuh
    ExactLinearSolvers.cuh
    ChFsiLinearSolver.h
    ChFsiLinearSolverBiCGStab.h
    ChFsiInterface.h
    ChSystemFsi.h
    ChApiFsi.h
    ChFsiTypeConvert.h
    custom_math.h
)

source_group("" FILES
    ${ChronoEngine_FSI_SOURCES}
    ${ChronoEngine_FSI_HEADERS})

set(ChronoEngine_FSI_UTILS_SOURCES
    utils/ChUtilsJsonInput.cpp
    utils/ChUtilsGeneratorBce.cpp
    utils/ChUtilsGeneratorFluid.cpp
    utils/ChUtilsGeneratorFsi.cpp
    utils/ChUtilsPrintSph.cu
)

set(ChronoEngine_FSI_UTILS_HEADERS
    utils/ChUtilsJsonInput.h
    utils/ChUtilsGeneratorBce.h
    utils/ChUtilsGeneratorFluid.h
    utils/ChUtilsGeneratorFsi.h
    utils/ChUtilsPrintStruct.h
    utils/ChUtilsPrintSph.cuh
)

source_group(utils FILES
    ${ChronoEngine_FSI_UTILS_SOURCES}
    ${ChronoEngine_FSI_UTILS_HEADERS})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_fsi library
#-----------------------------------------------------------------------------

list(APPEND LIBRARIES "ChronoEngine")

cuda_add_library(ChronoEngine_fsi SHARED
    ${ChronoEngine_FSI_SOURCES}
    ${ChronoEngine_FSI_HEADERS}
    ${ChronoEngine_FSI_UTILS_SOURCES}
    ${ChronoEngine_FSI_UTILS_HEADERS}
)

set_target_properties(ChronoEngine_fsi PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_fsi PRIVATE "CH_API_COMPILE_FSI")
target_compile_definitions(ChronoEngine_fsi PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_fsi ${LIBRARIES})

install(TARGETS ChronoEngine_fsi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_FSI_HEADERS}
        DESTINATION include/chrono_fsi)
install(FILES ${ChronoEngine_FSI_UTILS_HEADERS}
        DESTINATION include/chrono_fsi/utils)
