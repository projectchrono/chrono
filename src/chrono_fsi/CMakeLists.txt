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

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI")
    set(ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

# Return now if CUDA is not available
if(NOT CUDA_FOUND)
    message(WARNING "Chrono::FSI requires CUDA, but CUDA was not found; disabling Chrono::FSI")
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

    ChSystemFsi.cpp
    ChFsiInterface.cpp
    ChFsiDataManager.cu
    utils/ChUtilsDevice.cu
    utils/ChUtilsTypeConvert.cpp
    
 	physics/ChBce.cu
    physics/ChFluidDynamics.cu
    physics/ChCollisionSystemFsi.cu
    physics/ChFsiForce.cu
    physics/ChFsiForceExplicitSPH.cu   
    physics/ChFsiForceI2SPH.cu
    physics/ChFsiForceIISPH.cu
    physics/ChFsiGeneral.cu
    physics/ChSphGeneral.cu
    
    math/ChFsiLinearSolverBiCGStab.cpp
    math/ChFsiLinearSolverGMRES.cpp

)

set(ChronoEngine_FSI_HEADERS

    ChApiFsi.h
    ChSystemFsi.h
    ChFsiInterface.h
    ChFsiDataManager.cuh
 	utils/ChUtilsDevice.cuh
    utils/ChUtilsTypeConvert.h

    physics/ChBce.cuh
    physics/ChFluidDynamics.cuh
    physics/ChCollisionSystemFsi.cuh
    physics/ChFsiForce.cuh    
    physics/ChFsiForceExplicitSPH.cuh    
    physics/ChFsiForceI2SPH.cuh
    physics/ChFsiForceIISPH.cuh
    physics/ChFsiGeneral.cuh
    physics/ChSphGeneral.cuh
    physics/ChParams.cuh

    math/custom_math.h
    math/ExactLinearSolvers.cuh
    math/ChFsiLinearSolver.h
    math/ChFsiLinearSolverBiCGStab.h   
    math/ChFsiLinearSolverGMRES.h 
)

source_group("" FILES
    ${ChronoEngine_FSI_SOURCES}
    ${ChronoEngine_FSI_HEADERS})

set(ChronoEngine_FSI_UTILS_SOURCES
	utils/ChUtilsJSON.cpp
    utils/ChUtilsGeneratorBce.cpp
    utils/ChUtilsGeneratorFluid.cpp
    utils/ChUtilsGeneratorFsi.cpp
    utils/ChUtilsPrintSph.cu
)

set(ChronoEngine_FSI_UTILS_HEADERS
   	utils/ChUtilsJSON.h
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

set(CXX_FLAGS ${CH_CXX_FLAGS})

list(APPEND LIBRARIES "ChronoEngine")
#set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_PARALLEL)
  set(CXX_FLAGS ${CH_PARALLEL_CXX_FLAGS})
  include_directories(${CH_PARALLEL_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_parallel)
endif()

if(ENABLE_MODULE_VEHICLE)
  include_directories(${CH_VEHICLE_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_vehicle)
endif()

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
