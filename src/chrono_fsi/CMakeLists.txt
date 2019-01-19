#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

IF(NOT ENABLE_MODULE_FSI)
  return()
ENDIF()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

find_package(CUDA)
#SET (ENABLE_MODULE_FEA TRUE)

message(STATUS "==== Chrono FSI module ====")
mark_as_advanced(CLEAR CUDA_USE_STATIC_CUDA_RUNTIME)
mark_as_advanced(CLEAR USE_FSI_DOUBLE)
mark_as_advanced(CLEAR CUDA_TOOLKIT_ROOT_DIR)


SET(CUDA_SEPARABLE_COMPILATION OFF)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CUDA_SEPARABLE_COMPILATION OFF)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
IF(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  IF(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
    SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
    SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++14")
    SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++14")
  ENDIF()
ENDIF()
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
ENDIF()

# SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -gencode=arch=compute_30,code=sm_30)

INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/FindCudaArch.cmake)
SELECT_NVCC_ARCH_FLAGS(NVCC_FLAGS_EXTRA)
LIST(APPEND CUDA_NVCC_FLAGS ${NVCC_FLAGS_EXTRA})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

find_package(CUDA)

message(STATUS "  CUDA toolkit includes:    ${CUDA_TOOLKIT_ROOT_DIR}/include")
message(STATUS "  CUDA compile flags:       ${CUDA_NVCC_FLAGS}")

option(CUDA_PROPAGATE_HOST_FLAGS "Pass host compiler flags to cuda compiler" OFF)
option(USE_FSI_DOUBLE "Compile Chrono::FSI with double precision math" ON)
IF(USE_FSI_DOUBLE)
  SET(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
ENDIF()

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the FSI module.
# Make some variables visible from parent directory
# ----------------------------------------------------------------------------

set(CH_FSI_INCLUDES
    "${CUDA_TOOLKIT_ROOT_DIR}/include"
)


list(APPEND ${CUDA_cudadevrt_LIBRARY} LIBRARIES)
list(APPEND LIBRARIES ${CUDA_CUDART_LIBRARY})
list(APPEND LIBRARIES ${CUDA_CUDA_LIBRARY})
list(APPEND LIBRARIES ${CUDA_cusparse_LIBRARY})
list(APPEND LIBRARIES ${CUDA_cublas_LIBRARY})
list(APPEND LIBRARIES ${CUDA_cudart_static_LIBRARY})

message( STATUS "FSI_LIBRARIES: ${LIBRARIES}")

message( STATUS "CUDA_CUDART_LIBRARY: ${CUDA_CUDART_LIBRARY}")


set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES}" PARENT_SCOPE)





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

SET(ChronoEngine_FSI_SOURCES
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
    ChFsiInterface.cu
    ChSystemFsi.cpp
    ChFsiTypeConvert.cpp

)

SET(ChronoEngine_FSI_HEADERS
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
    ChFsiInterface.cuh
    ChSystemFsi.h
    ChApiFsi.h
    ChFsiTypeConvert.h
    custom_math.h
)

SOURCE_GROUP("" FILES
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

set(CXX_FLAGS ${CH_CXX_FLAGS})

list(APPEND LIBRARIES "ChronoEngine")
#set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_PARALLEL)
  set(CXX_FLAGS ${CH_PARALLEL_CXX_FLAGS})
  include_directories(${CH_PARALLEL_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_parallel)
endif()

if(ENABLE_MODULE_FEA)
	set(CXX_FLAGS ${CH_FEA_CXX_FLAGS})
	include_directories(${CH_FEA_INCLUDES})
	list(APPEND LIBRARIES ChronoEngine_fea)
endif()

if(ENABLE_MODULE_VEHICLE)
  include_directories(${CH_VEHICLE_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_vehicle)
endif()

CUDA_ADD_LIBRARY(ChronoEngine_fsi SHARED
    ${ChronoEngine_FSI_SOURCES}
    ${ChronoEngine_FSI_HEADERS}
    ${ChronoEngine_FSI_UTILS_SOURCES}
    ${ChronoEngine_FSI_UTILS_HEADERS}
)

SET_TARGET_PROPERTIES(ChronoEngine_fsi PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_FSI")



TARGET_LINK_LIBRARIES(ChronoEngine_fsi ${LIBRARIES})
message(STATUS "  FSI Libraries:    ${LIBRARIES}")

INSTALL(TARGETS ChronoEngine_fsi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_FSI_HEADERS}
        DESTINATION include/chrono_fsi)
INSTALL(FILES ${ChronoEngine_FSI_UTILS_HEADERS}
        DESTINATION include/chrono_fsi/utils)
