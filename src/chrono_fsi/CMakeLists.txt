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

message(STATUS "\n==== Chrono FSI module ====\n")

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

option(USE_FSI_DOUBLE "Compile Chrono::FSI with double precision math" ON)
if(USE_FSI_DOUBLE)
  set(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
endif()

# ------------------------------------------------------------------------------
# If using MSVC, disable warnings related to missing DLL interface
# ------------------------------------------------------------------------------

if(MSVC)
  #add_compile_options(/wd4251)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /wd4251")
endif()

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the FSI module.
# Make some variables visible from parent directory
# ----------------------------------------------------------------------------

set(CH_FSI_INCLUDES "${CUDA_TOOLKIT_ROOT_DIR}/include")
set(CH_FSI_LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")
set(CH_FSI_LINKED_LIBRARIES ${CUDA_FRAMEWORK})

list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cudadevrt_LIBRARY})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_CUDART_LIBRARY})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cusparse_LIBRARY})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cublas_LIBRARY})

message(STATUS "CUDA libraries: ${CH_FSI_LINKED_LIBRARIES}")

list(APPEND CH_FSI_LINKED_LIBRARIES ChronoEngine)

# ------------------------------------------------------------------------------
# Add optional run-time visualization support
# ------------------------------------------------------------------------------

if(ENABLE_MODULE_OPENGL)
  include_directories(${CH_OPENGL_INCLUDES})
  set(CH_FSI_INCLUDES ${CH_FSI_INCLUDES} ${CH_OPENGL_INCLUDES})
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(CH_FSI_LINKER_FLAGS  "${CH_FSI_LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
  list(APPEND CH_FSI_LINKED_LIBRARIES ChronoEngine_opengl)
endif()

if(ENABLE_MODULE_VSG)
  set(CMAKE_CXX_STANDARD 17)
  list(APPEND CH_FSI_LINKED_LIBRARIES ChronoEngine_vsg)
endif()

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

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
#-----------------------------------------------------------------------------

set(ChronoEngine_FSI_FILES
    ChApiFsi.h
    ChSystemFsi.h
    ChDefinitionsFsi.h
    ChSystemFsi.cpp
)

source_group("" FILES ${ChronoEngine_FSI_FILES})

set(ChronoEngine_FSI_PHYSICS_FILES
    physics/ChParams.h
    physics/ChFsiInterface.h
    physics/ChFsiGeneral.h
    physics/ChSystemFsi_impl.cuh
    physics/ChBce.cuh
    physics/ChFluidDynamics.cuh
    physics/ChCollisionSystemFsi.cuh
    physics/ChFsiForce.cuh    
    physics/ChFsiForceExplicitSPH.cuh
    physics/ChFsiForceI2SPH.cuh
    physics/ChFsiForceIISPH.cuh
    physics/ChSphGeneral.cuh

    physics/ChFsiInterface.cpp
    physics/ChSystemFsi_impl.cu
 	physics/ChBce.cu
    physics/ChFluidDynamics.cu
    physics/ChCollisionSystemFsi.cu
    physics/ChFsiForce.cu
    physics/ChFsiForceExplicitSPH.cu
    physics/ChFsiForceI2SPH.cu
    physics/ChFsiForceIISPH.cu
    physics/ChFsiGeneral.cpp
    physics/ChSphGeneral.cu
)

source_group(physics FILES ${ChronoEngine_FSI_PHYSICS_FILES})

set(ChronoEngine_FSI_MATH_FILES
    math/custom_math.h
    math/ExactLinearSolvers.cuh
    math/ChFsiLinearSolver.h
    math/ChFsiLinearSolverBiCGStab.h
    math/ChFsiLinearSolverGMRES.h

    math/ChFsiLinearSolverBiCGStab.cpp
    math/ChFsiLinearSolverGMRES.cpp
)

source_group(math FILES ${ChronoEngine_FSI_MATH_FILES})

set(ChronoEngine_FSI_UTILS_FILES
    utils/ChUtilsGeneratorFluid.h
    utils/ChUtilsPrintStruct.h
    utils/ChUtilsPrintSph.cuh
    utils/ChUtilsDevice.cuh
    utils/ChUtilsTypeConvert.h

    utils/ChUtilsGeneratorFluid.cpp
    utils/ChUtilsPrintSph.cu
    utils/ChUtilsDevice.cu
    utils/ChUtilsTypeConvert.cpp
)

source_group(utils FILES ${ChronoEngine_FSI_UTILS_FILES})

set(ChronoEngine_FSI_VIS_FILES
    visualization/ChFsiVisualization.h
    visualization/ChFsiVisualization.cpp
)

if(ENABLE_MODULE_OPENGL)
    set(ChronoEngine_FSI_VIS_FILES ${ChronoEngine_FSI_VIS_FILES}
        visualization/ChFsiVisualizationGL.h
        visualization/ChFsiVisualizationGL.cpp
    )
endif()

if(ENABLE_MODULE_VSG)
    set(ChronoEngine_FSI_VIS_FILES ${ChronoEngine_FSI_VIS_FILES}
        visualization/ChFsiVisualizationVSG.h
        visualization/ChFsiVisualizationVSG.cpp
    )
endif()

source_group(visualization FILES ${ChronoEngine_FSI_VIS_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_fsi library
#-----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})

cuda_add_library(ChronoEngine_fsi SHARED
    ${ChronoEngine_FSI_FILES}
    ${ChronoEngine_FSI_PHYSICS_FILES}
    ${ChronoEngine_FSI_MATH_FILES}
    ${ChronoEngine_FSI_UTILS_FILES}
    ${ChronoEngine_FSI_VIS_FILES}
)

set_target_properties(ChronoEngine_fsi PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_FSI_LINKER_FLAGS}")

target_compile_definitions(ChronoEngine_fsi PRIVATE "CH_API_COMPILE_FSI")
target_compile_definitions(ChronoEngine_fsi PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_fsi ${CH_FSI_LINKED_LIBRARIES})

install(TARGETS ChronoEngine_fsi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_fsi
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")
