#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

message(STATUS "Add SPH-based Chrono::FSI submodule")

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI")
    set(ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

# Return now if CUDA is not available
if(NOT CUDA_FOUND)
    message("Chrono::FSI requires CUDA, but CUDA was not found; disabling Chrono::FSI")
    set(ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

message(STATUS "  Use double precision? " ${FSI_USE_DOUBLE})

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the FSI-SPH module
# ----------------------------------------------------------------------------

set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES} ${CUDA_TOOLKIT_ROOT_DIR}/include")

list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_FRAMEWORK})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cudadevrt_LIBRARY})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_CUDART_LIBRARY})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cusparse_LIBRARY})
list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cublas_LIBRARY})

message(STATUS "  FSI-SPH libraries: ${CH_FSI_LINKED_LIBRARIES}")

# Make some variables visible from parent directory

set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES}" PARENT_SCOPE)
set(CH_FSI_LINKED_LIBRARIES "${CH_FSI_LINKED_LIBRARIES}" PARENT_SCOPE)

#-----------------------------------------------------------------------------
# List all files for the SPH-based Chrono::FSI library
#-----------------------------------------------------------------------------

set(ChronoEngine_FSI_BASE_FILES
    ../ChApiFsi.h
    ../ChFsiDefinitions.h
    ../ChFsiSystem.h
    ../ChFsiSystem.cpp
    ../ChFluidSystem.h
    ../ChFluidSystem.cpp
    ../ChFsiInterface.h
    ../ChFsiInterface.cpp
)

set(ChronoEngine_FSI_FILES
    ChFsiDefinitionsSPH.h
    ChFsiSystemSPH.h
    ChFsiSystemSPH.cpp
    ChFluidSystemSPH.h
    ChFluidSystemSPH.cpp
    ChFsiInterfaceSPH.h
    ChFsiInterfaceSPH.cpp
    ChFsiProblemSPH.h
    ChFsiProblemSPH.cpp
)

source_group("" FILES ${ChronoEngine_FSI_BASE_FILES} ${ChronoEngine_FSI_FILES})

set(ChronoEngine_FSI_PHYSICS_FILES
    physics/ChParams.h
    physics/ChMarkerType.cuh
    physics/FsiDataManager.cuh
    physics/BceManager.cuh
    physics/ChFluidDynamics.cuh
    physics/ChCollisionSystemFsi.cuh
    physics/ChFsiForce.cuh    
    physics/ChFsiForceExplicitSPH.cuh
    physics/ChFsiForceI2SPH.cuh
    physics/ChSphGeneral.cuh

    physics/FsiDataManager.cu
 	physics/BceManager.cu
    physics/ChFluidDynamics.cu
    physics/ChCollisionSystemFsi.cu
    physics/ChFsiForce.cu
    physics/ChFsiForceExplicitSPH.cu
    physics/ChFsiForceI2SPH.cu
    physics/ChSphGeneral.cu
)

source_group(physics FILES ${ChronoEngine_FSI_PHYSICS_FILES})

set(ChronoEngine_FSI_MATH_FILES
    math/CustomMath.h
    math/ExactLinearSolvers.cuh
    math/ChFsiLinearSolver.h
    math/ChFsiLinearSolverBiCGStab.h
    math/ChFsiLinearSolverGMRES.h

    math/ChFsiLinearSolverBiCGStab.cpp
    math/ChFsiLinearSolverGMRES.cpp
)

source_group(math FILES ${ChronoEngine_FSI_MATH_FILES})

set(ChronoEngine_FSI_UTILS_FILES
    utils/ChUtilsPrintStruct.h
    utils/ChUtilsPrintSph.cuh
    utils/ChUtilsDevice.cuh
    utils/ChUtilsTypeConvert.h

    utils/ChUtilsPrintSph.cu
    utils/ChUtilsDevice.cu
    ../utils/ChBenchmark.h
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

set(ChronoEngine_FSI_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("utils\\stb" FILES ${ChronoEngine_FSI_STB_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_fsi_sph library
#-----------------------------------------------------------------------------

cuda_add_library(ChronoEngine_fsi_sph
    ${ChronoEngine_FSI_BASE_FILES}
    ${ChronoEngine_FSI_FILES}
    ${ChronoEngine_FSI_PHYSICS_FILES}
    ${ChronoEngine_FSI_MATH_FILES}
    ${ChronoEngine_FSI_UTILS_FILES}
    ${ChronoEngine_FSI_VIS_FILES}
    ${ChronoEngine_FSI_STB_FILES}
)

set_target_properties(ChronoEngine_fsi_sph PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_FSI_LINKER_FLAGS}")

target_compile_definitions(ChronoEngine_fsi_sph PRIVATE "CH_API_COMPILE_FSI")
target_compile_definitions(ChronoEngine_fsi_sph PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_fsi_sph ${CH_FSI_LINKED_LIBRARIES})

install(TARGETS ChronoEngine_fsi_sph
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_fsi
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")
