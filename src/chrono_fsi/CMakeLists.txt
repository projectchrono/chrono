#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

IF(NOT ENABLE_MODULE_FSI)
  #mark_as_advanced(FORCE FSI_USE_DOUBLE)
  return()
ENDIF()

message(STATUS "\n==== Chrono FSI module ====\n")


#mark_as_advanced(CLEAR FSI_USE_DOUBLE)

# ------------------------------------------------------------------------------
# Additional options for this module
# ------------------------------------------------------------------------------

option(FSI_USE_DOUBLE "Compile Chrono::FSI with double precision math" ON)
if(FSI_USE_DOUBLE)
  set(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
endif()


# # ------------------------------------------------------------------------------
# # Hack to deal with MSVC runtime libraries
# # ------------------------------------------------------------------------------

# if(MSVC)
#   if(USE_MSVC_STATIC_RUNTIME)
#     set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
#     set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
#   else()
#     set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
#     set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
#   endif()
# endif()

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the FSI module.
# Make some variables visible from parent directory
# ----------------------------------------------------------------------------

# set(CH_FSI_INCLUDES "${CUDA_TOOLKIT_ROOT_DIR}/include")
# set(CH_FSI_LINKER_FLAGS "${CH_LINKERFLAG_LIB}")

# list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cudadevrt_LIBRARY})
# list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_CUDART_LIBRARY})
# list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cusparse_LIBRARY})
# list(APPEND CH_FSI_LINKED_LIBRARIES ${CUDA_cublas_LIBRARY})

# ------------------------------------------------------------------------------
# Add optional run-time visualization support
# ------------------------------------------------------------------------------


# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

# set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES}" PARENT_SCOPE)

# ----------------------------------------------------------------------------
# Configure submodules
# ----------------------------------------------------------------------------

add_subdirectory(sph)


# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFsi.h.in
               ${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFsi.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFsi.h"
        DESTINATION include/chrono_fsi)


# #-----------------------------------------------------------------------------
# # List all files for the SPH-based Chrono::FSI library
# #-----------------------------------------------------------------------------

# set(ChronoEngine_FSI_BASE_FILES
#     ../ChApiFsi.h
#     ../ChFsiDefinitions.h
#     ../ChFsiSystem.h
#     ../ChFsiSystem.cpp
#     ../ChFluidSystem.h
#     ../ChFluidSystem.cpp
#     ../ChFsiInterface.h
#     ../ChFsiInterface.cpp
# )

# set(ChronoEngine_FSI_FILES
#     ChFsiDefinitionsSPH.h
#     ChFsiSystemSPH.h
#     ChFsiSystemSPH.cpp
#     ChFluidSystemSPH.h
#     ChFluidSystemSPH.cpp
#     ChFsiInterfaceSPH.h
#     ChFsiInterfaceSPH.cpp
#     ChFsiProblemSPH.h
#     ChFsiProblemSPH.cpp
# )

# source_group("" FILES ${ChronoEngine_FSI_BASE_FILES} ${ChronoEngine_FSI_FILES})

# set(ChronoEngine_FSI_PHYSICS_FILES
#     physics/ChParams.h
#     physics/ChMarkerType.cuh
#     physics/FsiDataManager.cuh
#     physics/BceManager.cuh
#     physics/ChFluidDynamics.cuh
#     physics/ChCollisionSystemFsi.cuh
#     physics/ChFsiForce.cuh    
#     physics/ChFsiForceExplicitSPH.cuh
#     physics/ChFsiForceI2SPH.cuh
#     physics/ChSphGeneral.cuh

#     physics/FsiDataManager.cu
#  	physics/BceManager.cu
#     physics/ChFluidDynamics.cu
#     physics/ChCollisionSystemFsi.cu
#     physics/ChFsiForce.cu
#     physics/ChFsiForceExplicitSPH.cu
#     physics/ChFsiForceI2SPH.cu
#     physics/ChSphGeneral.cu
# )

# source_group(physics FILES ${ChronoEngine_FSI_PHYSICS_FILES})

# set(ChronoEngine_FSI_MATH_FILES
#     math/CustomMath.h
#     math/ExactLinearSolvers.cuh
#     math/ChFsiLinearSolver.h
#     math/ChFsiLinearSolverBiCGStab.h
#     math/ChFsiLinearSolverGMRES.h

#     math/ChFsiLinearSolverBiCGStab.cpp
#     math/ChFsiLinearSolverGMRES.cpp
# )

# source_group(math FILES ${ChronoEngine_FSI_MATH_FILES})

# set(ChronoEngine_FSI_UTILS_FILES
#     utils/ChUtilsPrintStruct.h
#     utils/ChUtilsPrintSph.cuh
#     utils/ChUtilsDevice.cuh
#     utils/ChUtilsTypeConvert.h

#     utils/ChUtilsPrintSph.cu
#     utils/ChUtilsDevice.cu
#     ../utils/ChBenchmark.h
# )

# source_group(utils FILES ${ChronoEngine_FSI_UTILS_FILES})

# set(ChronoEngine_FSI_VIS_FILES
#     visualization/ChFsiVisualization.h
#     visualization/ChFsiVisualization.cpp
# )

# if(ENABLE_MODULE_OPENGL)
#     set(ChronoEngine_FSI_VIS_FILES ${ChronoEngine_FSI_VIS_FILES}
#         visualization/ChFsiVisualizationGL.h
#         visualization/ChFsiVisualizationGL.cpp
#     )
# endif()

# if(ENABLE_MODULE_VSG)
#     set(ChronoEngine_FSI_VIS_FILES ${ChronoEngine_FSI_VIS_FILES}
#         visualization/ChFsiVisualizationVSG.h
#         visualization/ChFsiVisualizationVSG.cpp
#     )
# endif()

# source_group(visualization FILES ${ChronoEngine_FSI_VIS_FILES})

# set(ChronoEngine_FSI_STB_FILES
#     ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
#     ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
#     ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
#     ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
#     ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
# )
# source_group("utils\\stb" FILES ${ChronoEngine_FSI_STB_FILES})

# #-----------------------------------------------------------------------------
# # Create the ChronoEngine_fsi_sph library
# #-----------------------------------------------------------------------------

# add_library(ChronoEngine_fsi_sph
#     ${ChronoEngine_FSI_BASE_FILES}
#     ${ChronoEngine_FSI_FILES}
#     ${ChronoEngine_FSI_PHYSICS_FILES}
#     ${ChronoEngine_FSI_MATH_FILES}
#     ${ChronoEngine_FSI_UTILS_FILES}
#     ${ChronoEngine_FSI_VIS_FILES}
#     ${ChronoEngine_FSI_STB_FILES}
# )

# add_library(ChronoEngine::fsi ALIAS ChronoEngine_fsi)

# target_link_libraries(ChronoEngine_fsi PRIVATE ChronoEngine)
# target_link_libraries(ChronoEngine_fsi PRIVATE CUDA::cudart_static)
# target_link_libraries(ChronoEngine_fsi PRIVATE CUDA::nvrtc)
# target_link_libraries(ChronoEngine_fsi PRIVATE CUDA::cuda_driver)
# target_link_libraries(ChronoEngine_fsi PRIVATE CUDA::cublas)
# target_link_libraries(ChronoEngine_fsi PRIVATE CUDA::cusparse)


# if(ENABLE_MODULE_OPENGL)
#   target_link_libraries(ChronoEngine_fsi PRIVATE ChronoEngine_opengl)
#   if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#      target_compile_options(ChronoEngine_fsi PUBLIC $<$<COMPILE_LANGUAGE:CXX>:"-framework IOKit -framework Cocoa -framework OpenGL">)
#   endif()
# endif()

# if(ENABLE_MODULE_VSG)
#   target_link_libraries(ChronoEngine_fsi PRIVATE ChronoEngine_vsg)
# endif()

# # If using MSVC, disable warnings related to missing DLL interface
# if(MSVC)
#     target_compile_options(ChronoEngine_fsi PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
# endif()


# target_compile_definitions(ChronoEngine_fsi PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)

# set_target_properties(ChronoEngine_fsi PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})


# install(TARGETS ChronoEngine_fsi
#         EXPORT ChronoEngineTargets
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         INCLUDES DESTINATION include/chrono_fsi)

# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#         DESTINATION include/chrono_fsi
#         FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")
