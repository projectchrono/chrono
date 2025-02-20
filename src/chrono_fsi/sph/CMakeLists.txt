#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

message(STATUS "Add SPH-based Chrono::FSI submodule")

# Return now if Eigen version < 3.3.6
if(EIGEN3_VERSION VERSION_LESS "3.3.6")
    message(WARNING "Eigen version (${EIGEN3_VERSION}) is less than the required version (3.3.6); disabling Chrono::FSI")
    set(CH_ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

# Return now if CUDA is not available
if(NOT CHRONO_CUDA_FOUND)
    message(WARNING "Chrono::FSI::SPH requires CUDA, but CUDA was not found; disabling Chrono::FSI::SPH")
    set(CH_ENABLE_MODULE_FSI OFF CACHE BOOL "Enable the Chrono FSI module" FORCE)
    return()
endif()

message(STATUS "  Use double precision? ${CH_USE_FSI_DOUBLE}")
message(STATUS "  Chrono CUDA architectures: ${CHRONO_CUDA_ARCHITECTURES}")

#-----------------------------------------------------------------------------
# List all files for the SPH-based Chrono::FSI library
#-----------------------------------------------------------------------------

set(FSI_FILES
    ../ChApiFsi.h
    ../ChFsiDefinitions.h
    ../ChFsiSystem.h
    ../ChFsiSystem.cpp
    ../ChFluidSystem.h
    ../ChFluidSystem.cpp
    ../ChFsiInterface.h
    ../ChFsiInterface.cpp
)
source_group("" FILES ${FSI_FILES})

# --------------- SPH FSI FILES

set(FSISPH_BASE_FILES
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
source_group(base FILES ${FSISPH_BASE_FILES})

set(FSISPH_PHYSICS_FILES
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
source_group(physics FILES ${FSISPH_PHYSICS_FILES})

set(FSISPH_MATH_FILES
    math/CustomMath.h
    math/ExactLinearSolvers.cuh
    math/ChFsiLinearSolver.h
    #math/ChFsiLinearSolverBiCGStab.h
    #math/ChFsiLinearSolverGMRES.h
    #math/ChFsiLinearSolverBiCGStab.cpp
    #math/ChFsiLinearSolverGMRES.cpp
)
source_group(math FILES ${FSISPH_MATH_FILES})

set(FSISPH_UTILS_FILES
    utils/ChUtilsPrintStruct.h
    utils/ChUtilsPrintSph.cuh
    utils/ChUtilsDevice.cuh
    utils/ChUtilsTypeConvert.h

    utils/ChUtilsPrintSph.cu
    utils/ChUtilsDevice.cu
    ../utils/ChBenchmark.h
)
source_group(utils FILES ${FSISPH_UTILS_FILES})

set(FSISPH_VIS_FILES
    visualization/ChFsiVisualization.h
    visualization/ChFsiVisualization.cpp
)
source_group(visualization FILES ${FSISPH_VIS_FILES})

# --------------- THIRD-PARTY FILES

set(FSISPH_STB_FILES
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
    ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)
source_group("utils\\stb" FILES ${FSISPH_STB_FILES})

# --------------- SPH FSI OpenGL INTERFACE

if(CH_ENABLE_MODULE_OPENGL)
    set(FSISPH_GL_FILES
        visualization/ChFsiVisualizationGL.h
        visualization/ChFsiVisualizationGL.cpp
    )
else()
    set(FSISPH_GL_FILES "")
endif()
source_group("" FILES ${FSISPH_GL_FILES})

# --------------- SPH FSI VSG INTERFACE

if(CH_ENABLE_MODULE_VSG)
    set(FSISPH_VSG_FILES
        visualization/ChFsiVisualizationVSG.h
        visualization/ChFsiVisualizationVSG.cpp
    )
else()
    set(FSISPH_VSG_FILES "")
endif()
source_group("" FILES ${FSISPH_VSG_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_FSI "")
if(CH_ENABLE_MODULE_OPENGL)
  list (APPEND DEPENDENCIES_FSI "OPENGL")
endif()
if(CH_ENABLE_MODULE_VSG)
  list (APPEND DEPENDENCIES_FSI "VSG")
endif()

set(DEPENDENCIES_FSI ${DEPENDENCIES_FSI} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Create the Chrono_fsisph library
#-----------------------------------------------------------------------------

message(STATUS "Add Chrono_fsisph library")

add_library(Chrono_fsisph
    ${FSI_FILES}
    ${FSISPH_BASE_FILES}
    ${FSISPH_PHYSICS_FILES}
    ${FSISPH_MATH_FILES}
    ${FSISPH_UTILS_FILES}
    ${FSISPH_VIS_FILES}
    ${FSISPH_STB_FILES}
)
add_library(Chrono::fsisph ALIAS Chrono_fsisph)

if (CH_STATIC)
  set_target_properties(Chrono_fsisph PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  # disable warnings related to missing DLL interface
  target_compile_options(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
  set_target_properties(Chrono_fsisph PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_fsisph PRIVATE Chrono_core)

target_link_libraries(Chrono_fsisph PUBLIC CUDA::cudart_static)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::nvrtc)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::cuda_driver)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::cublas)
target_link_libraries(Chrono_fsisph PUBLIC CUDA::cusparse)

target_link_libraries(Chrono_fsisph PUBLIC Thrust::Thrust)
target_compile_definitions(Chrono_fsisph PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
target_compile_definitions(Chrono_fsisph PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

target_compile_definitions(Chrono_fsisph PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)

set_target_properties(Chrono_fsisph PROPERTIES CUDA_ARCHITECTURES ${CHRONO_CUDA_ARCHITECTURES})

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
    message(STATUS "Add Chrono_fsisph_vsg library")

    add_library(Chrono_fsisph_vsg
        ${FSISPH_VSG_FILES}
    )
    add_library(Chrono::fsisph_vsg ALIAS Chrono_fsisph_vsg)
   
    if (CH_STATIC)
      set_target_properties(Chrono_fsisph_vsg PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()
 
    if(MSVC)
      # disable warnings related to missing DLL interface
      target_compile_options(Chrono_fsisph_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
      set_target_properties(Chrono_fsisph_vsg PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_compile_definitions(Chrono_fsisph_vsg PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)
    
    target_link_libraries(Chrono_fsisph_vsg PRIVATE Chrono_core Chrono_fsisph)
    target_link_libraries(Chrono_fsisph_vsg PUBLIC Chrono_vsg)

    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cudart_static)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::nvrtc)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cuda_driver)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cublas)
    target_link_libraries(Chrono_fsisph_vsg PRIVATE CUDA::cusparse)

    target_link_libraries(Chrono_fsisph_vsg PRIVATE Thrust::Thrust)
    target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
    target_compile_definitions(Chrono_fsisph_vsg PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

    install(TARGETS Chrono_fsisph_vsg
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_fsi) # TODO DARIOM: verify if correct folder

endif()

#-------------------------------------------------------------------------------
# Add the Chrono_fsisph_opengl library
#-------------------------------------------------------------------------------

if(CH_ENABLE_MODULE_OPENGL)
    message(STATUS "Add Chrono_fsisph_opengl library")

    add_library(Chrono_fsisph_opengl
        ${FSISPH_GL_FILES}
    )
    add_library(Chrono::fsisph_opengl ALIAS Chrono_fsisph_opengl)
   
    if (CH_STATIC)
      set_target_properties(Chrono_fsisph_opengl PROPERTIES POSITION_INDEPENDENT_CODE ON)
    endif()
    
    if(MSVC)
      # disable warnings related to missing DLL interface
      target_compile_options(Chrono_fsisph_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
      set_target_properties(Chrono_fsisph_opengl PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_compile_definitions(Chrono_fsisph_opengl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_FSI>)
    
    target_link_libraries(Chrono_fsisph_opengl PRIVATE Chrono_core Chrono_fsisph)
    target_link_libraries(Chrono_fsisph_opengl PUBLIC Chrono_opengl)
    
    target_link_libraries(Chrono_fsisph_opengl PRIVATE CUDA::cudart_static)
    target_link_libraries(Chrono_fsisph_opengl PRIVATE CUDA::nvrtc)
    target_link_libraries(Chrono_fsisph_opengl PRIVATE CUDA::cuda_driver)
    target_link_libraries(Chrono_fsisph_opengl PRIVATE CUDA::cublas)
    target_link_libraries(Chrono_fsisph_opengl PRIVATE CUDA::cusparse)

    target_link_libraries(Chrono_fsisph_opengl PRIVATE OpenMP::OpenMP_CXX) # TODO DARIOM: if needed we should check that OpenMP is available
    target_link_libraries(Chrono_fsisph_opengl PRIVATE Thrust::Thrust)
    target_compile_definitions(Chrono_fsisph_opengl PRIVATE "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA")
    target_compile_definitions(Chrono_fsisph_opengl PRIVATE "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")

    if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        target_compile_options(Chrono_fsisph_opengl PUBLIC $<$<COMPILE_LANGUAGE:CXX>:"-framework IOKit -framework Cocoa -framework OpenGL">)
    endif()

    install(TARGETS Chrono_fsisph_opengl
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            INCLUDES DESTINATION include/chrono_fsi) # TODO DARIOM: verify if correct folder

endif()

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_fsi
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")

# Install 3rd party headers       
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
