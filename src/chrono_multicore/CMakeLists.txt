#===============================================================================
# CMake configuration file for the ChronoEngine_Multicore library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# The Chrono::Multicore module will build even if OpenMP is not found,
# it just won't have parallel computation support.
# If TBB is defined, thrust will use TBB but the rest of the code will be single
# threaded

option(ENABLE_MODULE_MULTICORE "Enable the Chrono Multicore module" OFF)

# Return now if this module is not enabled

if(NOT ENABLE_MODULE_MULTICORE)
  mark_as_advanced(FORCE Blaze_ROOT)
  mark_as_advanced(FORCE USE_MULTICORE_DOUBLE)
  mark_as_advanced(FORCE USE_MULTICORE_SIMD)
  mark_as_advanced(FORCE USE_MULTICORE_CUDA)
  return()
endif()

message(STATUS "\n==== Chrono Multicore module ====\n")

# Return now if Thrust not available
if (NOT THRUST_FOUND)
  message("Chrono::Multicore requires Thrust, but Thrust was not found; disabling Chrono::Multicore")
  set(ENABLE_MODULE_MULTICORE OFF CACHE BOOL "Enable the Chrono Multicore module" FORCE)
  return()
endif()

find_package(Blaze REQUIRED)

if (NOT Blaze_FOUND)
  set(Blaze_ROOT "" CACHE PATH "Blaze library location (should contain 'blaze/Blaze.h')")
  message(FATAL_ERROR "Chrono::Multicore requires Blaze, but Blaze was not found; set Blaze_ROOT to the Blaze installation directory")
endif()



mark_as_advanced(CLEAR Blaze_ROOT)
mark_as_advanced(CLEAR USE_MULTICORE_DOUBLE)
mark_as_advanced(CLEAR USE_MULTICORE_SIMD)
mark_as_advanced(CLEAR USE_MULTICORE_CUDA)

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------

# ----- CUDA support -----
cmake_dependent_option(USE_MULTICORE_CUDA "Enable CUDA support in Chrono::Multicore (if available)" OFF "CMAKE_CUDA_COMPILER" OFF)

if(USE_MULTICORE_CUDA)
  set(CHRONO_MULTICORE_USE_CUDA "#define CHRONO_MULTICORE_USE_CUDA")
else()
  set(CHRONO_MULTICORE_USE_CUDA "#undef CHRONO_MULTICORE_USE_CUDA")
endif()

# ----- Double precision support -----

OPTION(USE_MULTICORE_DOUBLE "Compile Chrono::Multicore with double precision math" ON)

if(USE_MULTICORE_DOUBLE)
  set(CHRONO_MULTICORE_USE_DOUBLE "#define CHRONO_MULTICORE_USE_DOUBLE")
endif()


# ----------------------------------------------------------------------------
# Generate and install configuration header file.
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ChConfigMulticore.h.in
  ${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h
  )

install(FILES "${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h"
        DESTINATION include/chrono_multicore)

# ------------------------------------------------------------------------------
# List the files in the Chrono::Multicore module
# ------------------------------------------------------------------------------

set(ChronoEngine_Multicore_BASE
    ChApiMulticore.h
    ChMulticoreDefines.h
    ChSettings.h
    ChMeasures.h
    ChDataManager.h
    ChTimerMulticore.h
    ChDataManager.cpp
    )

source_group("" FILES ${ChronoEngine_Multicore_BASE})

set(ChronoEngine_Multicore_PHYSICS
    physics/ChSystemMulticore.h
    physics/ChSystemMulticore.cpp
    physics/ChSystemMulticoreNSC.cpp
    physics/ChSystemMulticoreSMC.cpp
    physics/Ch3DOFContainer.h
    physics/Ch3DOFContainer.cpp
    physics/ChFluidKernels.h
    physics/ChFluidContainer.cpp
    physics/ChParticleContainer.cpp
    physics/ChMPMSettings.h
    )

source_group(physics FILES ${ChronoEngine_Multicore_PHYSICS})

set(ChronoEngine_Multicore_CUDA
    cuda/matrixf.cuh
    cuda/svd.h
    cuda/ChCudaHelper.cuh
    cuda/ChGPUVector.cuh
    cuda/ChMPM.cu
    cuda/ChMPM.cuh
    cuda/ChMPMUtils.h
    )

source_group(cuda FILES ${ChronoEngine_Multicore_CUDA})

    
set(ChronoEngine_Multicore_SOLVER
    solver/ChSystemDescriptorMulticore.h
    solver/ChIterativeSolverMulticore.h
    solver/ChIterativeSolverMulticore.cpp
    solver/ChIterativeSolverMulticoreNSC.cpp
    solver/ChIterativeSolverMulticoreSMC.cpp
    solver/ChSolverMulticore.h
    solver/ChSolverMulticore.cpp
    solver/ChSolverMulticoreAPGD.cpp
    solver/ChSolverMulticoreAPGDREF.cpp
    solver/ChSolverMulticoreMINRES.cpp
    solver/ChSolverMulticoreBB.cpp
    solver/ChSolverMulticoreJacobi.cpp
    solver/ChSolverMulticoreCG.cpp
    solver/ChSolverMulticoreGS.cpp
    solver/ChSolverMulticoreSPGQP.cpp
    solver/ChSchurProduct.cpp
    )

source_group(solver FILES ${ChronoEngine_Multicore_SOLVER})

set(ChronoEngine_Multicore_CONSTRAINTS
    constraints/ChConstraintRigidRigid.cpp
    constraints/ChConstraintRigidRigid.h
    constraints/ChConstraintBilateral.cpp
    constraints/ChConstraintBilateral.h
    constraints/ChConstraintUtils.cpp
    constraints/ChConstraintUtils.h
    )

source_group(constraints FILES ${ChronoEngine_Multicore_CONSTRAINTS})

set(ChronoEngine_Multicore_COLLISION
    collision/ChCollisionSystemChronoMulticore.h
    collision/ChCollisionSystemChronoMulticore.cpp
    collision/ChContactContainerMulticore.h
    collision/ChContactContainerMulticore.cpp
    collision/ChContactContainerMulticoreNSC.h
    collision/ChContactContainerMulticoreNSC.cpp
    collision/ChContactContainerMulticoreSMC.h
    collision/ChContactContainerMulticoreSMC.cpp
    )

source_group(collision FILES ${ChronoEngine_Multicore_COLLISION})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_multicore library
# ------------------------------------------------------------------------------

add_library(ChronoEngine_multicore
  ${ChronoEngine_Multicore_BASE}
  ${ChronoEngine_Multicore_PHYSICS}
  ${ChronoEngine_Multicore_COLLISION}
  ${ChronoEngine_Multicore_CONSTRAINTS}
  ${ChronoEngine_Multicore_SOLVER}
)

add_library(ChronoEngine::ChronoEngine_multicore ALIAS ChronoEngine_multicore)

if(USE_MULTICORE_CUDA)
  target_sources(ChronoEngine_multicore PRIVATE ${ChronoEngine_Multicore_CUDA})
  target_link_libraries(ChronoEngine_multicore PUBLIC CUDA::cudart_static)
  target_link_libraries(ChronoEngine_multicore PUBLIC CUDA::nvrtc)
  target_link_libraries(ChronoEngine_multicore PUBLIC CUDA::cuda_driver)
endif()

set_target_properties(ChronoEngine_multicore PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})

target_link_libraries(ChronoEngine_multicore PRIVATE ChronoEngine)
if(ENABLE_OPENMP)
  target_link_libraries(ChronoEngine_multicore PRIVATE OpenMP::OpenMP_CXX)
elseif(ENABLE_TBB)
  target_link_libraries(ChronoEngine_multicore PRIVATE ${TBB_LIBRARIES})
endif()
target_link_libraries(ChronoEngine_multicore PRIVATE Thrust::Thrust) # here Thrust is available for sure
target_link_libraries(ChronoEngine_multicore PRIVATE Blaze::Blaze)
if(BOOST_REQUIRED)
  target_include_directories(ChronoEngine_multicore PRIVATE "${Boost_INCLUDE_DIRS}")
endif()

# On Visual Studio, disable warning C4146 from Blaze
# ("unary minus operator applied to unsigned type, result still unsigned")
if(MSVC)
  target_compile_options(ChronoEngine_multicore PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4146>)
endif()

target_compile_definitions(ChronoEngine_multicore PRIVATE CH_API_COMPILE_MULTICORE)
target_compile_definitions(ChronoEngine_multicore PRIVATE CH_IGNORE_DEPRECATED)
target_compile_definitions(ChronoEngine_multicore PRIVATE BT_THREADSAFE)

# ----- Configure Blaze and Thrust -----

if(ENABLE_OPENMP)
  target_compile_definitions(ChronoEngine_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP")
  target_compile_definitions(ChronoEngine_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP")
elseif(ENABLE_TBB)
  target_compile_definitions(ChronoEngine_multicore PUBLIC "BLAZE_USE_CPP_THREADS")
  target_compile_definitions(ChronoEngine_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_TBB")
  target_compile_definitions(ChronoEngine_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_TBB")
else()
  target_compile_definitions(ChronoEngine_multicore PUBLIC "BLAZE_USE_CPP_THREADS")
  target_compile_definitions(ChronoEngine_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CPP")
  target_compile_definitions(ChronoEngine_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")
endif()




file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindBlaze.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)


install(TARGETS ChronoEngine_multicore
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_multicore)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_multicore
        FILES_MATCHING PATTERN "*.h")

if(USE_MULTICORE_CUDA)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
          DESTINATION include/chrono_multicore
          FILES_MATCHING PATTERN "*.cuh")
endif()

# Install required chrono_thirdparty headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/easylogging
        DESTINATION include/chrono_thirdparty
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")

install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindBlaze.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
        )