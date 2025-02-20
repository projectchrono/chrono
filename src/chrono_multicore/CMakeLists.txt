#===============================================================================
# CMake configuration file for the Chrono_Multicore library
#
# Invoked from the main CMakeLists.txt using ADD_SUBDIRECTORY()
#===============================================================================

# The Chrono::Multicore module will build even if OpenMP is not found,
# it just won't have parallel computation support.
# If TBB is defined, thrust will use TBB but the rest of the code will be single
# threaded

option(CH_ENABLE_MODULE_MULTICORE "Enable the Chrono Multicore module" OFF)

# Return now if this module is not enabled

if(NOT CH_ENABLE_MODULE_MULTICORE)
  mark_as_advanced(FORCE Blaze_ROOT)
  mark_as_advanced(FORCE CH_USE_MULTICORE_DOUBLE)
  return()
endif()

message(STATUS "\n==== Chrono Multicore module ====\n")

# Return now if OpenMP is disabled
if(NOT CH_ENABLE_OPENMP)
  message("Chrono::Multicore requires OpenMP, but OpenMP is disabled; disabling Chrono::Multicore")
  set(CH_ENABLE_MODULE_MULTICORE OFF CACHE BOOL "Enable the Chrono Multicore module" FORCE)
  return()
endif()

# Return now if Thrust not available
if(NOT CHRONO_THRUST_FOUND)
  message("Chrono::Multicore requires Thrust, but Thrust was not found; disabling Chrono::Multicore")
  set(CH_ENABLE_MODULE_MULTICORE OFF CACHE BOOL "Enable the Chrono Multicore module" FORCE)
  return()
endif()

# Find Blaze
find_package(Blaze REQUIRED)
if(Blaze_FOUND)
  mark_as_advanced(FORCE Blaze_ROOT)
else()
  set(Blaze_ROOT "" CACHE PATH "Blaze library location (should contain 'blaze/Blaze.h')")
  mark_as_advanced(CLEAR Blaze_ROOT)
  message(FATAL_ERROR "Chrono::Multicore requires Blaze, but Blaze was not found; set Blaze_ROOT to the Blaze installation directory")
endif()

mark_as_advanced(CLEAR CH_USE_MULTICORE_DOUBLE)

# ------------------------------------------------------------------------------
# Additional dependencies, specific to this module
# ------------------------------------------------------------------------------
# ----- Double precision support -----

OPTION(CH_USE_MULTICORE_DOUBLE "Compile Chrono::Multicore with double precision math" ON)

if(CH_USE_MULTICORE_DOUBLE)
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

set(Chrono_Multicore_BASE
    ChApiMulticore.h
    ChMulticoreDefines.h
    ChSettings.h
    ChMeasures.h
    ChDataManager.h
    ChTimerMulticore.h
    ChDataManager.cpp
    )

source_group("" FILES ${Chrono_Multicore_BASE})

set(Chrono_Multicore_PHYSICS
    physics/ChSystemMulticore.h
    physics/ChSystemMulticore.cpp
    physics/ChSystemMulticoreNSC.cpp
    physics/ChSystemMulticoreSMC.cpp
    physics/Ch3DOFContainer.h
    physics/Ch3DOFContainer.cpp
    physics/ChFluidKernels.h
    physics/ChFluidContainer.cpp
    physics/ChParticleContainer.cpp
    )

source_group(physics FILES ${Chrono_Multicore_PHYSICS})
    
set(Chrono_Multicore_SOLVER
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

source_group(solver FILES ${Chrono_Multicore_SOLVER})

set(Chrono_Multicore_CONSTRAINTS
    constraints/ChConstraintRigidRigid.cpp
    constraints/ChConstraintRigidRigid.h
    constraints/ChConstraintBilateral.cpp
    constraints/ChConstraintBilateral.h
    constraints/ChConstraintUtils.cpp
    constraints/ChConstraintUtils.h
    )

source_group(constraints FILES ${Chrono_Multicore_CONSTRAINTS})

set(Chrono_Multicore_COLLISION
    collision/ChCollisionSystemChronoMulticore.h
    collision/ChCollisionSystemChronoMulticore.cpp
    collision/ChContactContainerMulticore.h
    collision/ChContactContainerMulticore.cpp
    collision/ChContactContainerMulticoreNSC.h
    collision/ChContactContainerMulticoreNSC.cpp
    collision/ChContactContainerMulticoreSMC.h
    collision/ChContactContainerMulticoreSMC.cpp
    )

source_group(collision FILES ${Chrono_Multicore_COLLISION})

# ------------------------------------------------------------------------------
# Add the Chrono_multicore library
# ------------------------------------------------------------------------------

add_library(Chrono_multicore
  ${Chrono_Multicore_BASE}
  ${Chrono_Multicore_PHYSICS}
  ${Chrono_Multicore_COLLISION}
  ${Chrono_Multicore_CONSTRAINTS}
  ${Chrono_Multicore_SOLVER}
)
add_library(Chrono::multicore ALIAS Chrono_multicore)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_multicore PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_multicore PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_multicore PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  # for Blaze, unary minus operator applied to unsigned type, result still unsigned
  target_compile_options(Chrono_multicore PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4146>)
  set_target_properties(Chrono_multicore PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_multicore PRIVATE Chrono_core)
if(CH_ENABLE_OPENMP)
  target_link_libraries(Chrono_multicore PRIVATE OpenMP::OpenMP_CXX)
elseif(CH_ENABLE_TBB)
  target_link_libraries(Chrono_multicore PRIVATE ${TBB_LIBRARIES})
endif()
target_link_libraries(Chrono_multicore PUBLIC Thrust::Thrust)
target_link_libraries(Chrono_multicore PUBLIC Blaze::Blaze)
if(BOOST_REQUIRED)
  target_include_directories(Chrono_multicore PRIVATE "${Boost_INCLUDE_DIRS}")
endif()

target_compile_definitions(Chrono_multicore PRIVATE CH_API_COMPILE_MULTICORE)
target_compile_definitions(Chrono_multicore PRIVATE BT_THREADSAFE)

# ----- Configure Blaze and Thrust -----
## TODO: DARIOM shall they be guarded for only CXX?
if(CH_ENABLE_OPENMP)
  target_compile_definitions(Chrono_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP")
  target_compile_definitions(Chrono_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP")
elseif(CH_ENABLE_TBB)
  target_compile_definitions(Chrono_multicore PUBLIC "BLAZE_USE_CPP_THREADS")
  target_compile_definitions(Chrono_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_TBB")
  target_compile_definitions(Chrono_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_TBB")
else()
  target_compile_definitions(Chrono_multicore PUBLIC "BLAZE_USE_CPP_THREADS")
  target_compile_definitions(Chrono_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CPP")
  target_compile_definitions(Chrono_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP")
endif()

install(TARGETS Chrono_multicore
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_multicore)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_multicore
        FILES_MATCHING PATTERN "*.h")

# Install files for chrono-config (in both build and install trees)
file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindBlaze.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindBlaze.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})