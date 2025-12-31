#===============================================================================
# CMake configuration file for the Chrono_Multicore library
#===============================================================================

option(CH_ENABLE_MODULE_MULTICORE "Enable the Chrono Multicore module" OFF)

# Return now if this module is not enabled

if(NOT CH_ENABLE_MODULE_MULTICORE)
  mark_as_advanced(FORCE blaze_INCLUDE_DIR)
  mark_as_advanced(FORCE blaze_DIR)
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
if(blaze_FOUND)
  message(STATUS "Blaze include dir: ${blaze_INCLUDE_DIR}")
  mark_as_advanced(FORCE blaze_INCLUDE_DIR)
  mark_as_advanced(FORCE blaze_DIR)
else()
  message("ERROR: Blaze cannot be found. Set either blaze_INCLUDE_DIR to the location of the Blaze headers or provide location of blaze-config.cmake through blaze_DIR.")
  set(blaze_INCLUDE_DIR "" CACHE PATH "Path to Blaze directory (should contain a subfolder named 'blaze'")
  set(blaze_DIR "" CACHE PATH "Path to Blaze config file (should contain the 'blaze-config.cmake' file")
  mark_as_advanced(CLEAR blaze_INCLUDE_DIR)
  mark_as_advanced(CLEAR blaze_DIR)
  return()
endif()

set(blaze_INCLUDE_DIR ${blaze_INCLUDE_DIR} PARENT_SCOPE)

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

set(Chrono_Multicore_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono_multicore/ChConfigMulticore.h)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigMulticore.h.in ${Chrono_Multicore_CONFIG_FILE} @ONLY)

source_group("" FILES ${Chrono_Multicore_CONFIG_FILE})

install(FILES ${Chrono_Multicore_CONFIG_FILE} DESTINATION include/chrono_multicore)

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
  ${Chrono_Multicore_CONFIG_FILE}
)
add_library(Chrono::multicore ALIAS Chrono_multicore)

set_target_properties(Chrono_multicore PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

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
target_link_libraries(Chrono_multicore PRIVATE OpenMP::OpenMP_CXX)
target_link_libraries(Chrono_multicore PUBLIC Thrust::Thrust)
target_link_libraries(Chrono_multicore PUBLIC blaze::blaze)
if(BOOST_REQUIRED)
  target_include_directories(Chrono_multicore PRIVATE "${Boost_INCLUDE_DIRS}")
endif()

target_compile_definitions(Chrono_multicore PRIVATE CH_API_COMPILE_MULTICORE)
target_compile_definitions(Chrono_multicore PRIVATE BT_THREADSAFE)

# ----- Configure Blaze and Thrust -----
target_compile_definitions(Chrono_multicore PUBLIC "THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP")
target_compile_definitions(Chrono_multicore PUBLIC "THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP")

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