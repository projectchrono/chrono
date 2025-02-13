#=============================================================================
# CMake configuration file for the Chrono MUMPS module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MUMPS "Enable the Chrono MUMPS module" OFF)

if(NOT ENABLE_MODULE_MUMPS)
    return()
endif()

message(STATUS "\n==== Chrono Engine Mumps module ====\n")

# ------------------------------------------------------------------------------
# Find MUMPS (required)

message(STATUS "Searching for MUMPS...")
find_package(MUMPS REQUIRED CONFIG)

if(${MUMPS_FOUND})
  message(STATUS "MUMPS found.")
else()
  message("MUMPS not found. Chrono::Mumps module disabled.")
  set(ENABLE_MODULE_MUMPS OFF CACHE BOOL "Enable the Chrono MUMPS module" FORCE)
  return()
endif()

enable_language(Fortran)
message(STATUS "  Fortran compiler: ${CMAKE_Fortran_COMPILER}")

# ------------------------------------------------------------------------------
# List all files in the Chrono mumps module

set(ChronoEngine_MUMPS_HEADERS
  ChApiMumps.h
  ChSolverMumps.h
  ChMumpsEngine.h
)

set(ChronoEngine_MUMPS_SOURCES
  ChSolverMumps.cpp
  ChMumpsEngine.cpp
)

source_group("" FILES ${ChronoEngine_MUMPS_HEADERS} ${ChronoEngine_MUMPS_SOURCES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_mumps library

add_definitions("-D_OPENMP_NOFORCE_MANIFEST")

add_library(ChronoEngine_mumps 
            ${ChronoEngine_MUMPS_SOURCES}
            ${ChronoEngine_MUMPS_HEADERS})

target_compile_definitions(ChronoEngine_mumps PRIVATE "CH_API_COMPILE_MUMPS")
target_compile_definitions(ChronoEngine_mumps PRIVATE "CH_IGNORE_DEPRECATED")

if(MSVC)
  set_target_properties(ChronoEngine_mumps PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(ChronoEngine_mumps
                      ChronoEngine
                      MUMPS::MUMPS)

install(TARGETS ChronoEngine_mumps
  EXPORT ChronoEngineTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_mumps)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${ChronoEngine_MUMPS_HEADERS} 
        DESTINATION include/chrono_mumps)

# On Windows, extract DLLs from targets and install (always look for Release DLLs)
# TODO (RADU)  This does not actually work for MUMPS because MUMPS::MUMPS is an "umbrella" target. Need to look at the actual MUMPS targets it contains.
#              OK for now because we usually build MUMPS as static libraries.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  get_target_property(MUMPS_TYPE MUMPS::MUMPS TYPE)
  if (NOT MUMPS_TYPE STREQUAL STATIC_LIBRARY)
    get_target_property(MUMPS_DLL MUMPS::MUMPS IMPORTED_LOCATION_RELEASE)
    if(EXISTS "${MUMPS_DLL}")
       install(FILES "${MUMPS_DLL}" DESTINATION bin)
    endif()
  endif()
endif()
