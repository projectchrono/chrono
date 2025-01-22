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

target_link_libraries(ChronoEngine_mumps
                      ChronoEngine
                      MUMPS::MUMPS)

#-------------------------------------------------------------------------------
# Install the ChronoEngine_mumps library

install(TARGETS ChronoEngine_mumps
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_MUMPS_HEADERS} 
        DESTINATION include/chrono_mumps)