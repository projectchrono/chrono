#=============================================================================
# CMake configuration file for the Chrono MUMPS module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_MUMPS "Enable the Chrono MUMPS module" OFF)

if(NOT CH_ENABLE_MODULE_MUMPS)
    return()
endif()

message(STATUS "\n==== Chrono Engine Mumps module ====\n")

# ------------------------------------------------------------------------------
# Find MUMPS (required)

message(STATUS "Enabling Fortran language")
check_language(Fortran)
if(NOT CMAKE_Fortran_COMPILER)
  message("  MUMPS requires a Fortran compiler, but none could be found. Chrono::Mumps module disabled.")
  set(CH_ENABLE_MODULE_MUMPS OFF CACHE BOOL "Enable the Chrono MUMPS module" FORCE)
  return()
endif()

enable_language(Fortran)
message(STATUS "  Fortran compiler: ${CMAKE_Fortran_COMPILER}")

message(STATUS "Searching for MUMPS")
find_package(MUMPS QUIET REQUIRED CONFIG)

if(${MUMPS_FOUND})
  message(STATUS "  MUMPS found.")
else()
  message("  MUMPS not found. Chrono::Mumps module disabled.")
  set(CH_ENABLE_MODULE_MUMPS OFF CACHE BOOL "Enable the Chrono MUMPS module" FORCE)
  return()
endif()

# ------------------------------------------------------------------------------
# List all files in the Chrono mumps module

set(Chrono_MUMPS_HEADERS
  ChApiMumps.h
  ChSolverMumps.h
  ChMumpsEngine.h
)

set(Chrono_MUMPS_SOURCES
  ChSolverMumps.cpp
  ChMumpsEngine.cpp
)

source_group("" FILES ${Chrono_MUMPS_HEADERS} ${Chrono_MUMPS_SOURCES})

# ------------------------------------------------------------------------------
# Add the Chrono_mumps library


add_library(Chrono_mumps 
            ${Chrono_MUMPS_SOURCES}
            ${Chrono_MUMPS_HEADERS})
add_library(Chrono::mumps ALIAS Chrono_mumps)


set_target_properties(Chrono_mumps PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_mumps PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_mumps PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_mumps PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_mumps PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_mumps PRIVATE "CH_API_COMPILE_MUMPS")
target_compile_definitions(Chrono_mumps PRIVATE "CH_IGNORE_DEPRECATED")
target_compile_definitions(Chrono_mumps PRIVATE "_OPENMP_NOFORCE_MANIFEST")


target_link_libraries(Chrono_mumps
                      Chrono_core
                      MUMPS::MUMPS)

install(TARGETS Chrono_mumps
  EXPORT ChronoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_mumps)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_MUMPS_HEADERS} 
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
