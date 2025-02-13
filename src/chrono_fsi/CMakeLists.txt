#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

if(NOT CH_ENABLE_MODULE_FSI)
  mark_as_advanced(FORCE CH_USE_FSI_DOUBLE)
  return()
else()
  mark_as_advanced(CLEAR CH_USE_FSI_DOUBLE)
endif()

message(STATUS "\n==== Chrono FSI module ====\n")

# Additional options for this module

option(CH_USE_FSI_DOUBLE "Compile Chrono::FSI with double precision math" OFF)
if(CH_USE_FSI_DOUBLE)
  set(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
endif()

# Configure submodules

add_subdirectory(sph)

# Generate and install configuration file

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFsi.h.in
               ${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFsi.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFsi.h"
        DESTINATION include/chrono_fsi)

