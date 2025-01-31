#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

if(NOT ENABLE_MODULE_FSI)
  #mark_as_advanced(FORCE FSI_USE_DOUBLE)
  return()
endif()

message(STATUS "\n==== Chrono FSI module ====\n")

# Additional options for this module

option(FSI_USE_DOUBLE "Compile Chrono::FSI with double precision math" ON)
if(FSI_USE_DOUBLE)
  set(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
endif()

# Configure submodules

add_subdirectory(sph)

# Generate and install configuration file

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFsi.h.in
               ${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFsi.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_fsi/ChConfigFsi.h"
        DESTINATION include/chrono_fsi)

