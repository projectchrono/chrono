#=============================================================================
# CMake configuration file for Chrono FSI module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_FSI "Enable the Chrono FSI module" OFF)

IF(NOT ENABLE_MODULE_FSI)
  #mark_as_advanced(FORCE USE_FSI_DOUBLE)
  return()
ENDIF()

message(STATUS "\n==== Chrono FSI module ====\n")

#mark_as_advanced(CLEAR USE_FSI_DOUBLE)

# ------------------------------------------------------------------------------
# Additional options for this module
# ------------------------------------------------------------------------------

option(FSI_USE_DOUBLE "Compile Chrono::FSI with double precision math" ON)
if(FSI_USE_DOUBLE)
  set(CHRONO_FSI_USE_DOUBLE "#define CHRONO_FSI_USE_DOUBLE")
endif()

# ------------------------------------------------------------------------------
# Include directories, additional libraries, and linker flags
# ------------------------------------------------------------------------------

set(CH_FSI_INCLUDES "")
set(CH_FSI_LINKED_LIBRARIES ChronoEngine)
set(CH_FSI_LINKER_FLAGS "${CH_LINKERFLAG_LIB}")

# Add optional run-time visualization support

if(ENABLE_MODULE_OPENGL)
  include_directories(${CH_OPENGL_INCLUDES})
  set(CH_FSI_INCLUDES ${CH_FSI_INCLUDES} ${CH_OPENGL_INCLUDES})
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(CH_FSI_LINKER_FLAGS  "${CH_FSI_LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
  list(APPEND CH_FSI_LINKED_LIBRARIES ChronoEngine_opengl)
endif()

if(ENABLE_MODULE_VSG)
  include_directories(${CH_VSG_INCLUDES})
  list(APPEND CH_FSI_LINKED_LIBRARIES ChronoEngine_vsg)
endif()

# ------------------------------------------------------------------------------
# If using MSVC, disable warnings related to missing DLL interface
# ------------------------------------------------------------------------------

if(MSVC)
  #add_compile_options(/wd4251)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4251")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /wd4251")
endif()

# ------------------------------------------------------------------------------
# Hack to deal with MSVC runtime libraries
# ------------------------------------------------------------------------------

if(MSVC)
  if(USE_MSVC_STATIC_RUNTIME)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
  endif()
endif()

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

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory
# ------------------------------------------------------------------------------

set(CH_FSI_INCLUDES "${CH_FSI_INCLUDES}" PARENT_SCOPE)
