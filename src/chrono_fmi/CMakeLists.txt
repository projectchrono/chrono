#=============================================================================
# CMake configuration file for the Chrono::FMI module
#=============================================================================

option(ENABLE_MODULE_FMI "Enable the Chrono::FMI module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_FMI)
    mark_as_advanced(FORCE FMU_EXPORT_SUPPORT)
    mark_as_advanced(FORCE FMU_TOOLS_DIR)
    mark_as_advanced(FORCE FMU_TESTING)
    mark_as_advanced(FORCE FMU_GUID_NAMESPACE)
	return()
else()
    mark_as_advanced(CLEAR FMU_EXPORT_SUPPORT)
    mark_as_advanced(CLEAR FMU_TOOLS_DIR)
    mark_as_advanced(CLEAR FMU_TESTING)
    mark_as_advanced(CLEAR FMU_GUID_NAMESPACE)
endif()

MESSAGE(STATUS "\n==== Chrono FMI module ====\n")

cmake_dependent_option(FMU_EXPORT_SUPPORT "Enable FMU export capabilities" ON "ENABLE_MODULE_FMI" OFF)

# -------------------------------------------------

if(FMU_EXPORT_SUPPORT AND BUILD_SHARED_LIBS)
  message(NOTICE "For FMI export support, Chrono must be configured and built in static mode. Disable 'BUILD_SHARED_LIBS'.")
  message(NOTICE "FMU export capabilities in Chrono::FMI module disabled.")
  set(FMU_EXPORT_SUPPORT OFF CACHE BOOL "Enable FMU export capabilities" FORCE)
endif()

if(MSVC)
  if (FMU_EXPORT_SUPPORT AND NOT USE_MSVC_STATIC_RUNTIME)
    message(NOTICE "For FMI export support on Windows, Chrono must be configured and built with a statically-linked runtime library. Enable 'USE_MSVC_STATIC_RUNTIME'.")
    message(NOTICE "FMU export capabilities in Chrono::FMI module disabled.")
    set(FMU_EXPORT_SUPPORT OFF CACHE BOOL "Enable FMU export capabilities" FORCE)
  endif()
endif()

# -------------------------------------------------
# Allow user to point to a different location of fmu_tools.
# FMU_TOOLS_DIR must point to the *top-level* directory of the fmu_tools distribution.
# Default: Chrono submodule in src/chrono_thirdparty/

set(FMU_TOOLS_DIR "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/fmu_tools" CACHE PATH "Path to top-level fmu_tools directory")

include_directories(${FMU_TOOLS_DIR})

# -------------------------------------------------
# Enable testing of generated FMUs

cmake_dependent_option(FMU_TESTING "Enable FMU testing" ON "ENABLE_FMU_EXPORT" OFF)

# -------------------------------------------------
# Set Architecture and Operating System Flags
# -------------------------------------------------
set(UNKNOWN_TARGET_SYSTEM FALSE)
if(WIN32)
    set(FMI_PLATFORM "win")
elseif(APPLE)
    set(FMI_PLATFORM "darwin")
elseif(CYGWIN)
    set(FMI_PLATFORM "cygwin")
elseif(UNIX)
    set(FMI_PLATFORM "linux")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(FMI_PLATFORM "${FMI_PLATFORM}64")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(FMI_PLATFORM "${FMI_PLATFORM}32")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(UNKNOWN_TARGET_SYSTEM)
    message(WARNING "Cannot retrieve target operating system or architecture. Please specify it manually through TARGET_SYSTEM variable (e.g. win64)")
    set(TARGET_SYSTEM "win64" CACHE STRING "Target operating system and architecture (e.g. win64, linux32, ...)")
    mark_as_advanced(CLEAR TARGET_SYSTEM)
else()
    mark_as_advanced(FORCE TARGET_SYSTEM)
endif()

set(FMI_PLATFORM "${FMI_PLATFORM}" PARENT_SCOPE)

message(STATUS "FMI target version:   ${FMI_VERSION}")
message(STATUS "FMI platform:         ${FMI_PLATFORM}")
message(STATUS "FMU Tools directory:  ${FMU_TOOLS_DIR}")

if(FMU_EXPORT_SUPPORT)
  include(FetchContent)
endif()

#=============================================================================
# Generate and install the Chrono::FMI configuration header

if(FMU_EXPORT_SUPPORT)
  set(DEFINE_FMU_EXPORT_SUPPORT "#define FMU_EXPORT_SUPPORT")
else()
  set(DEFINE_FMU_EXPORT_SUPPORT "#undef FMU_EXPORT_SUPPORT")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFMI.h.in
               ${PROJECT_BINARY_DIR}/chrono_fmi/ChConfigFMI.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_fmi/ChConfigFMI.h"
        DESTINATION include/chrono_fmi)

#=============================================================================
# Create the Chrono::FMI library

set(ChronoEngine_FMI_SOURCES 
    fmi2/ChExternalFmu.cpp
)

set(ChronoEngine_FMI_HEADERS
    ChApiFMI.h 
    fmi2/ChFmuToolsImport.h
    fmi2/ChExternalFmu.h
)

source_group("" FILES 
            ${ChronoEngine_FMI_HEADERS} 
            ${ChronoEngine_FMI_SOURCES})

#-----------------------------------------------------------------------------

add_library(ChronoEngine_fmi
            ${ChronoEngine_FMI_HEADERS}
            ${ChronoEngine_FMI_SOURCES})

set_target_properties(ChronoEngine_fmi PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")

target_include_directories(ChronoEngine_fmi PUBLIC ${FMU_TOOLS_DIR})

target_compile_definitions(ChronoEngine_fmi PRIVATE "CH_API_COMPILE_FMI")
target_compile_definitions(ChronoEngine_fmi PRIVATE "CH_IGNORE_DEPRECATED")

target_compile_definitions(ChronoEngine_fmi PUBLIC FMU_OS_SUFFIX="${FMI_PLATFORM}")
target_compile_definitions(ChronoEngine_fmi PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")

target_link_libraries(ChronoEngine_fmi ChronoEngine)

add_dependencies(ChronoEngine_fmi ChronoEngine)

install(TARGETS ChronoEngine_fmi
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)        

install(FILES ${ChronoEngine_FMI_HEADERS}
        DESTINATION include/chrono_fmi)
