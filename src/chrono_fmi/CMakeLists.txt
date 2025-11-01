#=============================================================================
# CMake configuration file for the Chrono::FMI module
#=============================================================================

option(CH_ENABLE_MODULE_FMI "Enable the Chrono::FMI module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_FMI)
    mark_as_advanced(FORCE FMU_EXPORT_SUPPORT)
    mark_as_advanced(FORCE FMU_FORGE_DIR)
    mark_as_advanced(FORCE FMU_TESTING)
    mark_as_advanced(FORCE FMU_GUID_NAMESPACE)
	return()
else()
    mark_as_advanced(CLEAR FMU_EXPORT_SUPPORT)
    mark_as_advanced(CLEAR FMU_FORGE_DIR)
    mark_as_advanced(CLEAR FMU_TESTING)
    mark_as_advanced(CLEAR FMU_GUID_NAMESPACE)
endif()

message(STATUS "\n==== Chrono FMI module ====\n")

cmake_dependent_option(FMU_EXPORT_SUPPORT "Enable FMU export capabilities" ON "CH_ENABLE_MODULE_FMI" OFF)

# -------------------------------------------------

if(FMU_EXPORT_SUPPORT AND BUILD_SHARED_LIBS)
  message(NOTICE "For FMI export support, Chrono must be configured and built in static mode. Disable 'BUILD_SHARED_LIBS'.")
  message(NOTICE "FMU export capabilities in Chrono::FMI module disabled.")
  set(FMU_EXPORT_SUPPORT OFF CACHE BOOL "Enable FMU export capabilities" FORCE)
endif()

if(MSVC)
  if (FMU_EXPORT_SUPPORT AND NOT CH_USE_MSVC_STATIC_RUNTIME)
    message(NOTICE "For FMI export support on Windows, Chrono must be configured and built with a statically-linked runtime library. Enable 'CH_USE_MSVC_STATIC_RUNTIME'.")
    message(NOTICE "FMU export capabilities in Chrono::FMI module disabled.")
    set(FMU_EXPORT_SUPPORT OFF CACHE BOOL "Enable FMU export capabilities" FORCE)
  endif()
endif()

# -------------------------------------------------
# Allow user to point to a different location of fmu-forge.
# FMU_FORGE_DIR must point to the *top-level* directory of the fmu-forge distribution.
# Default: Chrono submodule in src/chrono_thirdparty/

set(FMU_FORGE_DIR "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/fmu-forge" CACHE PATH "Path to top-level fmu-forge directory")

# -------------------------------------------------
# Enable testing of generated FMUs

cmake_dependent_option(FMU_TESTING "Enable FMU testing" ON "ENABLE_FMU_EXPORT" OFF)

# -------------------------------------------------
# Set Architecture and Operating System Flags
# -------------------------------------------------

set(UNKNOWN_TARGET_SYSTEM FALSE)

if(WIN32)
    set(FMI2_SYS "win")
    set(FMI3_SYS "windows")
elseif(APPLE)
    set(FMI2_SYS "darwin")
    set(FMI3_SYS "darwin")
elseif(UNIX)
    set(FMI2_SYS "linux")
    set(FMI3_SYS "linux")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(FMI2_ARCH "64")
    set(FMI3_ARCH "x86_64")
elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(FMI2_ARCH "32")
    set(FMI3_ARCH "x86")
else()
    set(UNKNOWN_TARGET_SYSTEM TRUE)
endif()

if(UNKNOWN_TARGET_SYSTEM)
    message(WARNING "Cannot retrieve target operating system or architecture. Please manually specify the FMI2_PLATFORM (e.g. win64) and the FMI3_PLATFORM (e.g. x86_64-windows)")
    set(FMI2_PLATFORM "win64" CACHE STRING "Target operating system and architecture for FMI 2.0 (e.g. win64, linux32, ...)")
    set(FMI3_PLATFORM "x86_64-windows" CACHE STRING "Target operating system and architecture for FMI 3.0 (e.g. x86_64-windows, x86-linux, ...)")
    mark_as_advanced(CLEAR FMI2_PLATFORM)
    mark_as_advanced(CLEAR FMI3_PLATFORM)
else()
    set(FMI2_PLATFORM ${FMI2_SYS}${FMI2_ARCH})
    set(FMI3_PLATFORM ${FMI3_ARCH}-${FMI3_SYS})
    mark_as_advanced(FORCE FMI2_PLATFORM)
    mark_as_advanced(FORCE FMI3_PLATFORM)
endif()

set(FMI2_PLATFORM "${FMI2_PLATFORM}" PARENT_SCOPE)
set(FMI3_PLATFORM "${FMI3_PLATFORM}" PARENT_SCOPE)

message(STATUS "FMI2 platform:        ${FMI2_PLATFORM}")
message(STATUS "FMI3 platform:        ${FMI3_PLATFORM}")
message(STATUS "fmu-forge directory:  ${FMU_FORGE_DIR}")
message(STATUS "FMU export:           ${FMU_EXPORT_SUPPORT}")

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

set(Chrono_FMI_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono_fmi/ChConfigFMI.h)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigFMI.h.in ${Chrono_FMI_CONFIG_FILE} @ONLY)

source_group("" FILES ${Chrono_FMI_CONFIG_FILE})

install(FILES ${Chrono_FMI_CONFIG_FILE} DESTINATION include/chrono_fmi)

#=============================================================================
# Create the Chrono::FMI library

set(Chrono_FMI_SOURCES 
    ChExternalFmu.cpp
)

set(Chrono_FMI_HEADERS
    ChApiFMI.h 
    ChExternalFmu.h
    fmi2/ChFmuToolsImport.h
    fmi3/ChFmuToolsImport.h
    fmi2/ChFmuToolsExport.h
    fmi3/ChFmuToolsExport.h
)

source_group("" FILES 
            ${Chrono_FMI_HEADERS} 
            ${Chrono_FMI_SOURCES})

#-----------------------------------------------------------------------------

add_library(Chrono_fmi
            ${Chrono_FMI_HEADERS}
            ${Chrono_FMI_SOURCES}
            ${Chrono_FMI_CONFIG_FILE}
)

add_library(Chrono::fmi ALIAS Chrono_fmi)

set_target_properties(Chrono_fmi PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

target_include_directories(Chrono_fmi PRIVATE "${FMU_FORGE_DIR}")

if (CH_STATIC)
  set_target_properties(Chrono_fmi PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_fmi PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_fmi PRIVATE "CH_API_COMPILE_FMI")
target_link_libraries(Chrono_fmi PUBLIC Chrono_core)

target_compile_definitions(Chrono_fmi PUBLIC FMI2_PLATFORM="${FMI2_PLATFORM}")
target_compile_definitions(Chrono_fmi PUBLIC FMI3_PLATFORM="${FMI3_PLATFORM}")
target_compile_definitions(Chrono_fmi PUBLIC SHARED_LIBRARY_SUFFIX="${CMAKE_SHARED_LIBRARY_SUFFIX}")


install(TARGETS Chrono_fmi
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_fmi)

# Old way
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_fmi
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY ${FMU_FORGE_DIR}
        DESTINATION include/chrono_thirdparty
        PATTERN ".git" EXCLUDE)

if(FMU_EXPORT_SUPPORT)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project_fmi2/ DESTINATION "template_project_fmi2")
  else()
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project_fmi2/ DESTINATION "share/chrono/template_project_fmi2")
  endif()
endif()
