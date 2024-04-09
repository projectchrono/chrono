cmake_minimum_required(VERSION 3.21) # fmu_tools requires a more recent version of CMake
cmake_policy(SET CMP0091 NEW)

project(template_project_fmu)

# ===========
# Find Chrono
# ===========

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../chrono/lib")

find_package(Chrono
             COMPONENTS FMI                 # required Chrono module
             OPTIONAL_COMPONENTS Irrlicht   # optional Chrono module
             CONFIG)

if (NOT Chrono_FOUND)
  message(FATAL_ERROR "Could not find Chrono or one of its required modules")
  return()
endif()

if (NOT CHRONO_STATIC)
  message(NOTICE "It is highly recommended to compile Chrono in static mode as suggested by FMI reference.")
endif()

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})
include_directories(${CHRONO_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()


# ===============
# Fetch fmu_tools
# ===============

message(STATUS "\nFetch fmu_tools...")
include(FetchContent)

set(FMU_MODEL_IDENTIFIER "FmuComponentChrono" CACHE STRING "Set the FMU model identifier (name).")
set(FMU_RESOURCES_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/fmu_resources" CACHE PATH "Path to FMU resources directory.")
set(BUILD_TESTING ON CACHE BOOL "Enable testing.")
set(BUILD_FMU_HOST ON CACHE BOOL "Build fmi_host executable to run FMUs.")
set(FMU_MSG_PREFIX "   ")

FetchContent_Declare(
    ${FMU_MODEL_IDENTIFIER}
    SOURCE_DIR ${CHRONO_FMU_TOOLS_DIR}/fmi2
)

FetchContent_MakeAvailable(${FMU_MODEL_IDENTIFIER})

##message(STATUS "FMI_PLATFORM: ${FMI_PLATFORM}")
##message(STATUS "FMU_RUNTIME_OUTPUT_DIRECTORY: ${FMU_RUNTIME_OUTPUT_DIRECTORY}")

MARK_AS_ADVANCED(FETCHCONTENT_BASE_DIR)
MARK_AS_ADVANCED(FETCHCONTENT_FULLY_DISCONNECTED)
MARK_AS_ADVANCED(FETCHCONTENT_QUIET)
MARK_AS_ADVANCED(FETCHCONTENT_SOURCE_DIR_${FMU_MODEL_IDENTIFIER})
MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED)
MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED_${FMU_MODEL_IDENTIFIER})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${FMU_RUNTIME_OUTPUT_DIRECTORY})

# =======================================
# Add command to copy DLLs (windows only)
# =======================================

message(STATUS "")
add_DLL_copy_command()

# =========
# Build FMU
# =========

message(STATUS "\nConfigure FMU...")

# Append FMU sources toi automatically generated target
target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE FmuComponentChrono.h FmuComponentChrono.cpp)

target_compile_definitions(${FMU_MODEL_IDENTIFIER} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(${FMU_MODEL_IDENTIFIER} PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(${FMU_MODEL_IDENTIFIER} PUBLIC ${CHRONO_LINKER_FLAGS})
target_link_libraries(${FMU_MODEL_IDENTIFIER} ${CHRONO_LIBRARIES})

add_dependencies(${FMU_MODEL_IDENTIFIER} COPY_DLLS)

# Enable testing
if(TESTING_ENABLED)
  enable_testing()
endif()
