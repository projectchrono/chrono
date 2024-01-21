cmake_minimum_required(VERSION 3.18)


project(template_project_fmu)

set(FMU_TOOLS_DIR "${CMAKE_SOURCE_DIR}/../../fmu_tools/" CACHE PATH "Path to fmu_tools.")

#########################
# Importing Chrono
#########################

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../chrono/lib")
find_package(Chrono
             OPTIONAL_COMPONENTS Irrlicht
             CONFIG)

if (NOT Chrono_FOUND)
  message(FATAL_ERROR "Could not find Chrono or one of its required modules")
  return()
endif()

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

if (NOT CHRONO_STATIC)
  message(NOTICE "It is highly recommended to compile Chrono in static mode as suggested by FMI reference.")
endif()

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


#########################
# Fetching fmu_tools
#########################

MESSAGE(STATUS "-------------- FetchContent(fmu_tools) --------------")

set(FMU_MODEL_IDENTIFIER "FmuComponentChrono" CACHE STRING "FMU Model Identifier: should fulfill operating system and C-function naming standards.")
set(BUILD_TESTING ON CACHE BOOL "Enable testing.")
set(USE_EXTERNAL_FMU_COMPONENT_SOURCES ON CACHE INTERNAL "")

include(FetchContent)
FetchContent_Declare(
    ${FMU_MODEL_IDENTIFIER}
    SOURCE_DIR ${FMU_TOOLS_DIR}
)

FetchContent_MakeAvailable(${FMU_MODEL_IDENTIFIER})

# MESSAGE(STATUS "FMI_PLATFORM: ${FMI_PLATFORM}")
# MESSAGE(STATUS "FMU_RUNTIME_OUTPUT_DIRECTORY: ${FMU_RUNTIME_OUTPUT_DIRECTORY}")

MARK_AS_ADVANCED(FETCHCONTENT_BASE_DIR)
MARK_AS_ADVANCED(FETCHCONTENT_FULLY_DISCONNECTED)
MARK_AS_ADVANCED(FETCHCONTENT_QUIET)
MARK_AS_ADVANCED(FETCHCONTENT_SOURCE_DIR_${FMU_MODEL_IDENTIFIER})
MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED)
MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED_${FMU_MODEL_IDENTIFIER})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${FMU_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${FMU_RUNTIME_OUTPUT_DIRECTORY})
add_DLL_copy_command()

MESSAGE(STATUS "---------------------------------------------------------")

target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE FmuComponentChrono.h FmuComponentChrono.cpp) ## FMU_ACTION: put your source code here
target_compile_definitions(${FMU_MODEL_IDENTIFIER} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(${FMU_MODEL_IDENTIFIER} PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(${FMU_MODEL_IDENTIFIER} PUBLIC ${CHRONO_LINKER_FLAGS})
target_link_libraries(${FMU_MODEL_IDENTIFIER} ${CHRONO_LIBRARIES})


IF (NOT EXISTS "${CMAKE_SOURCE_DIR}/fmu_host.cpp")
  file(COPY_FILE "${FMU_TOOLS_DIR}/fmu_host.cpp" "${CMAKE_SOURCE_DIR}/fmu_host.cpp")
endif()
target_sources(fmu_host_${FMU_MODEL_IDENTIFIER} PRIVATE fmu_host.cpp)

add_dependencies(${FMU_MODEL_IDENTIFIER} COPY_DLLS)

# Enable testing
if(TESTING_ENABLED)
  enable_testing()
endif()
