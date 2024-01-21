cmake_minimum_required(VERSION 3.18)


project(template_project_multifmu)

set(FMU_TOOLS_DIR "${CMAKE_SOURCE_DIR}/../../fmu_tools/" CACHE PATH "Path to fmu_tools.")
set(BUILD_TESTING ON CACHE BOOL "Enable testing.")

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

# let Chrono COPY_DLLS to copy all the required files in a intermediate directory to be later copied into FMU-specific folders
set(CHRONO_INTERMEDIATE_DLLS_DIR "${CMAKE_BINARY_DIR}/chrono_dlls")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CHRONO_INTERMEDIATE_DLLS_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CHRONO_INTERMEDIATE_DLLS_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CHRONO_INTERMEDIATE_DLLS_DIR}")
add_DLL_copy_command()


if (BUILD_TESTING)
  enable_testing()
endif()

###############################################
# Copy required DLLs into FMU-specific folders
###############################################

# all subprojects should add their FMU_RUNTIME_OUTPUT_DIRECTORY to FMU_RUNTIME_OUTPUT_DIRECTORIES
set(FMU_RUNTIME_OUTPUT_DIRECTORIES)
include(FetchContent)

add_subdirectory(FmuComponentChrono1)
add_subdirectory(FmuComponentChrono2)


###############################################
# Copy required DLLs into FMU-specific folders
###############################################

add_custom_target(COPY_DLLS_FMU)
add_dependencies(COPY_DLLS_FMU COPY_DLLS)

# TODO: copy_directory_if_different is available since 3.26
foreach(FMU_RUNTIME_OUTPUT_DIR ${FMU_RUNTIME_OUTPUT_DIRECTORIES})
  add_custom_command(
    TARGET COPY_DLLS_FMU PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CHRONO_INTERMEDIATE_DLLS_DIR} ${FMU_RUNTIME_OUTPUT_DIR}
  )
endforeach()
