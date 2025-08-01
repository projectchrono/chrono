#=============================================================================
#
# CMake configuration file for the Chrono Project
#
#=============================================================================

#-----------------------------------------------------------------------------
# Check backward compatibility
#-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.18)
# Features achievable with later versions of CMake:
# - install(IMPORTED_RUNTIME_ARTIFACTS) CMake 3.21

#-----------------------------------------------------------------------------
# Prevent in-source builds.
#-----------------------------------------------------------------------------

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR  "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

#-----------------------------------------------------------------------------

# Be sure to avoid troubles with library paths when using old policy
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0048 NEW)
    cmake_policy(SET CMP0091 NEW)
endif(COMMAND cmake_policy)

#-----------------------------------------------------------------------------
# Disable solution folders because not supported in VC Express
#-----------------------------------------------------------------------------

set_property(GLOBAL PROPERTY USE_FOLDERS Off)

#-----------------------------------------------------------------------------
# Define project name and specify version information
#-----------------------------------------------------------------------------

set(CHRONO_VERSION_MAJOR 9)
set(CHRONO_VERSION_MINOR 0)
set(CHRONO_VERSION_PATCH 1)

project(Chrono VERSION ${CHRONO_VERSION_MAJOR}.${CHRONO_VERSION_MINOR}.${CHRONO_VERSION_PATCH})

set(CHRONO_VERSION "${CHRONO_VERSION_MAJOR}.${CHRONO_VERSION_MINOR}.${CHRONO_VERSION_PATCH}")

set(CPACK_PACKAGE_VERSION "${CHRONO_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${CHRONO_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${CHRONO_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${CHRONO_VERSION_PATCH}")

# Version as a hexadecimal integer of the form 0x00MMmmPP
set(CH_VERSION 0x00090001)

#-----------------------------------------------------------------------------
# Detect operating system and processor for host and target
#-----------------------------------------------------------------------------
message(STATUS "We are on a ${CMAKE_HOST_SYSTEM_NAME} system")
message(STATUS "The host processor is ${CMAKE_HOST_SYSTEM_PROCESSOR}")
message(STATUS "Building for a ${CMAKE_SYSTEM_NAME} system")
message(STATUS "The target processor is ${CMAKE_SYSTEM_PROCESSOR}")

# Get rid of cmake RPATH warning on OSX
if(APPLE)
    set(CMAKE_MACOSX_RPATH OFF)
endif()

#-----------------------------------------------------------------------------
# Location of additional CMake scripts
#-----------------------------------------------------------------------------

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC VCS_REVISION)

if("${VCS_REVISION}" MATCHES ".*-NOTFOUND$")
    set(VCS_REVISION "")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#-----------------------------------------------------------------------------
# Enable CTest
#-----------------------------------------------------------------------------

# Note: no need to explicitly call 'enable_testing' as this is invoked by
#       'include(CTest)'

option(BUILD_TESTING "Build unit tests" OFF)
include(CTest)

set(VALGRIND_COMMAND "/usr/bin/valgrind")
set(VALGRIND_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")
set(MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")

if (NOT WIN32)
	option(USE_CCACHE "Build using ccache compiler driver" OFF) 

	if (USE_CCACHE)
		find_program(CCACHE_EXE ccache)
		if(CCACHE_EXE)
	        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
	    endif()
	else()
		unset(CCACHE_EXE CACHE)
		set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE)
	endif()
endif()

#-----------------------------------------------------------------------------
# Use dependent options
#-----------------------------------------------------------------------------

include(CMakeDependentOption)

#-----------------------------------------------------------------------------
# Also, some variables that were used in previous makefile system (some
# of them could be set automatically because CMake should recognize automatically
# the compiler, generator, platform, etc....
# Set a default build type if none was specified
#-----------------------------------------------------------------------------

if(MSVC OR XCODE_VERSION)
    set(CMAKE_CONFIGURATION_TYPES Release CACHE STRING "Choose the type of build.")
    set_property(CACHE CMAKE_CONFIGURATION_TYPES PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    mark_as_advanced(FORCE CMAKE_BUILD_TYPE)
    mark_as_advanced(CLEAR CMAKE_CONFIGURATION_TYPES)
    set(CPACK_BUILD_TYPE ${CMAKE_CONFIGURATION_TYPES})
else()
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CPACK_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()

if(XCODE_VERSION)
    set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvmgcc42")
endif()

#-----------------------------------------------------------------------------

set(CH_COMPILER " " CACHE STRING "Compiler Type" FORCE)

if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    if(CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(CH_COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    else()
        set(CH_COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    endif()
endif()

if(MSVC AND CMAKE_CL_64)
    set(CH_COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
elseif(MSVC)
    set(CH_COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
endif()

mark_as_advanced(FORCE CH_COMPILER)

#-----------------------------------------------------------------------------
# Specify locations of executables and libraries
#-----------------------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

#-----------------------------------------------------------------------------

set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(CPACK_SYSTEM_NAME "win32-${CMAKE_SYSTEM_PROCESSOR}")
    else()
        set(CPACK_SYSTEM_NAME "win64-${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()

#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    execute_process(COMMAND "date" "+%Y_%m_%d" OUTPUT_VARIABLE DATE)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    execute_process(COMMAND "date" "+%Y_%m_%d" OUTPUT_VARIABLE DATE)
endif()

string(REGEX REPLACE "(\r?\n)+$" "" DATE "${DATE}")

#set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_GENERATOR "ZIP")
#set(CPACK_INSTALL_CMAKE_PROJECTS "${PROJECT_BINARY_DIR};Chrono;ALL;/")
#set(CPACK_NSIS_DISPLAY_NAME "Chrono")
#set(CPACK_PACKAGE_DESCRIPTION_FILE "/home/andy/vtk/CMake/Copyright.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Chrono is a multibody-dynamics package")
#set(CPACK_PACKAGE_EXECUTABLES "ccmake;CMake")
set(CPACK_PACKAGE_FILE_NAME "Chrono-${CPACK_SYSTEM_NAME}-${CPACK_BUILD_TYPE}-${DATE}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Chrono")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "Chrono")
set(CPACK_PACKAGE_NAME "Chrono")
set(CPACK_PACKAGE_VENDOR "UWSBEL")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
#set(CPACK_RESOURCE_FILE_WELCOME "/home/andy/vtk/CMake/Templates/CPack.GenericWelcome.txt")
set(CPACK_SOURCE_GENERATOR "TGZ")
#set(CPACK_SOURCE_PACKAGE_FILE_NAME "Chrono")
set(CPACK_SOURCE_STRIP_FILES "")
#set(CPACK_STRIP_FILES "bin/ccmake;bin/cmake;bin/cpack;bin/ctest")
#set(CPACK_TOPLEVEL_TAG "Linux-x64")

#------------------------------------------------------------
# Install destinations for data and demo programs
#------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CH_INSTALL_DATA "data")
    set(CH_INSTALL_DEMO "bin")
    set(CH_INSTALL_PYTHON "bin")
else()
    set(CH_INSTALL_DATA "share/chrono/data")
    set(CH_INSTALL_DEMO "share/chrono/bin")
    set(CH_INSTALL_PYTHON "share/chrono/python")
endif()

#------------------------------------------------------------
# Copy data directory to BUILD tree 
# Install data directory 
# Install sample template_project
# Install blender import script
#------------------------------------------------------------

get_property(MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(MULTI_CONFIG)
    set(CH_BUILD_DATA ${CMAKE_BINARY_DIR}/bin/data)
    file(COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
    message(STATUS "Using a multi-config generator (${CMAKE_GENERATOR})")
    message(STATUS "Data directory copied to: ${CMAKE_BINARY_DIR}/bin/data/")
    message(STATUS "Binaries will be created in ${CMAKE_BINARY_DIR}/bin/<CONFIGURATION>/")
else()
    set(CH_BUILD_DATA ${CMAKE_BINARY_DIR}/data)
    file(COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/data/)
    message(STATUS "Using a single configuration generator (${CMAKE_GENERATOR})")
    message(STATUS "Data directory copied to: ${CMAKE_BINARY_DIR}/data/")
    message(STATUS "Binaries will be created in ${CMAKE_BINARY_DIR}/bin/")
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CH_INSTALL_DATA})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project/ DESTINATION "template_project")
else()
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project/ DESTINATION "share/chrono/template_project")
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/importer_blender/ DESTINATION "importer_blender")

#------------------------------------------------------------
# Defer configuration of all Chrono libraries and programs
#------------------------------------------------------------

add_subdirectory(src)

#------------------------------------------------------------
# Use CPack
#------------------------------------------------------------

include(CPack)
