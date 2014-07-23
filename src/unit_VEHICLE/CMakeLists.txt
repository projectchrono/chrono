#=============================================================================
# CMake configuration file for Chrono-T
#=============================================================================

cmake_minimum_required(VERSION 2.8)

project(Chrono-T)

# Location of additional CMAKE modules
#SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ------------------------------------------------------------------------------
# Required Chrono library and headers
# ------------------------------------------------------------------------------
SET(CHRONO_INC       ""  CACHE PATH "Location of the Chrono headers")
SET(CHRONO_LIB_PATH  ""  CACHE PATH "Location of the Chrono library")

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
OPTION(ENABLE_IRRLICHT "Enable Irrlich support in Chrono" OFF)

# ------------------------------------------------------------------------------
# Specify output locations for libraries and executables
# ------------------------------------------------------------------------------
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

IF (MSVC OR XCODE_VERSION)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ENDIF()

# ------------------------------------------------------------------------------
# Compiler settings
# ------------------------------------------------------------------------------

# set the compiler as a variable
IF(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    IF(CMAKE_SIZEOF_VOID_P MATCHES 4)
        SET (COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    ELSE()
        SET (COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    ENDIF()
ENDIF()

IF(MSVC AND CMAKE_CL_64)
    SET (COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
ELSEIF(MSVC)
    SET (COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
ENDIF()

IF(MSVC)
    # Avoid deprecation warnings
    ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    
    # Specify a single Build type, so we can import the correct .lib for Chrono
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "For single-configuration generators (e.g. make) set the type of build: Release, Debug, RelWithDebInfo, MinSizeRel")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")

    # Set compiler flags based on which compiler we're using
    IF("${COMPILER}" STREQUAL "COMPILER_MSVC")
        SET(CH_BUILDFLAGS "-DWIN32; -DNOMINMAX; -MP")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        SET(CH_BUILDFLAGS "-DWIN64; -D_WIN64; -DNOMINMAX; -MP")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_GCC")
        SET(CH_BUILDFLAGS "-DWIN32 -D_MINGW -D_WINDOWS")
    ELSEIF("${COMPILER}" STREQUAL "COMPILER_GCC_X64")
        SET(CH_BUILDFLAGS "-DWIN64 -D_MINGW -D_WINDOWS -m64")
    ENDIF()
ENDIF()

# ------------------------------------------------------------------------------
# Find the Chrono library
# ------------------------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Hack to be able to link different libraries depending on build type
    IF(CHRONO_LIB)
        UNSET(CHRONO_LIB CACHE)
    ENDIF()

    # Find the library based on the build type
    FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH}/${CMAKE_BUILD_TYPE}          REQUIRED)

    # Copy the DLL from/to appropriate place, based on build type
    SET(CHRONO_DLL "${CHRONO_LIB_PATH}/../bin/${CMAKE_BUILD_TYPE}/ChronoEngine.dll")

    FILE(COPY ${CHRONO_DLL} DESTINATION "${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}")

    # Report what library we use and which DLL was copied
    MESSAGE(STATUS "Found chrono library:    ${CHRONO_LIB}")
    MESSAGE(STATUS "Copied chrono DLL:       ${CHRONO_DLL}")
ELSE()
    FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine  PATHS ${CHRONO_LIB_PATH}          REQUIRED)
ENDIF()

# ------------------------------------------------------------------------------
# Add paths to required Chrono headers
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(
    ${CHRONO_INC}
    "${CHRONO_INC}/collision"
    "${CHRONO_INC}/collision/bullet"
    "${CHRONO_INC}/collision/bullet/LinearMath"
    "${CHRONO_INC}/collision/bullet/BulletCollision/NarrowPhaseCollision"
    )

# ------------------------------------------------------------------------------
# Add paths to the top of the source directory and the binary directory
# ------------------------------------------------------------------------------
INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

# ------------------------------------------------------------------------------
# Irrlicht support
# ------------------------------------------------------------------------------
IF(ENABLE_IRRLICHT)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        SET(CH_IRRLICHTDIR "C:/irrlicht-1.8"  CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")
        
        IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-visualstudio")
        ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-visualStudio")
        ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win32-gcc")
        ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
            FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "${CH_IRRLICHTDIR}/lib/Win64-gcc")
        ENDIF()
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        find_path(CH_IRRLICHTDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
        FIND_LIBRARY(CH_IRRLICHTLIB NAMES Irrlicht PATHS  ${CH_IRRLICHTDIR}/lib/Linux)
        SET (CH_IRRLICHTLIB "${CH_IRRLICHTLIB}" -lXxf86vm -lglut -lX11 -lGL)
    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        find_path(CH_IRRLICHTDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
        FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "/usr/local/lib" ${CH_IRRLICHTDIR}/lib/Linux)
        SET(CH_IRRLICHTLIB "${CH_IRRLICHTLIB}")
    ENDIF()

    IF(EXISTS "${CH_IRRLICHTDIR}/include")
        SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}/include")
    ELSE()
        SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}")
    ENDIF()

    INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()

# ------------------------------------------------------------------------------
# Add subdirectories
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(utils)
ADD_SUBDIRECTORY(models)
