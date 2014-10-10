#=============================================================================
# CMake configuration file for Chrono-T
#
#=============================================================================

cmake_minimum_required(VERSION 2.8)

project(Chrono-T)

# Location of additional CMAKE modules
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ------------------------------------------------------------------------------
# Specify output locations for libraries and executables
# ------------------------------------------------------------------------------
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

IF (MSVC OR XCODE_VERSION)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ENDIF()

# ------------------------------------------------------------------------------
# Find the Chrono library
# ------------------------------------------------------------------------------

# TODO: Right now, we cannot check whether or not Chrono was built with Irrlicht
#       support.  This will require making unit_IRRLICHT a separate DLL.  
#       For now, we assume we know what we're asking for.
#       Later on, if Irrlicht is requested, but unit_IRRLICHT is not available,
#       we will warn the user and disable Irrlicht support in chrono-T

OPTION(ENABLE_IRRLICHT "Enable Irrlicht support" ON)

IF(ENABLE_IRRLICHT)
    FIND_PACKAGE(ChronoEngine COMPONENTS unit_POSTPROCESS unit_IRRLICHT)
ELSE()
    FIND_PACKAGE(ChronoEngine COMPONENTS unit_POSTPROCESS)
ENDIF()

OPTION(ENABLE_IRRKLANG "Use Irrklang library for sound" OFF)



MESSAGE(STATUS "Compiler: ${CH_COMPILER}")
#MESSAGE(STATUS "CH_BUILDFLAGS: ${CH_BUILDFLAGS}")
#MESSAGE(STATUS "CH_LINKERFLAG_EXE: ${CH_LINKERFLAG_EXE}")

#MESSAGE(STATUS "Chrono includes:  ${CHRONOENGINE_INCLUDES}")
#MESSAGE(STATUS "Chrono main lib:  ${CHRONOENGINE_LIBRARY}")
#MESSAGE(STATUS "Chrono libraries: ${CHRONOENGINE_LIBRARIES}")

IF(CHRONOENGINE_LIBRARY_DEBUG)
  MESSAGE(STATUS "ChronoEngine library (debug) found")
ENDIF()
IF(CHRONOENGINE_LIBRARY_RELEASE)
  MESSAGE(STATUS "ChronoEngine library (release) found")
ENDIF()

# Stop now if the path to the Chrono SDK is invalid or if we could not find
# at least one of the debug or release Chrono libraries.
IF(NOT EXISTS "${CH_CHRONO_SDKDIR}/physics/ChSystem.h")
  MESSAGE(FATAL_ERROR "Cannot find the Chrono SDK. Make sure CH_CHRONO_SDKDIR is set correctly.")
  RETURN()
ENDIF()
IF(NOT CHRONOENGINE_LIBRARY_DEBUG AND NOT CHRONOENGINE_LIBRARY_RELEASE)
  MESSAGE(FATAL_ERROR "Cannot find the ChronoEngine library. Check CH_LIBDIR_DEBUG and CH_LIBDIR_RELEASE.")
  RETURN()
ENDIF()

# Add paths to Chrono headers
INCLUDE_DIRECTORIES(${CHRONOENGINE_INCLUDES})

# ------------------------------------------------------------------------------
# Find IrrKlang library
# ------------------------------------------------------------------------------
IF(ENABLE_IRRKLANG)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      SET(CH_IRRKLANG_SDKDIR ""  CACHE PATH   "Where is your IrrKlang SDK installed?")
	  SET (CH_IRRKLANGLIB   "" CACHE FILEPATH "Where is your Irrklank library?")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      FIND_PATH(CH_IRRKLANG_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrklang" "/usr/local/include/irrklang")
      FIND_LIBRARY(CH_IRRKLANGLIB NAMES Irrklang PATHS "/usr/local/lib" ${CH_IRRKLANG_SDKDIR}/lib/Linux)
  ENDIF()

  IF(EXISTS "${CH_IRRKLANG_SDKDIR}/include")
      SET(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}/include")
  ELSE()
      SET(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}")
  ENDIF()

  INCLUDE_DIRECTORIES(${CH_IRRKLANGINC})

  MARK_AS_ADVANCED(CLEAR CH_IRRKLANG_SDKDIR)
  MARK_AS_ADVANCED(CLEAR CH_IRRKLANGLIB)

ELSE()

  MARK_AS_ADVANCED(FORCE CH_IRRKLANG_SDKDIR)
  MARK_AS_ADVANCED(FORCE CH_IRRKLANGLIB)

ENDIF()

# ------------------------------------------------------------------------------
# Add paths to the top of the source directory and the binary directory
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
)

# ------------------------------------------------------------------------------
# Automatically copy DLLs 
# ------------------------------------------------------------------------------
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # HACK?
    IF(CH_LIBDIR_DEBUG)
        SET(CH_BINDIR "${CH_LIBDIR_DEBUG}/../../bin")
    ENDIF()
    IF(CH_LIBDIR_RELEASE)
        SET(CH_BINDIR "${CH_LIBDIR_RELEASE}/../../bin")
    ENDIF()

    # Create custom target for copying DLLs; add it to the default build target
    ADD_CUSTOM_TARGET(COPY_DLLS ALL)

    # Create custom commands, invoked post-build to copy DLLs to the appropriate
    # directory (depending on the configuration selected at build time in VS)
    ADD_CUSTOM_COMMAND(
        TARGET COPY_DLLS POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine.dll"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_POSTPROCESS.dll"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    )

    IF(ENABLE_IRRLICHT)
        IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_SDKDIR}/bin/Win32-VisualStudio/Irrlicht.dll")
        ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_SDKDIR}/bin/Win64-VisualStudio/Irrlicht.dll")
        ENDIF()

        ADD_CUSTOM_COMMAND(
            TARGET COPY_DLLS POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CH_BINDIR}/$<CONFIGURATION>/ChronoEngine_IRRLICHT.dll"
                "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CH_IRRLICHT_DLL}"
                "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
        )
    ENDIF()

    IF(ENABLE_IRRKLANG)
        IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
            SET(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Win32-VisualStudio/irrKlang.dll")
        ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
            SET(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Winx64-VisualStudio/irrKlang.dll")
        ENDIF()

        ADD_CUSTOM_COMMAND(
            TARGET COPY_DLLS POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CH_IRRKLANG_DLL}"
                "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
        )
    ENDIF()

ENDIF()

# ------------------------------------------------------------------------------
# Copy the data folder to the bin directory
# ------------------------------------------------------------------------------

IF(MSVC)
  FILE( COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSEIF(XCODE_VERSION)
  FILE( COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSE()
  FILE( COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/data/)
ENDIF()


# ------------------------------------------------------------------------------
# Create configuration file
# ------------------------------------------------------------------------------

# Prepare replacement variables

IF(ENABLE_IRRLICHT)
  SET(IRRLICHT_ENABLED "1")
ELSE()
  SET(IRRLICHT_ENABLED "0")
ENDIF()

IF(ENABLE_IRRKLANG)
  SET(IRRKLANG_ENABLED "1")
ELSE()
  SET(IRRKLANG_ENABLED "0")
ENDIF()

SET(CHRONO_DATA_DIR "${CH_CHRONO_SDKDIR}/demos/data/")

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_MODULE_PATH}/ChronoT_config.h.in
  ${PROJECT_BINARY_DIR}/ChronoT_config.h
  )

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})


# ------------------------------------------------------------------------------
# Add subdirectories
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(utils)
ADD_SUBDIRECTORY(subsys)
ADD_SUBDIRECTORY(models)
ADD_SUBDIRECTORY(tests)
