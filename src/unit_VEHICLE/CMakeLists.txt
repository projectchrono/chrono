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

#MESSAGE(STATUS "Compiler: ${CH_COMPILER}")
#MESSAGE(STATUS "CH_BUILDFLAGS: ${CH_BUILDFLAGS}")
#MESSAGE(STATUS "CH_LINKERFLAG_EXE: ${CH_LINKERFLAG_EXE}")

#MESSAGE(STATUS "Chrono includes:  ${CHRONOENGINE_INCLUDES}")
#MESSAGE(STATUS "Chrono main lib:  ${CHRONOENGINE_LIBRARY}")
#MESSAGE(STATUS "Chrono libraries: ${CHRONOENGINE_LIBRARIES}")


# Add paths to Chrono headers
INCLUDE_DIRECTORIES(${CHRONOENGINE_INCLUDES})

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

ENDIF()

# ------------------------------------------------------------------------------
# Add subdirectories
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(utils)
ADD_SUBDIRECTORY(subsys)
ADD_SUBDIRECTORY(models)
