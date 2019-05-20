#===============================================================================
# CMake configuration file for the Chrono irrlicht module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(ENABLE_MODULE_IRRLICHT "Enable the Chrono Irrlicht module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_IRRLICHT)
  MARK_AS_ADVANCED(FORCE IRRLICHT_LIBRARY)
  MARK_AS_ADVANCED(FORCE IRRLICHT_ROOT)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Irrlicht module ====")

MARK_AS_ADVANCED(CLEAR IRRLICHT_LIBRARY)
MARK_AS_ADVANCED(CLEAR IRRLICHT_ROOT)

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_irrlicht library

SET(ChronoEngine_IRRLICHT_SOURCES
  ChIrrApp.cpp
  ChBodySceneNode.cpp
  ChBodySceneNodeTools.cpp
  ChIrrAppInterface.cpp
  ChIrrAssetConverter.cpp
  ChIrrCamera.cpp
  ChIrrMeshTools.cpp
  ChIrrNode.cpp
  ChIrrNodeProxyToAsset.cpp
  ChIrrParticlesSceneNode.cpp
  ChIrrTools.cpp
  ChIrrWizard.cpp
  ChIrrNodeAsset.cpp
)

SET(ChronoEngine_IRRLICHT_HEADERS
  ChApiIrr.h
  ChBodySceneNode.h
  ChBodySceneNodeTools.h
  ChIrrTools.h
  ChIrrAppInterface.h
  ChIrrApp.h
  ChIrrCamera.h
  ChIrrMeshTools.h
  ChIrrWizard.h
  ChIrrParticlesSceneNode.h
  ChIrrAssetConverter.h
  ChIrrNode.h
  ChIrrNodeProxyToAsset.h
  ChIrrNodeAsset.h
  ChIrrEffects.h
)

SOURCE_GROUP("" FILES
  ${ChronoEngine_IRRLICHT_SOURCES}
  ${ChronoEngine_IRRLICHT_HEADERS}
)

#-------------------------------------------------------------------------------
# Find the IRRLICHT library

set(CH_IRRLICHT_CXX_FLAGS "")
set(CH_IRRLICHT_C_FLAGS "")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(IRRLICHT_ROOT "C:/irrlicht-1.8.1"  CACHE PATH   "Path to Irrlicht SDK installation.")
  IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_ROOT}/lib/Win32-visualstudio")
    set(CH_IRRLICHT_CXX_FLAGS "${CH_IRRLICHT_CXX_FLAGS} /wd4275")
    set(CH_IRRLICHT_C_FLAGS "${CH_IRRLICHT_C_FLAGS} /wd4275")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_ROOT}/lib/Win64-visualStudio")
    set(CH_IRRLICHT_CXX_FLAGS "${CH_IRRLICHT_CXX_FLAGS} /wd4275")
    set(CH_IRRLICHT_C_FLAGS "${CH_IRRLICHT_C_FLAGS} /wd4275")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_ROOT}/lib/Win32-gcc")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_ROOT}/lib/Win64-gcc")
  ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(IRRLICHT_ROOT NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY(IRRLICHT_LIBRARY NAMES Irrlicht PATHS  ${IRRLICHT_ROOT}/lib/Linux)
  SET(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}" -lXxf86vm -lglut -lX11 -lGL)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(IRRLICHT_ROOT NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "/usr/local/lib" ${IRRLICHT_ROOT}/lib/Linux)
  INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
  FIND_LIBRARY(COCOA_LIBRARY Cocoa)
  FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
  FIND_LIBRARY(IOKIT_LIBRARY IOKit)
  SET(MAC_LIBS ${COCOA_LIBRARY} ${OPENGL_LIBRARY} ${IOKIT_LIBRARY})
  SET(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}")
else()
  FIND_PATH(IRRLICHT_ROOT NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY(IRRLICHT_LIBRARY NAMES Irrlicht PATHS ${IRRLICHT_ROOT}/../../lib)
  set(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}")
ENDIF()

# Add path to IRRLICHT headers

IF(EXISTS "${IRRLICHT_ROOT}/include")
  SET(CH_IRRLICHTINC "${IRRLICHT_ROOT}/include")
ELSE()
  SET(CH_IRRLICHTINC "${IRRLICHT_ROOT}")
ENDIF()

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )


# Let some variables be visible also from outside this directory

SET(CH_IRRLICHTINC        "${CH_IRRLICHTINC}"        PARENT_SCOPE)
SET(IRRLICHT_LIBRARY      "${IRRLICHT_LIBRARY}"      PARENT_SCOPE)
set(CH_IRRLICHT_CXX_FLAGS "${CH_IRRLICHT_CXX_FLAGS}" PARENT_SCOPE)
set(CH_IRRLICHT_C_FLAGS   "${CH_IRRLICHT_C_FLAGS}"   PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Create the ChronoEngine_irrlicht library

ADD_LIBRARY(ChronoEngine_irrlicht SHARED
  ${ChronoEngine_IRRLICHT_SOURCES}
  ${ChronoEngine_IRRLICHT_HEADERS}
)

SET_TARGET_PROPERTIES(ChronoEngine_irrlicht PROPERTIES
  COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_IRRLICHT")

TARGET_LINK_LIBRARIES(ChronoEngine_irrlicht
  ChronoEngine
  ${IRRLICHT_LIBRARY}
)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
TARGET_LINK_LIBRARIES(ChronoEngine_irrlicht
  ${MAC_LIBS}
)
ENDIF()

ADD_DEPENDENCIES (ChronoEngine_irrlicht ChronoEngine)

#-------------------------------------------------------------------------------
# On Windows, copy the IRRLICHT DLL to the bin/ directory
# We do this through a custom command so that the Irrlich DLL is copied to the
# appropriate directory (depending on the build type); however, we use
# copy_if_different.

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    SET(CH_IRRLICHT_DLL "${IRRLICHT_ROOT}/bin/Win32-VisualStudio/Irrlicht.dll")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    SET(CH_IRRLICHT_DLL "${IRRLICHT_ROOT}/bin/Win64-VisualStudio/Irrlicht.dll")
  ENDIF()

  ADD_CUSTOM_COMMAND(
    TARGET ChronoEngine_irrlicht POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_IRRLICHT_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    MAIN_DEPENDENCY ChronoEngine_irrlicht
    COMMENT "Copying Irrlicht DLL..."
  )

  # copy the Irrlicht dll into the installed package to be on the safe side 
  install(FILES "${CH_IRRLICHT_DLL}" DESTINATION bin)

  # Make variable visible from outside this directory
  SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_DLL}" PARENT_SCOPE)

ENDIF()

#-------------------------------------------------------------------------------
# Install the ChronoEngine_irrlicht library

INSTALL(TARGETS ChronoEngine_irrlicht
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_IRRLICHT_HEADERS} DESTINATION include/chrono_irrlicht)
