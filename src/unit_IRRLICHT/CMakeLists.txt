#===============================================================================
# CHRONO::ENGINE   CMake configuration file for IRRLICHT unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

SET(ENABLE_UNIT_IRRLICHT
  FALSE  CACHE  BOOL
  "Turn this ON to generate the Chrono::Engine Irrlicht unit."
)

IF(NOT ENABLE_UNIT_IRRLICHT)
  RETURN()
ENDIF()

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_IRRLICHT library

SET(ChronoEngine_UNIT_IRRLICHT_SOURCES
  ChIrrApp.cpp
  ChIrrAppInterface.cpp
  ChIrrAssetConverter.cpp
  ChIrrCamera.cpp
  ChIrrNodeProxyToAsset.cpp
  ChIrrTools.cpp
)

SET(ChronoEngine_UNIT_IRRLICHT_HEADERS
  ChApiIrr.h
  ChBodySceneNode.h
  ChBodySceneNodeTools.h
  ChIrrTools.h
  ChIrrAppInterface.h
  ChIrrApp.h
  ChIrrCamera.h
  ChIrrMeshTools.h
  ChIrrWizard.h
  ChParticlesSceneNode.h
  ChIrrAssetConverter.h
  ChIrrNode.h
  ChIrrNodeProxyToAsset.h
  ChIrrNodeAsset.h
  ChIrrEffects.h
)

SOURCE_GROUP(unit_IRRLICHT FILES
  ${ChronoEngine_UNIT_IRRLICHT_SOURCES}
  ${ChronoEngine_UNIT_IRRLICHT_HEADERS}
)

#-------------------------------------------------------------------------------
# Find the IRRLICHT library

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(CH_IRRLICHTDIR "D:/irrlicht-1.8"  CACHE PATH   "Where is your Irrlicht SDK installed? You must set this path to compile demos with 3D display.")

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
  FIND_PATH(CH_IRRLICHTDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY(CH_IRRLICHTLIB NAMES Irrlicht PATHS  ${CH_IRRLICHTDIR}/lib/Linux)
  SET(CH_IRRLICHTLIB "${CH_IRRLICHTLIB}" -lXxf86vm -lglut -lX11 -lGL)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(CH_IRRLICHTDIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY( CH_IRRLICHTLIB NAMES Irrlicht PATHS "/usr/local/lib" ${CH_IRRLICHTDIR}/lib/Linux)
  SET(CH_IRRLICHTLIB "${CH_IRRLICHTLIB}")
ENDIF()

# Add path to IRRLICHT headers

IF(EXISTS "${CH_IRRLICHTDIR}/include")
  SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}/include")
ELSE()
  SET(CH_IRRLICHTINC "${CH_IRRLICHTDIR}")
ENDIF()

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

# Let some variables be visible also from outside this directory

SET(CH_IRRLICHTINC  "${CH_IRRLICHTINC}"  PARENT_SCOPE)
SET(CH_IRRLICHTLIB  "${CH_IRRLICHTLIB}"  PARENT_SCOPE)

#-------------------------------------------------------------------------------
# On Windows, copy the IRRLICHT DLL to the bin/ directory
# We do this through a custom command so that the Irrlich DLL is copied to the
# appropriate directory (depending on the build type); however, we use
# copy_if_different.

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  ADD_CUSTOM_TARGET(COPY_DLLS ALL)

  IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    SET(CH_IRRLICHT_DLL "${CH_IRRLICHTDIR}/bin/Win32-VisualStudio/Irrlicht.dll")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    SET(CH_IRRLICHT_DLL "${CH_IRRLICHTDIR}/bin/Win64-VisualStudio/Irrlicht.dll")
  ENDIF()

  ADD_CUSTOM_COMMAND(
    TARGET COPY_DLLS POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_IRRLICHT_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
  )

ENDIF()

#-------------------------------------------------------------------------------
# Create the ChronoEngine_IRRLICHT library

ADD_LIBRARY(ChronoEngine_IRRLICHT SHARED
  ${ChronoEngine_UNIT_IRRLICHT_SOURCES}
  ${ChronoEngine_UNIT_IRRLICHT_HEADERS}
)

SET_TARGET_PROPERTIES(ChronoEngine_IRRLICHT PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_IRRLICHT")

TARGET_LINK_LIBRARIES(ChronoEngine_IRRLICHT
  ChronoEngine
  ${CH_IRRLICHTLIB}
)

ADD_DEPENDENCIES (ChronoEngine_IRRLICHT ChronoEngine)

#-------------------------------------------------------------------------------
# Install the ChronoEngine_IRRLICHT library

INSTALL(TARGETS ChronoEngine_IRRLICHT
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

