#===============================================================================
# CMake configuration file for the Chrono irrlicht module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(ENABLE_MODULE_IRRLICHT "Enable the Chrono Irrlicht module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_IRRLICHT)
  MARK_AS_ADVANCED(FORCE IRRLICHT_LIBRARY)
  MARK_AS_ADVANCED(FORCE IRRLICHT_INSTALL_DIR)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Irrlicht module ====\n")

MARK_AS_ADVANCED(CLEAR IRRLICHT_LIBRARY)
MARK_AS_ADVANCED(CLEAR IRRLICHT_INSTALL_DIR)

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_irrlicht lib
SET(ChronoEngine_IRRLICHT_SOURCES
  ChIrrCamera.cpp
  ChIrrMeshTools.cpp
  ChIrrTools.cpp
  ChIrrSkyBoxSceneNode.cpp
  ChIrrGUI.cpp
  ChVisualSystemIrrlicht.cpp
  ChIrrNodeModel.cpp
  ChIrrNodeShape.cpp
)

SET(ChronoEngine_IRRLICHT_HEADERS
  ChApiIrr.h
  ChIrrTools.h
  ChIrrCamera.h
  ChIrrMeshTools.h
  ChIrrEffects.h
  ChIrrSkyBoxSceneNode.h
  ChIrrGUI.h
  ChVisualSystemIrrlicht.h
  ChIrrNodeModel.h
  ChIrrNodeShape.h
)

SOURCE_GROUP("" FILES
  ${ChronoEngine_IRRLICHT_SOURCES}
  ${ChronoEngine_IRRLICHT_HEADERS}
)

#-------------------------------------------------------------------------------
# Find the IRRLICHT library

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(IRRLICHT_INSTALL_DIR "C:/irrlicht-1.8.1"  CACHE PATH   "Path to Irrlicht SDK installation.")
  IF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_INSTALL_DIR}/lib/Win32-visualstudio")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_INSTALL_DIR}/lib/Win64-visualStudio")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_INSTALL_DIR}/lib/Win32-gcc")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
    FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "${IRRLICHT_INSTALL_DIR}/lib/Win64-gcc")
  ENDIF()
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  FIND_PATH(IRRLICHT_INSTALL_DIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY(IRRLICHT_LIBRARY NAMES Irrlicht PATHS  ${IRRLICHT_INSTALL_DIR}/lib/Linux)
  SET(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}" -lXxf86vm -lglut -lX11 -lGL)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  FIND_PATH(IRRLICHT_INSTALL_DIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY( IRRLICHT_LIBRARY NAMES Irrlicht PATHS "/usr/local/lib" ${IRRLICHT_INSTALL_DIR}/lib/Linux)
  INCLUDE_DIRECTORIES ( /System/Library/Frameworks )
  FIND_LIBRARY(COCOA_LIBRARY Cocoa)
  FIND_LIBRARY(OPENGL_LIBRARY OpenGL)
  FIND_LIBRARY(IOKIT_LIBRARY IOKit)
  SET(MAC_LIBS ${COCOA_LIBRARY} ${OPENGL_LIBRARY} ${IOKIT_LIBRARY})
  SET(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}")
else()
  FIND_PATH(IRRLICHT_INSTALL_DIR NAMES irrlicht.h PATHS "/usr/include/irrlicht" "/usr/local/include/irrlicht")
  FIND_LIBRARY(IRRLICHT_LIBRARY NAMES Irrlicht PATHS ${IRRLICHT_INSTALL_DIR}/../../lib)
  set(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}")
ENDIF()

# Add path to IRRLICHT headers
IF(EXISTS "${IRRLICHT_INSTALL_DIR}/include/irrlicht")
  SET(IRRLICHT_INCLUDE_DIR "${IRRLICHT_INSTALL_DIR}/include/irrlicht")
ELSEIF(EXISTS "${IRRLICHT_INSTALL_DIR}/include")
  SET(IRRLICHT_INCLUDE_DIR "${IRRLICHT_INSTALL_DIR}/include")
ELSE()
  SET(IRRLICHT_INCLUDE_DIR "${IRRLICHT_INSTALL_DIR}")
ENDIF()

# Irrlicht-specific compiler flags
set(IRRLICHT_CXX_FLAGS "")
set(IRRLICHT_C_FLAGS "")
if(MSVC)
  # If using MSVC, disable warning 4275 (non-DLL-interface class used as base for DLL-interface class)
  ###add_compile_options(/wd4275)
  set(IRRLICHT_CXX_FLAGS "${IRRLICHT_CXX_FLAGS} /wd4275")
  set(IRRLICHT_C_FLAGS "${IRRLICHT_C_FLAGS} /wd4275")
endif()

# Let some variables be visible also from outside this directory
set(CH_IRRLICHT_INCLUDES  "${IRRLICHT_INCLUDE_DIR}" PARENT_SCOPE)
set(CH_IRRLICHT_CXX_FLAGS "${IRRLICHT_CXX_FLAGS}"   PARENT_SCOPE)
set(CH_IRRLICHT_C_FLAGS   "${IRRLICHT_C_FLAGS}"     PARENT_SCOPE)

message(STATUS "Include directory: ${IRRLICHT_INCLUDE_DIR}")
message(STATUS "Library:           ${IRRLICHT_LIBRARY}")

#-------------------------------------------------------------------------------
# Create the ChronoEngine_irrlicht library

include_directories(${IRRLICHT_INCLUDE_DIR})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${IRRLICHT_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_LIB}")
set(LIBRARIES 
    ChronoEngine
    ${IRRLICHT_LIBRARY}
    )

if (ENABLE_MODULE_POSTPROCESS)
    set(LIBRARIES ${LIBRARIES} ChronoEngine_postprocess)
endif()
if (ENABLE_MODULE_MODAL)
    set(LIBRARIES ${LIBRARIES} ChronoEngine_modal)
endif()

ADD_LIBRARY(ChronoEngine_irrlicht
  ${ChronoEngine_IRRLICHT_SOURCES}
  ${ChronoEngine_IRRLICHT_HEADERS}
)

SET_TARGET_PROPERTIES(ChronoEngine_irrlicht PROPERTIES
  COMPILE_FLAGS "${COMPILER_FLAGS}"
  LINK_FLAGS "${LINKER_FLAGS}")

target_compile_definitions(ChronoEngine_irrlicht PRIVATE "CH_API_COMPILE_IRRLICHT")
target_compile_definitions(ChronoEngine_irrlicht PRIVATE "CH_IGNORE_DEPRECATED")
target_compile_definitions(ChronoEngine_irrlicht PRIVATE "IGNORE_DEPRECATED_WARNING")

TARGET_LINK_LIBRARIES(ChronoEngine_irrlicht
  ${LIBRARIES}
)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
TARGET_LINK_LIBRARIES(ChronoEngine_irrlicht
  ${MAC_LIBS}
)
ENDIF()

ADD_DEPENDENCIES (ChronoEngine_irrlicht ChronoEngine)

if (ENABLE_MODULE_POSTPROCESS)
    ADD_DEPENDENCIES (ChronoEngine_irrlicht ChronoEngine_postprocess)
endif()

if (ENABLE_MODULE_MODAL)
    ADD_DEPENDENCIES (ChronoEngine_irrlicht ChronoEngine_modal)
endif()


#-------------------------------------------------------------------------------
# On Windows, copy the IRRLICHT DLL to the bin/ directory
# We do this through a custom command so that the Irrlich DLL is copied to the
# appropriate directory (depending on the build type); however, we use
# copy_if_different.

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  IF(DEFINED ENV{CONDA_BUILD})
    SET(CH_IRRLICHT_DLL "$ENV{PREFIX}/Library/bin/Irrlicht.dll")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
    SET(CH_IRRLICHT_DLL "${IRRLICHT_INSTALL_DIR}/bin/Win32-VisualStudio/Irrlicht.dll")
  ELSEIF("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
    SET(CH_IRRLICHT_DLL "${IRRLICHT_INSTALL_DIR}/bin/Win64-VisualStudio/Irrlicht.dll")
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

if(CH_STATIC)
  SET(CH_IRRLICHT_LIB "${IRRLICHT_LIBRARY}" PARENT_SCOPE)
endif()


#-------------------------------------------------------------------------------
# Install the ChronoEngine_irrlicht library

INSTALL(TARGETS ChronoEngine_irrlicht
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_IRRLICHT_HEADERS} DESTINATION include/chrono_irrlicht)
