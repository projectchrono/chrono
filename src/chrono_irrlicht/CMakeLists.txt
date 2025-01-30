#===============================================================================
# CMake configuration file for the Chrono irrlicht module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(ENABLE_MODULE_IRRLICHT "Enable the Chrono Irrlicht module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_IRRLICHT)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Irrlicht module ====\n")

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

find_package(Irrlicht)

if(IRRLICHT_FOUND)
    message(STATUS "Irrlicht libraries:   ${IRRLICHT_LIBRARIES}")
    message(STATUS "Irrlicht include dir: ${IRRLICHT_INCLUDE_DIRS}")
    if(MSVC)
    message(STATUS "Irrlicht DLL:         ${IRRLICHT_DLL}")
    endif()
else()
    message(ERROR " IRRLICHT not found. Please set IRRLICHT_DIR to the Irrlicht SDK installation directory.")
    set(IRRLICHT_DIR "C:/irrlicht-1.8.1" CACHE PATH "Path to Irrlicht SDK installation.")
endif()

#-------------------------------------------------------------------------------
# Create the ChronoEngine_irrlicht library

add_library(ChronoEngine_irrlicht
  ${ChronoEngine_IRRLICHT_SOURCES}
  ${ChronoEngine_IRRLICHT_HEADERS}
)

add_library(ChronoEngine::ChronoEngine_irrlicht ALIAS ChronoEngine_irrlicht)

if(MSVC)
  set_target_properties(ChronoEngine PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(ChronoEngine_irrlicht PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_IRRLICHT>)
target_compile_definitions(ChronoEngine_irrlicht PRIVATE $<$<COMPILE_LANGUAGE:CXX>:IGNORE_DEPRECATED_WARNING>)

target_link_libraries(ChronoEngine_irrlicht PRIVATE ChronoEngine)
target_link_libraries(ChronoEngine_irrlicht PUBLIC Irrlicht::Irrlicht)

if (ENABLE_MODULE_POSTPROCESS)
    target_link_libraries(ChronoEngine_irrlicht PRIVATE ChronoEngine_postprocess)
endif()
if (ENABLE_MODULE_MODAL)
    target_link_libraries(ChronoEngine_irrlicht PRIVATE ChronoEngine_modal)
endif()


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(ChronoEngine_irrlicht PRIVATE ${MAC_LIBS})
ENDIF()

#-------------------------------------------------------------------------------
# On Windows, copy the Irrlicht DLL to the bin/ directory
# We do this through a custom command so that the Irrlich DLL is copied to the
# appropriate directory (depending on the build type); however, we use
# copy_if_different.


IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

  # Make variable visible from outside this directory
  SET(CH_IRRLICHT_DLL "${IRRLICHT_DLL}")
  SET(CH_IRRLICHT_DLL "${CH_IRRLICHT_DLL}" PARENT_SCOPE)

  add_custom_command(
    TARGET ChronoEngine_irrlicht POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_IRRLICHT_DLL}"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    COMMENT "Copying Irrlicht DLL..."
  )

  # copy the IRRLICHT dll into the installed package to be on the safe side 
  install(FILES "${CH_IRRLICHT_DLL}" DESTINATION bin)

ENDIF()


#-------------------------------------------------------------------------------
# Install the ChronoEngine_irrlicht library

install(TARGETS ChronoEngine_irrlicht
  EXPORT ChronoEngineTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_irrlicht
)

# # TODO: DARIOM check if this is already covered by the install TARGETS above
# install(FILES ${ChronoEngine_IRRLICHT_HEADERS}
#         DESTINATION include/chrono_irrlicht)

file(COPY "${CMAKE_SOURCE_DIR}/cmake/FindIrrlicht.cmake"
     DESTINATION ${CMAKE_BINARY_DIR}/cmake/)

install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindIrrlicht.cmake"
        DESTINATION ${CH_CONFIG_INSTALL_PATH}
        )