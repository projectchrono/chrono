#===============================================================================
# CMake configuration file for the Chrono irrlicht module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#===============================================================================

option(CH_ENABLE_MODULE_IRRLICHT "Enable the Chrono Irrlicht module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_IRRLICHT)
  return()
endif()

message(STATUS "\n==== Chrono Irrlicht module ====\n")

#-------------------------------------------------------------------------------
# List all the files in the ChronoEngine_irrlicht lib

set(IRR_SOURCE_FILES
  ChIrrCamera.cpp
  ChIrrMeshTools.cpp
  ChIrrTools.cpp
  ChIrrSkyBoxSceneNode.cpp
  ChIrrGUI.cpp
  ChVisualSystemIrrlicht.cpp
  ChIrrNodeModel.cpp
  ChIrrNodeShape.cpp
)

set(IRR_HEADER_FILES
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

source_group("" FILES ${IRR_SOURCE_FILES} ${IRR_HEADER_FILES})

#-------------------------------------------------------------------------------
# Find the Irrlicht library

find_package(Irrlicht MODULE REQUIRED)

if(NOT Irrlicht_FOUND)
  message("WARNING: Irrlicht not found; disabling Chrono::Irrlicht")
  set(CH_ENABLE_MODULE_IRRLICHT OFF CACHE BOOL "Enable the Chrono::Irrlicht module" FORCE)
  return()
endif()

set(CH_IRRLICHT_INCLUDE_DIR ${IRRLICHT_INCLUDE_DIR} PARENT_SCOPE)
set(CH_IRRLICHT_LIBRARY ${IRRLICHT_LIBRARY} PARENT_SCOPE)
set(CH_IRRLICHT_DLL ${IRRLICHT_DLL} PARENT_SCOPE)

#-------------------------------------------------------------------------------
# Create the ChronoEngine_irrlicht library

add_library(ChronoEngine_irrlicht ${IRR_SOURCE_FILES} ${IRR_HEADER_FILES})
add_library(ChronoEngine::irrlicht ALIAS ChronoEngine_irrlicht)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(ChronoEngine_irrlicht PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(ChronoEngine_irrlicht PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(ChronoEngine_irrlicht PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(ChronoEngine_irrlicht PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

if(MSVC)
  set_target_properties(ChronoEngine_irrlicht PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(ChronoEngine_irrlicht PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_IRRLICHT>)
target_compile_definitions(ChronoEngine_irrlicht PRIVATE $<$<COMPILE_LANGUAGE:CXX>:IGNORE_DEPRECATED_WARNING>)

target_link_libraries(ChronoEngine_irrlicht PRIVATE ChronoEngine)
target_link_libraries(ChronoEngine_irrlicht PUBLIC Irrlicht::Irrlicht)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(ChronoEngine_irrlicht PRIVATE ${MAC_LIBS})
endif()

install(TARGETS ChronoEngine_irrlicht
  EXPORT ChronoEngineTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_irrlicht
)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${IRR_HEADER_FILES}
        DESTINATION include/chrono_irrlicht)

# Install files for chrono-config (in both build and install trees)
file(COPY "${CMAKE_SOURCE_DIR}/cmake/FindIrrlicht.cmake" DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindIrrlicht.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})

# On Windows, install DLLs
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  install(FILES "${IRRLICHT_DLL}" DESTINATION bin)
endif()
