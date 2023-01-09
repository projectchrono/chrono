#--------------------------------------------------------------
# List of all VSG demos

SET(DEMOS
  demo_VSG_adams_parser
  demo_VSG_assets
  demo_VSG_callbackNSC
  demo_VSG_callbackSMC
  demo_VSG_earthquake
  demo_VSG_lab
  demo_VSG_paths
  demo_VSG_opensim_parser
  demo_VSG_spring
)

set(CMAKE_CXX_STANDARD 17)

#--- maybe it is not necessary to build app bundles with vsg
if(APPLE)
    set(CMAKE_MACOSX_BUNDLE OFF)
endif()

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES(${CH_VSG_INCLUDES})

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for VSG module...")

FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}")
  SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  TARGET_LINK_LIBRARIES(${PROGRAM}
    ChronoEngine
    ChronoEngine_vsg)
  ADD_DEPENDENCIES (${PROGRAM} ChronoEngine ChronoEngine_vsg)

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
