#--------------------------------------------------------------
# List of all OpenGL demos

SET(DEMOS
  demo_vsg_VEH_M113
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
  ChronoEngine_vehicle
  ChronoEngine_vehicle_vsg
  ChronoModels_vehicle
  ChronoEngine_vsg)
  ADD_DEPENDENCIES (${PROGRAM} ChronoEngine ChronoEngine_vsg)

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
