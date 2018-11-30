#--------------------------------------------------------------
# List of all Irrlicht demos

SET(DEMOS
  demo_GD_collisionNSC
)

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_GODOTINC})
# INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC})

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for GODOT module...")

FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

   MESSAGE(STATUS "CH_CXX FLAGS ${CH_CXX_FLAGS}")
   # MESSAGE(STATUS "IRR_CXX FLAGS ${CH_IRRLICHT_CXX_FLAGS}")
   MESSAGE(STATUS "GD_CXX FLAGS ${CH_GODOT_CXX_FLAGS}")

   IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
   SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
   ENDIF()

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_GODOT_CXX_FLAGS}"
    # COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

  TARGET_LINK_LIBRARIES(${PROGRAM}
    ChronoEngine
    # ChronoEngine_irrlicht
    ChronoEngine_godot
    # ${CH_IRRLICHTLIB}
    ${CH_GODOTLIB}
    )

  ADD_DEPENDENCIES (${PROGRAM} ChronoEngine ChronoEngine_godot)

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
