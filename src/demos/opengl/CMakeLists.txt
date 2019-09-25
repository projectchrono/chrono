#--------------------------------------------------------------
# List of all OpenGL demos

SET(DEMOS
  demo_GL_cohesion
  demo_GL_inv_pendulum
  demo_GL_benchmark
  demo_GL_shapes
)

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for OpenGL module...")

FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

  TARGET_LINK_LIBRARIES(${PROGRAM}
    ChronoEngine
    ChronoEngine_opengl
    )
    
  ADD_DEPENDENCIES (${PROGRAM} ChronoEngine ChronoEngine_opengl)

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()

