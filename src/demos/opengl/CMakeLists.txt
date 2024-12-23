#--------------------------------------------------------------
# List of all OpenGL demos

SET(DEMOS
  demo_GL_cohesion
  demo_GL_inv_pendulum
  demo_GL_benchmark
  demo_GL_shapes
  demo_GL_assets
)

#--------------------------------------------------------------

# Add executables

# MESSAGE(STATUS "Demo programs for OpenGL module...")

FOREACH(PROGRAM ${DEMOS})
  # MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  TARGET_LINK_LIBRARIES(${PROGRAM}
    ChronoEngine
    ChronoEngine_opengl)

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
