#--------------------------------------------------------------
# Add executables

SET(DEMOS
  demo_CH_coords
  demo_CH_linalg
  demo_CH_matrix_ref
  demo_CH_math
  demo_CH_stream
  demo_CH_archive
  demo_CH_buildsystem
  demo_CH_powertrain
  demo_CH_functions
  demo_CH_solver
  demo_CH_EulerAngles
  demo_CH_filesystem
)


MESSAGE(STATUS "Demo programs for CORE module...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine)
    ADD_DEPENDENCIES(${PROGRAM} ChronoEngine)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH(PROGRAM)

