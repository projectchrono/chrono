#--------------------------------------------------------------
# Add executables

SET(DEMOS
  demo_socket
  demo_cosimulation
  demo_cosim_hydraulics
)

MESSAGE(STATUS "Demo programs for COSIMULATION module...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER cosimulation
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_cosimulation)
    ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_cosimulation)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH(PROGRAM)
