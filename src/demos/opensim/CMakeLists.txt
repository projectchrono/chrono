#--------------------------------------------------------------
# Add executables

SET(DEMOS
  demo_OSIM_test
)


MESSAGE(STATUS "Demo programs for OpenSim module...")

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${CH_IRRLICHTLIB} ChronoEngine ChronoEngine_irrlicht)
    ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_irrlicht)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH(PROGRAM)
