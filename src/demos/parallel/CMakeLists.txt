# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_parallel
)

IF(ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)

    IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
       SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
    ENDIF()
ENDIF()

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
SET(DEMOS
    demo_PAR_ballsSMC
    demo_PAR_ballsNSC
    demo_PAR_mixerSMC
    demo_PAR_mixerNSC
    demo_PAR_fluidNSC
    demo_PAR_snowMPM
    demo_PAR_particlesNSC
    demo_PAR_friction
)

# Add programs that require OpenGL
if(ENABLE_MODULE_OPENGL)
    set(DEMOS ${DEMOS} demo_PAR_motors)
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Demo programs for PARALLEL module...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH(PROGRAM)

