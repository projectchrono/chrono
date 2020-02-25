set(MUMPS_DEMOS
    demo_MUMPS_MumpsEngine
)

# Additional include paths
INCLUDE_DIRECTORIES( ${CH_MUMPS_INCLUDES} )
 
MESSAGE(STATUS "Demo programs for Mumps module...")

# ---------------------------------------------------------------------

FOREACH(PROGRAM ${MUMPS_DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          FOLDER demos
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_mumps)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
