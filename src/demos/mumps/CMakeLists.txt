set(MUMPS_DEMOS
    demo_MUMPS_MumpsEngine
)

 
# MESSAGE(STATUS "Demo programs for Mumps module...")

# ---------------------------------------------------------------------

FOREACH(PROGRAM ${MUMPS_DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          FOLDER demos)
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} Chrono_core Chrono_mumps)

    if(MSVC)
        set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
