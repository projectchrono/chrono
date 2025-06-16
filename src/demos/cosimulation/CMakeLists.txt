#--------------------------------------------------------------
# Add executables

SET(DEMOS
  demo_COSIM_socket
  demo_COSIM_data_exchange
  demo_COSIM_hydraulics
)

# MESSAGE(STATUS "Demo programs for SOCKET COMMUNICATION module...")

FOREACH(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos)
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} Chrono_core)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    INSTALL(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH(PROGRAM)
