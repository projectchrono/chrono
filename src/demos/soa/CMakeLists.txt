if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

set(TESTS
    demo_SOA_pendulum
)

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS "Chrono_vsg")
endif()

message(STATUS "Add demo programs for SOA module")

foreach(PROGRAM ${TESTS})
    #message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  ${PROGRAM}.cpp ${BULLET_FILES})
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)


