set(TESTS
    utest_VEH_destructors
)

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_vehicle)

message(STATUS "Add unit test programs for VEHICLE module")

foreach(PROGRAM ${TESTS})
    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES FOLDER tests)
    if(MSVC)
      set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_link_libraries(${PROGRAM} ${LIBS} gtest_main)

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
    ##add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
endforeach(PROGRAM)
