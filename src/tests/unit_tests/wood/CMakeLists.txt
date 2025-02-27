# Unit tests for the wood Chrono module
# ==================================================================



SET(LIBRARIES
    Chrono_core
    Chrono_wood
)

SET(TESTS
    utest_WOOD_CBLCON
)

# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()


message(STATUS "Add unit test programs for WOOD module")

FOREACH(PROGRAM ${TESTS})

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
         FOLDER tests
         COMPILE_FLAGS "${CH_CXX_FLAGS}"
         LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

    set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})

ENDFOREACH(PROGRAM)
