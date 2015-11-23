SET(LIBRARIES ChronoEngine ChronoEngine_fea)

SET(TESTS
    test_ANCFBeam
    test_ANCFShell_Iso
    test_ANCFShell_Ort
    test_ANCFShell_OrtGrav
    test_EASBrickIso
    test_EASBrickIso_Grav
test_EASBrickMooneyR_Grav
test_ANCFConstraints)
MESSAGE(STATUS "Unit test programs for FEA module...")
# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/$<CONFIGURATION>")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

    #ADD_TEST(NAME ${PROGRAM}
    #         COMMAND ${PROJECT_BINARY_DIR}/bin/${PROGRAM}
    #         WORKING_DIRECTORY ${MY_WORKING_DIR})

    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

    SET_TESTS_PROPERTIES (${PROGRAM} PROPERTIES 
                          WORKING_DIRECTORY ${MY_WORKING_DIR})

ENDFOREACH()
