SET(LIBRARIES ChronoEngine)
INCLUDE_DIRECTORIES( ${CH_INCLUDES} )

SET(TESTS
    utest_CH_archive
    utest_CH_ChVector
    utest_CH_ChQuaternion
    utest_CH_ChState
    utest_CH_coords
    utest_CH_linalg
    utest_CH_math
    utest_CH_sparsematrix
    utest_CH_ISO2631
    #utest_CH_stream
)


if(CH_CXX14)
   set(TESTS ${TESTS}
       utest_CH_ChFunction_Lambda)
endif()

if(ENABLE_MODULE_IRRLICHT)
    include_directories( ${CH_IRRLICHT_INCLUDES} )
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES "ChronoEngine_irrlicht")
endif()



MESSAGE(STATUS "Unit test programs for CORE module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${COMPILER_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)


