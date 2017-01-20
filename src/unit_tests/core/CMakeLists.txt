SET(LIBRARIES ChronoEngine)
INCLUDE_DIRECTORIES( ${CH_INCLUDES} )

SET(TESTS
    utest_CH_ChVector
    utest_CH_coords
    utest_CH_math
    utest_CH_sparse_matrix
    utest_CH_ChCSR3Matrix
    #utest_CH_stream
)

if(AVX_FOUND)
   set(TESTS ${TESTS} 
       utest_CH_AVX
   )
endif()

if(CH_CXX14)
   set(TESTS ${TESTS}
       utest_CH_ChFunction_Lambda)
endif()

MESSAGE(STATUS "Unit test programs for CORE module...")

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

    #INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
