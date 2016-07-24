
# Test programss for the Chrono::MKL module
# ==================================================================

SET(LIBRARIES ChronoEngine ChronoEngine_mkl)
INCLUDE_DIRECTORIES( ${CH_INCLUDES} ${CH_MKL_INCLUDES})

SET(TESTS
    test_MKL_SparseMatrix
)

MESSAGE(STATUS "Additional test programs for MKL module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
ENDFOREACH(PROGRAM)