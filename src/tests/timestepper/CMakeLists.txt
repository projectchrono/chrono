#--------------------------------------------------------------
# Add executables

SET(TESTS
  test_CH_integratorHHT
)

# Compiler flags, libraries, etc.
SET(COMPILER_FLAGS "${CH_CXX_FLAGS}")
SET(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

IF(ENABLE_MODULE_MKL)
  INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
  SET(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  SET(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  SET(LIB_MKL ChronoEngine_mkl)
ELSE()
  SET(LIB_MKL "")
ENDIF()

MESSAGE(STATUS "Additional test programs for Chrono timesteppers...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ${LIB_MKL})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH(PROGRAM)
