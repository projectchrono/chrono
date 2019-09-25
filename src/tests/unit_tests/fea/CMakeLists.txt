# Unit tests for the Chrono::FEA module
# ==================================================================

set(TESTS
    utest_FEA_ANCFCable
    utest_FEA_ANCFBeam
	utest_FEA_IGABeam
    utest_FEA_ANCFShell_Iso
    utest_FEA_ANCFShell_Ort
    utest_FEA_ANCFShell_OrtGrav
    utest_FEA_EASBrickIso
    utest_FEA_EASBrickIso_Grav
    utest_FEA_EASBrickMooneyR_Grav
    utest_FEA_ANCFConstraints
    utest_FEA_ANCFContact
    utest_FEA_compute_contact_mesh
    utest_FEA_Brick9
)

# Tests that REQUIRE Chrono::MKL
set(TESTS_MKL
    utest_FEA_constraints
)

MESSAGE(STATUS "Unit test programs for FEA module...")

# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES ChronoEngine)

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBRARIES "ChronoEngine_mkl")
    set(TESTS ${TESTS} ${TESTS_MKL})
endif()

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

    SET_TESTS_PROPERTIES(${PROGRAM} PROPERTIES 
                         WORKING_DIRECTORY ${MY_WORKING_DIR})
ENDFOREACH()
