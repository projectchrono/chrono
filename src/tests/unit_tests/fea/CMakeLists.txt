# Unit tests for the Chrono::FEA module
# ==================================================================

set(TESTS
    utest_FEA_ANCFCable
    utest_FEA_ANCFbeam_3333
    utest_FEA_ANCFbeam_Twist
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
    utest_FEA_beams_static
	utest_FEA_ANCFbeam_3243_Formulation
	utest_FEA_ANCFbeam_3333_Formulation
	utest_FEA_ANCFshell_3423_Formulation
	utest_FEA_ANCFshell_3443_Formulation
	utest_FEA_ANCFshell_3833_Formulation
	utest_FEA_ANCFhexa_3843_Formulation
    utest_FEA_ANCFhexa_3813_9
)

# Tests that REQUIRE Chrono::MKL
set(TESTS_MKL
    utest_FEA_constraints
)

MESSAGE(STATUS "Add unit test programs for FEA module")

# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(LIBRARIES Chrono_core)

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBRARIES "Chrono_pardisomkl")
    set(TESTS ${TESTS} ${TESTS_MKL})
endif()

FOREACH(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES FOLDER tests)
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

    SET_TESTS_PROPERTIES(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
ENDFOREACH()
