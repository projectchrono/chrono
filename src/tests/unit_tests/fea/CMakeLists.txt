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

if(CH_ENABLE_MODULE_PARDISO_MKL)
  set(TESTS ${TESTS}
    utest_FEA_constraints
  )
endif()

set(LIBS Chrono_core)
if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS Chrono_pardisomkl)
endif()

MESSAGE(STATUS "Add unit test programs for FEA module")
build_utests(YES "${TESTS}" "${LIBS}")
