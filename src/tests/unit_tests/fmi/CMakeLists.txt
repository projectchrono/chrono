set(TESTS
    utest_FMI_resource_location
    )

set(LIBS Chrono_core)

MESSAGE(STATUS "Add unit test programs for FMI module")
build_utests(${BUILD_TESTING_FMI} "${TESTS}" "fmi" "${LIBS}")

# These tests exercise fmu-forge headers directly, so they need its top-level directory on the include path.
# Honor a user override of FMU_FORGE_DIR rather than assuming the in-tree submodule.
foreach(PROGRAM ${TESTS})
    target_include_directories(${PROGRAM} PRIVATE "${FMU_FORGE_DIR}")
endforeach()
