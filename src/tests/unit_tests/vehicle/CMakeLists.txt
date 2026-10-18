set(TESTS
    utest_VEH_destructors
    utest_VEH_tire_contact
    utest_VEH_scm_backend
    utest_VEH_scm
    utest_VEH_scm_gpu_fallback
)

set(UTILS
   ../ut_utils.h)

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_vehicle)

message(STATUS "Add unit test programs for VEHICLE module")
build_utests(YES "${TESTS}" "vehicle" "${LIBS}" "${UTILS}")

# Run the SCM backend tests a second time with every GPU hidden. A library built with GPU support must then
# select the CPU at run time, including when the GPU backend is requested explicitly.
add_test(NAME utest_VEH_scm_backend_without_device
    COMMAND utest_VEH_scm_backend
    WORKING_DIRECTORY $<TARGET_FILE_DIR:utest_VEH_scm_backend>)
set_tests_properties(utest_VEH_scm_backend_without_device PROPERTIES
    LABELS "vehicle"
    ENVIRONMENT "CUDA_VISIBLE_DEVICES=-1;HIP_VISIBLE_DEVICES=-1")

# Run the SCM GPU fallback test a second time with every GPU hidden. A build with the SCM GPU backend
# must then run SCM on the CPU instead of aborting. In a build without that backend this is a plain
# CPU run.
add_test(NAME utest_VEH_scm_gpu_fallback_nodevice
         COMMAND utest_VEH_scm_gpu_fallback
         WORKING_DIRECTORY $<TARGET_FILE_DIR:utest_VEH_scm_gpu_fallback>)
set_tests_properties(utest_VEH_scm_gpu_fallback_nodevice PROPERTIES
                     LABELS "vehicle"
                     ENVIRONMENT "CUDA_VISIBLE_DEVICES=-1;HIP_VISIBLE_DEVICES=-1;SCM_UTEST_EXPECT_NO_GPU=1")
