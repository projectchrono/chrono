set(TESTS
    utest_FSI-SPH_Poiseuille_flow
    utest_FSI-SPH_elastic_shear
    utest_FSI-SPH_domain_guard
    utest_FSI-SPH_domain_update
    utest_FSI-SPH_kernel_support
    utest_FSI-SPH_setter_guard
    utest_FSI-SPH_rheology_error_flag
    utest_FSI-SPH_mcc_parameters
    utest_FSI-SPH_proximity_cadence
    utest_FSI-SPH_reproducibility
    utest_FSI-SPH_free_surface_export
    utest_FSI-SPH_rigid_bce_state
    utest_FSI-SPH_marker_getters
)

if(CH_ENABLE_MODULE_FEA)
  list(APPEND TESTS utest_FSI-SPH_flex1D_bce_frame)
  list(APPEND TESTS utest_FSI-SPH_flex1D_node_directions)
endif()

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsisph)
if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS Chrono_fsisph_vsg)
endif()

message(STATUS "Add unit test programs for FSI module")
build_utests(YES "${TESTS}" "sph" "${LIBS}")

# Compile the internal force-transfer units directly, without exporting their private interfaces.
set(FLEX_FORCE_SOURCES
    ${PROJECT_SOURCE_DIR}/src/chrono_fsi/sph/physics/SphBceManager.cu
    ${PROJECT_SOURCE_DIR}/src/chrono_fsi/sph/physics/SphDataManager.cu
    ${PROJECT_SOURCE_DIR}/src/chrono_fsi/sph/utils/SphUtilsDevice.cu
)
chrono_set_gpu_source_language(${CHRONO_FSISPH_BACKEND} ${FLEX_FORCE_SOURCES})
set_source_files_properties(utest_FSI-SPH_flex_force.cpp PROPERTIES LANGUAGE ${CHRONO_FSISPH_BACKEND})
build_utests(YES "utest_FSI-SPH_flex_force" "sph" "PRIVATE;Chrono_core" "${FLEX_FORCE_SOURCES}")
chrono_apply_gpu_backend(utest_FSI-SPH_flex_force ${CHRONO_FSISPH_BACKEND})
target_compile_definitions(utest_FSI-SPH_flex_force PRIVATE THRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_CPP)
if(CHRONO_FSISPH_BACKEND STREQUAL "CUDA" OR CHRONO_HIP_PLATFORM STREQUAL "nvidia")
    target_compile_definitions(utest_FSI-SPH_flex_force PRIVATE THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_CUDA)
    target_compile_options(utest_FSI-SPH_flex_force PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
else()
    target_compile_definitions(utest_FSI-SPH_flex_force PRIVATE THRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_HIP)
endif()
if(CHRONO_FSISPH_BACKEND STREQUAL "CUDA")
    target_link_libraries(utest_FSI-SPH_flex_force PRIVATE Thrust::Thrust)
endif()
