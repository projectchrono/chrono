set(TESTS
    utest_SEN_gps
    utest_SEN_rng_streams
)

if(CH_USE_SENSOR_OPTIX)
  set(TESTS ${TESTS}
    utest_SEN_camera_convergence
    utest_SEN_data_access
    utest_SEN_interface
    utest_SEN_optixengine
    utest_SEN_optixgeometry
    utest_SEN_optixpipeline
    utest_SEN_radar
    utest_SEN_threadsafety
  )
  # Only meaningful when NVRTC reads shaders from the build tree. With NVRTC off the shaders are
  # compiled to PTX at build time and nothing is staged for this test to compare against.
  if(CH_USE_SENSOR_NVRTC)
    set(TESTS ${TESTS} utest_SEN_shader_staging)
  endif()
endif()

set(LIBS Chrono_core Chrono_sensor)

message(STATUS "Add unit test programs for SENSOR module")
build_utests(YES "${TESTS}" "sensor" "${LIBS}")

if(CH_USE_SENSOR_OPTIX AND CH_USE_SENSOR_NVRTC)
  # The staging guard compares the shaders the runtime actually compiles against the shaders in the
  # repository. Both directories and the file list are injected from the same variables the staging
  # rules are generated from, so the test cannot silently disagree with the build about which
  # shaders exist. Injected rather than discovered by globbing the staged directory, because a glob
  # would pass vacuously if staging had produced nothing at all.
  if(NOT CH_SENSOR_STAGED_SHADER_NAMES)
    message(FATAL_ERROR
            "utest_SEN_shader_staging is enabled but CH_SENSOR_STAGED_SHADER_NAMES is empty. The "
            "staging block in src/chrono_sensor/CMakeLists.txt did not export it.")
  endif()
  # Joined with commas, NOT left as a CMake list: a semicolon is CMake's list separator, so passing
  # the list verbatim expands into one -D flag per shader and produces a mangled command line.
  string(REPLACE ";" "," _staged_names_csv "${CH_SENSOR_STAGED_SHADER_NAMES}")
  target_compile_definitions(utest_SEN_shader_staging PRIVATE
      CH_SHADER_SOURCE_DIR="${CH_SENSOR_SHADER_SOURCE_DIR}"
      CH_STAGED_SHADER_DIR="${CH_SENSOR_STAGED_SHADER_DIR}"
      CH_STAGED_SHADER_NAMES="${_staged_names_csv}"
  )
endif()
