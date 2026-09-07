# Sensor module unit tests.
#
# Split three ways by what each test actually depends on, rather than by which backend
# happens to be the historical default.

# --- no render backend needed at all ----------------------------------------------
# These touch only the parts of Chrono::Sensor that exist in every configuration, so they
# also build in a sensor module with no ray-tracing backend selected.
set(TESTS
    utest_SEN_gps
    utest_SEN_rng_streams      # RNG stream-id packing and derivation; no rendering involved
)

# --- any render backend -----------------------------------------------------------
# These exercise the public sensor API only, with no OptiX header and no OptiX-only buffer
# format, so they run on OptiX, Vulkan RT or Metal RT alike. They cannot go in the list
# above: they use the camera, lidar and radar buffer types, which ChSensorBuffer.h declares
# only inside its per-backend blocks, so with no backend they do not compile.
if(CH_SENSOR_HAS_RENDER_BACKEND)
  set(TESTS ${TESTS}
    utest_SEN_data_access      # background colour reaches the user buffer intact
    utest_SEN_interface        # sensor bookkeeping + dynamic scene rebuild (box/sphere/cyl/mesh)
    utest_SEN_threadsafety     # buffer ownership handover out of the filter graph
    utest_SEN_radar            # radar clustering utilities
    utest_SEN_analytic_render  # rendered images vs closed-form ground truth
    utest_SEN_scene_lights     # Modify*Light reproduces Add*Light, for every light type
  )
endif()

# --- any render backend, and the vehicle module ------------------------------------
# Built separately only because it is the one sensor test that links Chrono::Vehicle;
# adding those libraries to the list above would link them into every test here.
set(VEHICLE_TESTS "")
if(CH_SENSOR_HAS_RENDER_BACKEND AND CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
  set(VEHICLE_TESTS
    utest_SEN_dynamic_sensors  # GPS/IMU/tachometer still work alongside a render backend
  )
endif()

# --- Metal RT specific ------------------------------------------------------------
if(CH_USE_SENSOR_METAL_RT)
  set(TESTS ${TESTS}
    utest_SEN_metal_stochastic  # tier 2: convergence / variance of the stochastic features
  )
endif()

# --- OptiX specific ---------------------------------------------------------------
# utest_SEN_optix{engine,geometry,pipeline} drive OptiX pipeline internals directly.
# utest_SEN_camera_convergence asks for a ChFilterRGBA16Access buffer and exercises
# Integrator::PATH plus ChPhysCameraSensor supersampling; the RGBA16 camera format and
# the PATH integrator are OptiX-only today, so the Metal build gets its own equivalent
# (utest_SEN_metal_stochastic) rather than a weakened copy of this one.
if(CH_USE_SENSOR_OPTIX)
  set(TESTS ${TESTS}
    utest_SEN_camera_convergence
    utest_SEN_optixengine
    utest_SEN_optixgeometry
    utest_SEN_optixpipeline
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

if(VEHICLE_TESTS)
  build_utests(YES "${VEHICLE_TESTS}" "sensor" "${LIBS};Chrono_vehicle;ChronoModels_vehicle")
endif()

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

