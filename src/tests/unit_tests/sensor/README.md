# Chrono::Sensor unit tests

```bash
ctest --test-dir build -L sensor                    # run them all
ctest --test-dir build -L sensor -N                 # list without running
ctest --test-dir build -R utest_SEN_analytic_render --output-on-failure
```

## What is here

The tests are split by **what they actually depend on**, not by which backend happens to be the
historical default. Anything that only touches the public sensor API (`ChSensorManager`,
`ChSensor`, the filter graph, the buffer types, `ChOptixScene`) runs on whichever ray-tracing backend
the build selected.

| test | scope |
|---|---|
| `utest_SEN_gps` | GPS coordinate conversion |
| `utest_SEN_data_access` | background colour reaches the user buffer intact |
| `utest_SEN_interface` | sensor bookkeeping and scene rebuild |
| `utest_SEN_threadsafety` | buffer ownership handover out of the filter graph |
| `utest_SEN_radar` | radar clustering utilities |
| `utest_SEN_analytic_render` | analytic ground truth for the ray tracer (see below) |
| `utest_SEN_dynamic_sensors` | accelerometer / gyroscope / magnetometer / GPS / tachometer on a driving vehicle |
| `utest_SEN_metal_stochastic` | convergence of the stochastic features (Metal RT builds) |
| `utest_SEN_camera_convergence`, `utest_SEN_optix{engine,geometry,pipeline}` | OptiX-specific |

`utest_SEN_analytic_render` and `utest_SEN_dynamic_sensors` print their measurements as they go, so a
failing run says what was measured and not merely that something was wrong. They are ordinary
googletest programs and go through `build_utests` like every other test here.

## Why the render tests assert against maths, not against another renderer

Two GPU ray tracers can never agree bit-for-bit. They use different RNG streams, different float
and transcendental precision, different BVH edge-case handling and different texture samplers.
Comparing a render against a reference image produced by another backend, or against images
blessed on other hardware, therefore fails for reasons that say nothing about correctness.

So `utest_SEN_analytic_render` derives every expected value from the scene geometry and the pinhole
camera model, using an independent double-precision ray/AABB intersector, and asserts only on
channels that carry no stochastic shading:

| channel | expectation |
|---|---|
| depth | distance to the first surface, to float precision |
| surface normals | exact on axis-aligned faces |
| segmentation | integer class/instance ids, exact equality |
| lidar / radar range | to float precision |
| projection | where a known point lands in pixels, per the pinhole model |

A failure means the backend is genuinely wrong, not merely different, and the test is equally
valid on OptiX, Vulkan RT and Metal RT. It is how the depth/normal/segmentation cameras were
found to be ignoring `GetHFOV()` and rendering at a hard-coded fallback.

Stochastic features (global illumination, area-light soft shadows, depth of field, sensor
noise) cannot be asserted this way. `utest_SEN_metal_stochastic` follows the pattern set by
`utest_SEN_camera_convergence` instead and asserts *statistically* (variance falls as
supersampling rises; a soft shadow's penumbra is wider than a hard one's), which is robust to
RNG differences by construction.

## A trap worth knowing

Chrono resolves its data directory **relative to the working directory**. A test launched from
anywhere other than `build/bin` silently loses every texture and HDR map. That changes only the
shaded output, so it presents as a rendering regression rather than a missing-file error. CTest
pins `WORKING_DIRECTORY` for exactly this reason; keep it that way for any test that loads assets.
