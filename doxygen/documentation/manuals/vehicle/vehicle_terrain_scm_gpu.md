## SCM GPU backends {#vehicle_terrain_scm_gpu}

Two optional GPU paths move the per-node-independent stages of an SCM step onto the GPU: ray casting (one ray per active grid node against vehicle collision geometry) and the dense Bekker / Mohr-Coulomb / Janosi contact-force loop in [SCMLoader::ComputeInternalForces()](@ref chrono::vehicle::SCMLoader::ComputeInternalForces). Contact-patch clustering and bulldozing remain on the CPU.

The shared device kernels are compiled as CUDA or HIP and built **into** `Chrono_vehicle` when enabled. No external library or extra install step is required.

### Prerequisites

- A CUDA or HIP toolchain detected by Chrono
- An NVIDIA GPU using CUDA or HIP's NVIDIA platform, or an AMD GPU using HIP (validated on gfx942 / MI300X and gfx90a / MI210)
- OpenMP-enabled host build (host-side packing and scatter remain OpenMP)

### CMake configuration

`CH_ENABLE_VEHICLE_SCM_GPU` defaults to `ON`, so no flags are required when a suitable toolchain and a supported GPU are present. `CHRONO_GPU_BACKEND=AUTO` selects CUDA when available, otherwise HIP. To select HIP and pin an AMD target architecture:

```bash
cmake -S chrono -B build \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCHRONO_GPU_BACKEND=HIP \
  -DCHRONO_HIP_ARCHITECTURES=gfx942
```

If neither toolchain is found, the feature is reported as unavailable in the configuration status and the CPU SCM path configures and builds as before.

When enabled, `ChConfigVehicle.h` defines `CHRONO_HAS_SCM_GPU`.

### Runtime

Both backends are enabled by default in a build that has them, and each falls back to the CPU on its own when a model cannot use it. No code change is needed to run either way.

To force CPU execution from a shell script, set this variable before starting the application:

```bash
CHRONO_SCM_FORCE_CPU=1 python3 my_simulation.py
# Or: CHRONO_SCM_FORCE_CPU=1 ./my_simulation
```

For an individual terrain in Python or C++, call `SetForceCPU` before `Initialize`:

```python
terrain = veh.SCMTerrain(system, False)  # veh is pychrono.vehicle
terrain.SetForceCPU(True)
terrain.Initialize(10.0, 10.0, 0.02)
```

```cpp
terrain.SetForceCPU(true);
```

This selects the default CPU ray-casting and contact-force paths, overrides the individual GPU and
GPU-reference switches, and skips GPU buffer initialization when set before `Initialize`. The API
is available in CPU-only builds as well. `IsForceCPU()` reports the override. Passing `false`
(`False` in Python) restores the underlying backend settings without changing them.
The environment variable sets the initial override for each terrain at construction; unset it or
set it to `0` to retain normal backend selection, or use `SetForceCPU(false)` for that terrain.

Contact forces run on the GPU when:

- soil parameters are uniform (no `RegisterSoilParametersCallback`)
- contactables are rigid `ChBody` only
- the hit count is at or above the minimum threshold (default **8192**)

Ray casting runs on the GPU when:

- the model registers explicit active domains ([AddActiveDomain](@ref chrono::vehicle::SCMTerrain::AddActiveDomain))
- the bodies in those domains carry triangle-mesh collision shapes, the only shape type these kernels intersect

A model built from primitives or convex hulls therefore stays on the CPU ray-cast path, which tests against the full collision system and handles every shape type. That case reports itself once on stderr rather than silently producing no hits.

Use the SCMTerrain API to control behavior:

```cpp
terrain.SetScmGpuEnabled(true);      // contact forces; default when CHRONO_HAS_SCM_GPU
terrain.EnableRaycastGpuHip(true);   // ray casting; default when CHRONO_HAS_SCM_GPU

chrono::vehicle::scm_gpu::Config cfg = terrain.GetScmGpuConfig();
cfg.min_hits = 4096;
cfg.reserve_hits = 65536;
cfg.async = true;
cfg.profile = false;
terrain.SetScmGpuConfig(cfg);
```

`EnableRaycastGpuHip` retains its existing API name and controls the compiled ray-casting backend, whether CUDA or HIP.

| Setting | Default | Role |
|---------|---------|------|
| `enabled` | `true` | Use GPU when eligibility checks pass |
| `min_hits` | 8192 | CPU fallback below this hit count |
| `reserve_hits` | 65536 | Pre-allocated hit buffer capacity |
| `async` | `true` | Async GPU streams for pack / compute / scatter |
| `profile` | `false` | Log pack / gpu / scatter timings to stderr |

The ray-cast kernels run in FP32 on every platform, so that a model produces the same trajectory on AMD and NVIDIA rather than diverging by vendor. FP64 is available via the environment variable `SCM_RAYCAST_GPU_PRECISION` (`fp32` or `fp64`), and is intended for validation.

### NVIDIA GPUs

NVIDIA hardware can use native CUDA or HIP's NVIDIA platform. Both compile the same kernel sources with nvcc; each has its own runtime host bridge. Native CUDA is selected by default when available. Set `CHRONO_GPU_BACKEND=HIP` to prefer HIP, or `CHRONO_GPU_BACKEND=CUDA` to prefer CUDA. The CPU override above applies to either compiled backend.

### Validation

Before production use, run parity and vehicle smoke tests on your target GPU. Target per-hit tolerance rtol `1e-5`, atol `1e-7` at ≥ 65k hits.

The GPU ray-cast backend does not reproduce the results of the default CPU path, which casts rays through Bullet. On `demo_ROBOT_Viper_SCM` the wheel sinks about 5.5 cm with this backend against about 4.3 cm through Bullet, consistently across grid resolutions and on both AMD and NVIDIA hardware.

This is a property of the formulation rather than of the GPU. `EnableRaycastGpuReference()`, which runs the same algorithm on the CPU, agrees with the kernels to within 0.09%; the contact-force backend accounts for about 2%, the collision margin for a few millimeters, and kernel precision for micrometers. Expect a systematic offset of this order when switching a model onto this backend, and validate against your own reference before relying on absolute sinkage.
