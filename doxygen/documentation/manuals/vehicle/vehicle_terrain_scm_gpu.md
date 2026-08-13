## SCM GPU backends {#vehicle_terrain_scm_gpu}

Two optional HIP backends move the per-node-independent stages of an SCM step onto the GPU: ray casting (one ray per active grid node against vehicle collision geometry) and the dense Bekker / Mohr-Coulomb / Janosi contact-force loop in [SCMLoader::ComputeInternalForces()](@ref chrono::vehicle::SCMLoader::ComputeInternalForces). Contact-patch clustering and bulldozing remain on the CPU.

The HIP device code is built **into** `Chrono_vehicle` when enabled (same pattern as Chrono::DEM / Chrono::FSI::SPH). No external library or extra install step is required.

### Prerequisites

- A HIP toolchain detected by Chrono
- An AMD GPU (validated on gfx942 / MI300X and gfx90a / MI210), or an NVIDIA GPU through HIP's NVIDIA platform
- OpenMP-enabled host build (host-side packing and scatter remain OpenMP)

### CMake configuration

`CH_ENABLE_VEHICLE_SCM_GPU` defaults to `ON`, so no flags are required when a HIP toolchain and a supported GPU are present. To pin a target architecture:

```bash
cmake -S chrono -B build \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCHRONO_HIP_ARCHITECTURES=gfx942
```

If no HIP toolchain is found, the feature is reported as unavailable in the configuration status and the CPU SCM path configures and builds as before.

When enabled, `ChConfigVehicle.h` defines `CHRONO_HAS_SCM_GPU`.

### Runtime

Both backends are enabled by default in a build that has them, and each falls back to the CPU on its own when a model cannot use it. No code change is needed to run either way.

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

| Setting | Default | Role |
|---------|---------|------|
| `enabled` | `true` | Use GPU when eligibility checks pass |
| `min_hits` | 8192 | CPU fallback below this hit count |
| `reserve_hits` | 65536 | Pre-allocated hit buffer capacity |
| `async` | `true` | Async HIP streams for pack / compute / scatter |
| `profile` | `false` | Log pack / gpu / scatter timings to stderr |

The ray-cast kernels run in FP64 on AMD, and in FP32 on NVIDIA where consumer parts throttle double precision. Setting the environment variable `SCM_RAYCAST_GPU_PRECISION` to `fp32` or `fp64` overrides that choice; it is intended for validation.

### NVIDIA GPUs

These kernels exist only in HIP. On NVIDIA hardware they compile through HIP's NVIDIA platform, where the HIP compiler is nvcc; no CUDA port of the kernels is involved. This is why `CHRONO_ENABLE_HIP_ON_NVIDIA` defaults to `ON`: for this feature, "is HIP available" and "can this use the GPU" are the same question. Chrono::DEM and Chrono::FSI::SPH still prefer CUDA and still resolve to CUDA on NVIDIA.

### Validation

Before production use, run parity and vehicle smoke tests on your target GPU. Target per-hit tolerance rtol `1e-5`, atol `1e-7` at ≥ 65k hits.

The GPU and CPU ray-cast paths do not produce identical hit sets: Bullet tests rays against a convex-decomposition approximation of a collision mesh, while these kernels test the mesh triangles directly. A small difference in contact forces is therefore expected; a large one indicates a problem.
