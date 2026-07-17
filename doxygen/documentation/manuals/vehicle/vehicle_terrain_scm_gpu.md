## SCM GPU contact-force backend {#vehicle_terrain_scm_gpu}

The optional HIP backend accelerates the dense Bekker / Mohr-Coulomb / Janosi contact-force loop in [SCMLoader::ComputeInternalForces()](@ref chrono::vehicle::SCMLoader::ComputeInternalForces). Ray casting, contact-patch clustering, and bulldozing remain on the CPU.

The HIP device code is built **into** `Chrono_vehicle` when enabled (same pattern as Chrono::DEM / Chrono::FSI::SPH). No external library or extra install step is required.

### Prerequisites

- ROCm / HIP toolchain detected by Chrono (`CHRONO_GPU_BACKEND=HIP` or `AUTO` with ROCm present)
- AMD Instinct or other HIP-capable GPU (validated on gfx942 / MI300X and gfx90a / MI210)
- OpenMP-enabled host build (SCM ray cast and pack remain OpenMP)

### CMake configuration

Enable in the Chrono::Vehicle module (same pattern as OpenCRG):

```bash
cmake -S chrono -B build \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_VEHICLE_SCM_GPU=ON \
  -DCHRONO_GPU_BACKEND=HIP \
  -DCHRONO_HIP_ARCHITECTURES=gfx942
```

If HIP/ROCm is not found, CMake prints a warning and disables SCM GPU support; the default CPU SCM path still configures and builds.

When enabled, `ChConfigVehicle.h` defines `CHRONO_HAS_SCM_GPU`.

### Runtime

When built with SCM GPU support, the HIP path is **enabled by default** whenever all eligibility checks pass:

- uniform soil parameters (no `RegisterSoilParametersCallback`)
- rigid `ChBody` contactables only
- hit count ≥ minimum threshold (default **8192**)

Use the SCMTerrain API to control behavior:

```cpp
terrain.SetScmGpuEnabled(true);   // default when CHRONO_HAS_SCM_GPU

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

### NVIDIA GPUs

This backend uses **HIP/ROCm** and targets AMD Instinct GPUs. It is **not** available on NVIDIA hardware with the current implementation.

Chrono's other GPU modules (DEM, FSI) support both CUDA (NVIDIA) and HIP (AMD) through separate code paths. A CUDA port of the SCM contact-force kernels would follow the same dual-backend pattern; that is out of scope for this PR.

### Further reading

- [docs/OPENMP_TO_HIP.md](@ref openmp_to_hip) — OpenMP→HIP porting guide (SCM reference implementation)

### Validation

Before production use, run parity and vehicle smoke tests on your target GPU. Target per-hit tolerance rtol `1e-5`, atol `1e-7` at ≥ 65k hits.
