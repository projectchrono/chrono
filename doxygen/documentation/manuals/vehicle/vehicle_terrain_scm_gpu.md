## SCM GPU contact-force backend {#vehicle_terrain_scm_gpu}

The optional HIP backend accelerates the dense Bekker / Mohr-Coulomb / Janosi contact-force loop in [SCMLoader::ComputeInternalForces()](@ref chrono::vehicle::SCMLoader::ComputeInternalForces). Ray casting, contact-patch clustering, and bulldozing remain on the CPU.

### Prerequisites

- ROCm / HIP toolchain (`hipcc`) to build the external `scm_gpu_core` library
- AMD Instinct or other HIP-capable GPU (validated on gfx942 / MI300X)
- OpenMP-enabled host build (SCM ray cast and pack remain OpenMP)

### CMake configuration

Enable in the Chrono::Vehicle module (same pattern as OpenCRG):

```bash
cmake -S chrono -B build \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_VEHICLE_SCM_GPU=ON \
  -DCHRONO_SCM_GPU_LIB_DIR=/path/to/scm_gpu_install/lib \
  -DCMAKE_HIP_ARCHITECTURES=gfx942
```

If HIP or `scm_gpu_core` is not found, CMake prints a warning and disables SCM GPU support; the default CPU SCM path still configures and builds.

When enabled, `ChConfigVehicle.h` defines `CHRONO_HAS_SCM_GPU`.

### Runtime

| Variable | Default | Role |
|----------|---------|------|
| `CHRONO_SCM_GPU` | off | `1` enables GPU contact forces |
| `CHRONO_SCM_GPU_MIN_HITS` | 8192 | CPU fallback below this hit count |
| `CHRONO_SCM_GPU_RESERVE` | 65536 | Pre-allocated hit buffer capacity |
| `CHRONO_SCM_GPU_ASYNC` | 1 | Async HIP streams for pack / compute / scatter |
| `CHRONO_SCM_GPU_PROFILE` | off | Log pack / gpu / scatter timings |

### External library

Chrono ships the API headers `SCMGpu.h` / `SCMGpuTypes.h` but does **not** vendor the HIP implementation. Build `scm_gpu_core` separately; see [docs/SCM_GPU_EXTERNAL.md](@ref scm_gpu_external) in the repository root and the OpenMP→HIP porting guide [docs/OPENMP_TO_HIP.md](@ref openmp_to_hip).

### Validation

Before production use, run parity (`scm_parity_test`) and a wheel / vehicle smoke test with `CHRONO_SCM_GPU=1`. Target per-hit tolerance rtol `1e-5`, atol `1e-7` at ≥ 65k hits.
