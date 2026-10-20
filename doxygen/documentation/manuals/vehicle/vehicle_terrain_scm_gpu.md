## SCM GPU backends {#vehicle_terrain_scm_gpu}

Two optional GPU paths move the per-node-independent stages of an SCM step onto the GPU: ray casting (one ray per active grid node against vehicle collision geometry) and the dense Bekker / Mohr-Coulomb / Janosi contact-force loop in [SCMLoader::ComputeInternalForces()](@ref chrono::vehicle::SCMLoader::ComputeInternalForces). Contact-patch clustering and bulldozing remain on the CPU.

The shared device kernels are compiled as CUDA or HIP and built **into** `Chrono_vehicle` whenever a toolchain is found. No external library or extra install step is required, and whether SCM uses the GPU is decided at run time.

### Prerequisites

- A CUDA or HIP toolchain detected by Chrono
- An NVIDIA GPU using CUDA or HIP's NVIDIA platform, or an AMD GPU using HIP (validated on gfx942 / MI300X and gfx90a / MI210)
- OpenMP-enabled host build (host-side packing and scatter remain OpenMP)

### CMake configuration

GPU support is compiled in whenever a CUDA or HIP toolchain is found, with no flags required. `CHRONO_GPU_BACKEND=AUTO` selects CUDA when available, otherwise HIP. To select HIP and pin an AMD target architecture:

```bash
cmake -S chrono -B build \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCHRONO_GPU_BACKEND=HIP \
  -DCHRONO_HIP_ARCHITECTURES=gfx942
```

If neither toolchain is found, the configuration status reports the GPU backend as unavailable and only the CPU implementation is built. The SCMTerrain API is the same in either case. To build without GPU support even when a toolchain is available, set `CHRONO_VEHICLE_SCM_GPU_BACKEND=NONE` (`CUDA` or `HIP` force a specific backend).

When GPU support is compiled in, `ChConfigVehicle.h` defines `CHRONO_HAS_SCM_GPU`.

A CUDA build links the CUDA runtime statically, so `Chrono_vehicle` loads and runs on machines without the CUDA toolkit, an NVIDIA driver, or a GPU. A HIP build links the HIP runtime dynamically, so the ROCm runtime libraries must be installed wherever the library is loaded, even if SCM then runs on the CPU.

### Runtime

Whether each stage (ray casting and contact forces) runs on the CPU or the GPU is selected at run time, per terrain:

```cpp
using Backend = chrono::vehicle::SCMTerrain::Backend;
terrain.SetBackend(Backend::GPU);              // both stages
terrain.SetRayCastBackend(Backend::CPU);       // or each stage separately
terrain.SetContactForceBackend(Backend::AUTO);
```

```python
terrain = veh.SCMTerrain(system, False)  # veh is pychrono.vehicle
terrain.SetBackend(veh.SCMTerrain.Backend_CPU)
terrain.Initialize(10.0, 10.0, 0.02)
```

| Backend | Behavior |
|---------|----------|
| `AUTO` (default) | GPU if available; GPU contact forces also require at least `GpuSettings::min_hits` ray hits |
| `CPU` | Always the CPU implementation |
| `GPU` | GPU if available, for any number of hits; otherwise the CPU, with a one-time warning |

The GPU is available if Chrono was built with GPU support and a usable device is found at run time; [SCMTerrain::IsGpuAvailable](@ref chrono::vehicle::SCMTerrain::IsGpuAvailable) reports this (the device is probed once per process). Without one, SCM runs on the CPU and never aborts.

With the default `AUTO` backend, each stage therefore runs as follows:

| Situation | Ray casting | Contact forces |
|-----------|-------------|----------------|
| Built without GPU support, or no usable GPU at run time | CPU | CPU |
| GPU available, active domains on bodies with triangle-mesh collision shapes | GPU | GPU when the step has at least `GpuSettings::min_hits` (default 8192) ray hits, otherwise CPU |
| GPU available, no active domains or no triangle-mesh collision shapes | CPU | as in the row above |
| Location-dependent soil parameters (`RegisterSoilParametersCallback`) | as in the rows above | CPU |

The hit threshold keeps small contact patches on the CPU, where they are typically faster than the transfer overhead of the GPU path. Use `Backend::GPU` to run a stage on the GPU regardless of the number of hits, or `Backend::CPU` to keep it on the CPU.

Independently of the selected backend, a stage runs on the CPU for any step the GPU implementation does not support. Contact forces run on the GPU only when:

- soil parameters are uniform (no `RegisterSoilParametersCallback`)
- contactables are rigid `ChBody` only

Ray casting runs on the GPU only when:

- the model registers explicit active domains ([AddActiveDomain](@ref chrono::vehicle::SCMTerrain::AddActiveDomain))
- the bodies in those domains carry triangle-mesh collision shapes, the only shape type these kernels intersect

A model built from primitives or convex hulls therefore stays on the CPU ray-cast path, which tests against the full collision system and handles every shape type. That case reports itself once on stderr rather than silently producing no hits. `GetNumRaycastGpuSteps()` and `GetNumContactForceGpuSteps()` report how many steps each stage actually ran on the GPU.

GPU tuning parameters are set per terrain:

```cpp
chrono::vehicle::SCMTerrain::GpuSettings settings;
settings.min_hits = 4096;
settings.raycast_fp64 = false;
terrain.SetGpuSettings(settings);
```

| Setting | Default | Role |
|---------|---------|------|
| `min_hits` | 8192 | Minimum ray hits for GPU contact forces in `AUTO` mode |
| `reserve_hits` | 65536 | Initial capacity of the GPU contact-force hit buffers |
| `async` | `true` | Asynchronous GPU streams for the contact-force transfers |
| `profile` | `false` | Log pack / gpu / scatter timings to stderr |
| `raycast_fp64` | `false` | Run GPU ray casting in double precision |

The ray-cast kernels run in FP32 by default on every platform, so that a model produces the same trajectory on AMD and NVIDIA rather than diverging by vendor. FP64 (`raycast_fp64`) is intended for validation.

The GPU contexts are shared by all terrains in a process; terrains stepped from different threads are serialized on them.

### NVIDIA GPUs

NVIDIA hardware can use native CUDA or HIP's NVIDIA platform. Both compile the same kernel sources with nvcc; each has its own runtime host bridge. Native CUDA is selected by default when available. Set `CHRONO_GPU_BACKEND=HIP` to prefer HIP, or `CHRONO_GPU_BACKEND=CUDA` to prefer CUDA. The run-time backend selection above applies to either compiled backend.

### Validation

Before production use, run parity and vehicle smoke tests on your target GPU. Target per-hit tolerance rtol `1e-5`, atol `1e-7` at ≥ 65k hits.

The GPU ray-cast backend does not reproduce the results of the default CPU path, which casts rays through Bullet. On `demo_ROBOT_Viper_SCM` the wheel sinks about 5.5 cm with this backend against about 4.3 cm through Bullet, consistently across grid resolutions and on both AMD and NVIDIA hardware.

This is a property of the formulation rather than of the GPU. `EnableRaycastGpuReference()`, which runs the same algorithm on the CPU, agrees with the kernels to within 0.09%; the contact-force backend accounts for about 2%, the collision margin for a few millimeters, and kernel precision for micrometers. Expect a systematic offset of this order when switching a model onto this backend, and validate against your own reference before relying on absolute sinkage.
