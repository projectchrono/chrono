# Building the SCM GPU backend for NVIDIA GPUs

Scope: this guide is about **`chrono_vehicle`'s SCM (deformable terrain) GPU backend** added on this
branch (`feat/scm-gpu`) -- both the original contact-force HIP kernel and the newer ray-cast HIP kernel
(ray-casting the active-domain grid against wheel collision meshes is ~90% of the SCM step, the part
that actually dominates runtime). It is **not** a general "Chrono on NVIDIA" guide -- Chrono's other GPU
modules (`chrono_gpu`, `chrono_fsi`) have their own native CUDA paths via `CHRONO_GPU_BACKEND=CUDA`; see
the [core install guide](https://api.projectchrono.org/tutorial_install_chrono.html#gpu) for those.

This backend is **HIP-only**: there is no separate CUDA source tree for it. Reaching NVIDIA hardware
means building through HIP's NVIDIA platform (a thin wrapper that compiles the same HIP source through
`nvcc`), not porting the kernel logic.

---

## 1. Why this should work at all

Everything in this backend is portable HIP: runtime API calls (`hipMalloc`, `hipMemcpy`, `hipStream_t`,
kernel launch syntax) and otherwise-ordinary CUDA-style device code (`__global__`, `__shared__`,
`blockIdx`/`threadIdx`). Nothing uses AMD-only intrinsics or ROCm-only libraries. On HIP's NVIDIA
platform, `hip_runtime.h` maps onto the CUDA runtime and `hipcc` compiles through `nvcc`, so the same
`.hip.cpp` sources validated on an AMD MI300X (gfx942) should compile and run unchanged on an NVIDIA
GPU. That claim has **not** been tested on real NVIDIA hardware yet -- that's what this guide is for.

## 2. FP32 vs FP64

Consumer NVIDIA GPUs (e.g. RTX 4080/5090) deliberately throttle double-precision throughput relative to
single precision, unlike MI300X (a proper datacenter part with strong FP64). The ray-cast kernel is
templated on `Real` (`float` or `double`); the build defaults to FP32 when compiled against HIP's NVIDIA
platform (detected via `__HIP_PLATFORM_NVIDIA__`) and FP64 on AMD. Override either way at runtime:

```bash
export SCM_RAYCAST_GPU_PRECISION=fp32   # or fp64
```

Both precisions have been validated against a CPU reference implementation on AMD hardware (FP32: 0 hit
diff, per-wheel force error 0-0.12%, well under this project's pass threshold). They have not yet been
run on NVIDIA hardware -- see "What to report back" below.

## 3. Prerequisites

1. **NVIDIA CUDA toolkit** (`nvcc` on `PATH`) -- required; HIP's NVIDIA platform compiles through it.
2. **HIP built/installed for the NVIDIA platform.** This is *not* the same as a normal ROCm install
   (which targets AMD only). Check current instructions at
   [ROCm/HIP](https://github.com/ROCm/HIP) for building/installing HIP with `HIP_PLATFORM=nvidia` --
   this has varied across ROCm releases, so don't rely on a specific package name from memory.
3. Whichever path you use, you need `<install>/include/hip/hip_version.h` to exist. This repo's CMake
   (`chrono_find_rocm()` in `src/CMakeLists.txt`) looks for it via, in order: `CHRONO_ROCM_ROOT`,
   `ROCM_PATH`, `$ROCM_PATH`, `$ROCM_HOME`, `$HIP_PATH`, then `/opt/rocm`, `/usr/lib/rocm`,
   `/usr/local/rocm`. Point one of these at your HIP-for-NVIDIA install if it isn't in a default
   location.

## 4. Configure and build

```bash
git clone <this fork/branch's URL> -b feat/scm-gpu chrono
cd chrono

export HIP_PATH=/path/to/your/hip-nvidia-install   # only if not auto-discovered

cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_MODULE_ROBOT=ON \
  -DCH_ENABLE_MODULE_IRRLICHT=OFF \
  -DCH_ENABLE_VEHICLE_SCM_GPU=ON \
  -DCHRONO_GPU_BACKEND=HIP \
  -DCMAKE_HIP_PLATFORM=nvidia

ninja -C build demo_ROBOT_Viper_SCM
```

(`CH_ENABLE_MODULE_IRRLICHT=OFF` keeps this to a headless build; drop it if you want visualization and
have Irrlicht/VSG available.)

**On `CHRONO_HIP_ARCHITECTURES`:** on AMD this is a `gfx` code (e.g. `gfx942`). Under HIP's NVIDIA
platform the equivalent concept is a CUDA compute-capability (`sm_XX`) string (RTX 4080 / Ada Lovelace =
`sm_89`; check `nvidia-smi` or NVIDIA's compute-capability table for your card, especially newer ones).
Leave `CHRONO_HIP_ARCHITECTURES` unset first -- CMake >= 3.21 defaults to `native`, which should
auto-detect your card -- and only hand-pick an architecture string if that fails.

## 5. Running

```bash
export LD_LIBRARY_PATH=$(pwd)/build/lib:$LD_LIBRARY_PATH

# production demo, GPU ray-cast + GPU contact-force backends both on
SCM_RAYCAST_GPU=hip ./build/bin/demo_ROBOT_Viper_SCM

# force FP64 to see whether NVIDIA's throttled FP64 costs real time here
SCM_RAYCAST_GPU=hip SCM_RAYCAST_GPU_PRECISION=fp64 ./build/bin/demo_ROBOT_Viper_SCM
```

Watch for the startup banner (`SCM ray-cast backend: HIP` / `precision: ...`) and the final
`[BENCH] steps=... (X ms/step, avg RTF=Y)` line.

## 6. What to report back

- Whether it configures/builds at all, and which HIP-for-NVIDIA install path got you there.
- The `[BENCH] ...` line from `demo_ROBOT_Viper_SCM` with `SCM_RAYCAST_GPU=hip`, both FP32 (default) and
  FP64 -- ms/step and RTF, so they can be compared against the AMD MI300X baseline (ray-casting
  ~9.9ms -> ~0.54ms, full step ~12.4ms -> ~2.6ms, RTF ~24.8 -> ~5.2).
- Whether per-wheel contact forces look physically reasonable (no crash, no NaN/zero-everywhere).

## 7. Troubleshooting

| Symptom | Likely cause |
|---|---|
| CMake: "SCM GPU requested but HIP/ROCm was not found" | `chrono_find_rocm()` didn't find `include/hip/hip_version.h`. Set `HIP_PATH` or `ROCM_PATH` to your HIP-for-NVIDIA install root. |
| `check_language(HIP)` fails / no HIP compiler found | `hipcc` isn't on `PATH`, or it isn't actually configured for the NVIDIA platform. Try `hipcc --version` and confirm it reports an NVIDIA/CUDA target. |
| Forces are far off vs CPU/Bullet -- more than a small (roughly single-digit percent) gap | That gap is expected and explained (Bullet collides against an HACD convex-decomposition approximation of the wheel, not the raw mesh -- see background docs below); a *much* larger gap would point at an actual bug, not this known effect. |
| Crash, NaN, or all-zero forces | Something is actually broken on this platform -- worth reporting with the exact build flags and NVIDIA driver/CUDA version used. |

## 8. Background

The design rationale, AMD MI300X benchmark numbers, and the convex-decomposition finding that explains
the expected CPU-vs-GPU force gap live in this project's planning notes (`SCM_RAYCAST_GPU_PLAN.md`,
`SCM_GPU_BENCHMARK.md`), which are outside this repository -- ask whoever handed you this branch for a
copy if you want the full history behind these numbers.

Relevant source in-repo:

- Kernel: `src/chrono_vehicle/terrain/gpu/SCMRaycastGpuKernels.hip.cpp`
- Host-side buffer/precision management: `src/chrono_vehicle/terrain/gpu/SCMRaycastGpuHost.cpp`
- Precision selection logic: `src/chrono_vehicle/terrain/SCMTerrainRaycastGpu.cpp`
- Public API: `src/chrono_vehicle/terrain/SCMRaycastGpu.h`, `SCMRaycastGpuTypes.h`
