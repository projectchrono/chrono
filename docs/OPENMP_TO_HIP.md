# OpenMP (CPU) → HIP (GPU) porting guide

A short playbook for Chrono modules that are **CPU + OpenMP today** and need a **HIP kernel** on AMD GPUs (ROCm). The **SCM terrain contact-force** backend (`CH_ENABLE_VEHICLE_SCM_GPU`) is the reference walkthrough in this repository.

**Hardware:** AMD Instinct GPUs (e.g. gfx942 MI300X, gfx90a MI210). Use ROCm and `ROCR_VISIBLE_DEVICES` — not CUDA-only APIs.

---

## 1. Three port strategies (pick one)

| Strategy | When to use | SCM example |
|----------|-------------|-------------|
| **A. Split CPU/GPU** | OpenMP loop is mixed with I/O, callbacks, or sparse data structures | **Yes** — ray cast + BFS on CPU; force loop on GPU |
| **B. Hipify CUDA** | Module already has `.cu` kernels (e.g. FSI/SPH) | Not applicable to SCM |
| **C. OpenMP target offload** | Quick experiment; compiler-dependent on ROCm | Not used for SCM |

**Rule of thumb:** If the hot loop touches Chrono collision, callbacks, or `std::map`, **do not** put it in a kernel. Extract a **dense, embarrassingly parallel** inner loop instead.

---

## 2. Step-by-step workflow (SCM template)

### Step 0 — Inventory the CPU path

1. Find the hot loop in Chrono (for SCM: `SCMLoader::ComputeInternalForces()` in `SCMTerrain.cpp`, contact-force section).
2. List what **must stay on CPU** (ray casting, graph traversal, file I/O, virtual callbacks).
3. List what **can move to GPU** (per-element math with no pointer chasing).

**SCM split:**

```text
[CPU] Update active domains
[CPU] OpenMP ray cast → hits + patch_oob
[CPU] BFS contact patches
[CPU] Pack batch (OpenMP parallel for)
[GPU] scm_compute_forces_kernel
[CPU] Reduce per-body forces → ChLoadBodyForce
```

### Step 1 — Standalone HIP library

Before touching Chrono CMake, build and test a small HIP library:

| Artifact | Role |
|----------|------|
| `*_types.h` | SoA structs (`HitInput`, `HitOutput`, `SoilParams`) |
| `*_cpu_reference.cpp` | Copy of the CPU loop, no OpenMP inside the kernel path |
| `*_kernels.hip.cpp` | `__global__` kernel, one thread per hit |
| `*_host.cpp` | H2D, launch, D2H, buffer reuse |

Configure with `hipcc`, set `CMAKE_HIP_ARCHITECTURES` to your GPU ISA (`gfx942`, `gfx90a`, …). Host code can stay on **g++**; device code uses the HIP language.

### Step 2 — CPU reference parity

1. Mirror the Chrono loop **literally** in `*_cpu_reference.cpp` (same formulas, same order).
2. Write a parity test that feeds random SoA batches to CPU ref and HIP kernel.
3. Gate: `rtol=1e-5`, `atol=1e-7` on forces.

**Do not skip this.** OpenMP and HIP must agree on the **same math**, not “close enough.”

### Step 3 — Keep OpenMP on the host (pack / reduce)

OpenMP stays **outside** HIP kernels:

- **Pack:** `#pragma omp parallel for` over hits → fill `HitInput[]`.
- **Reduce:** per-thread partial sums of body forces on host; move reduction to device only if profiling justifies it.

```cpp
#pragma omp parallel for schedule(static)
for (std::ptrdiff_t i = 0; i < n; ++i) {
    out[i].vn = vn[i];
    // ...
}
```

Callbacks (e.g. `GetContactPointSpeed`) run on **host** before pack; store scalars in the batch.

### Step 4 — SoA layout (host ↔ device)

Use **Structure of Arrays** or one struct per hit with fixed size — no `std::vector` inside the kernel.

| Principle | SCM |
|-----------|-----|
| Fixed-width fields | `level`, `sinkage_plastic`, `normal_z`, `fn`, `ft`, … |
| Global params once | `SoilParams` (Bekker, Mohr, Janosi, `dt`, `area`) |
| Optional per-body data | `body_id`, contactable cohesion/mu pre-filled on host |
| Outputs | Updated node scalars + per-hit forces; host sums by `body_id` |

### Step 5 — Chrono integration hook

1. Add `ComputeContactForcesGpu()` (or equivalent) behind a **runtime or CMake gate** (`CH_ENABLE_VEHICLE_SCM_GPU`).
2. In the CPU function, after ray cast + patch build: pack → external `scm_gpu` launch → scatter into body forces.
3. Bridge implementation: `src/chrono_vehicle/terrain/SCMTerrainGpu.cpp`.

**SCM runtime toggle:**

```bash
export CHRONO_SCM_GPU=1          # uniform soil, rigid ChBody contactables in v1
export CHRONO_SCM_GPU_MIN_HITS=8192   # CPU fallback below this count
```

Build Chrono with `-DCH_ENABLE_VEHICLE_SCM_GPU=ON` and point `CHRONO_SCM_GPU_INCLUDE_DIR` / `CHRONO_SCM_GPU_LIB_DIR` at the external `scm_gpu` install. See [`docs/SCM_GPU_EXTERNAL.md`](docs/SCM_GPU_EXTERNAL.md).

### Step 6 — Scaling benchmark

1. **Kernel-only** benchmark with large `n_hits` (overhead excluded) — fair GPU test.
2. **End-to-end** Chrono demo — may be slower on GPU if hits/step are small (launch + sync dominate).

Target: meaningful wall-time reduction on the ported loop at **production** hit counts (SCM kernel reference: ~2.5× @ 65k hits on MI300X).

---

## 3. CUDA → HIP cheat sheet (other modules)

| CUDA | HIP / ROCm |
|------|------------|
| `cudaMalloc` | `hipMalloc` |
| `cudaMemcpy` | `hipMemcpy` |
| `__shfl_sync` | `__shfl` (warp size **64** on gfx942) |
| CUB | rocPRIM |
| cuRAND | hipRAND |
| `cudaMemcpyToSymbol` | `hipMemcpyToSymbol` + `HIP_SYMBOL(...)` |
| texture loads | `__ldg` or explicit global loads |
| `CUDA_VISIBLE_DEVICES` | **`ROCR_VISIBLE_DEVICES`** |

For FSI/SPH modules that already ship CUDA sources, prefer upstream `CHRONO_GPU_BACKEND=HIP` and compile `.cu` with the HIP toolchain rather than duplicating host/device trees.

---

## 4. Common mistakes

| Mistake | Fix |
|---------|-----|
| Hipifying OpenMP host loops | Extract inner math; keep OpenMP on pack/reduce |
| Putting ray cast / BFS on GPU | Leave on CPU in first integration |
| Sync every step with tiny batches | Profile hit count; pipeline H2D/kernel/D2H or batch more work |
| `float` on GPU, `double` in Chrono | Use `double` in v1 for parity |
| Setting `CMAKE_CXX_COMPILER=hipcc` for the whole tree | Keep host on g++; enable HIP language for device TUs only |

---

## 5. Milestone checklist (per module)

| Gate | Criterion |
|------|-----------|
| M1 | Standalone HIP library builds for target arch |
| M2 | CPU vs HIP parity test at rtol=1e-5, atol=1e-7 |
| M3 | Chrono hook + smoke demo with runtime gate |
| M4 | Scaling benchmark at realistic batch size |

---

## 6. SCM integration map (this repository)

| Path | Role |
|------|------|
| `src/chrono_vehicle/terrain/SCMTerrain.cpp` | CPU path + GPU hook after ray cast |
| `src/chrono_vehicle/terrain/SCMTerrainGpu.{h,cpp}` | Pack → `scm_gpu` → scatter |
| `CMakeLists.txt` | `CH_ENABLE_VEHICLE_SCM_GPU` option |
| External `scm_gpu` | HIP force kernel (built separately; see `SCM_GPU_EXTERNAL.md`) |

---

## 7. When to redesign before porting

- Module has **no dense inner loop** → redesign split before writing kernels.
- **Spatial soil callbacks** → host pre-fills per-hit params each step.
- **FEA / UV contactables** → extend pack/scatter on host first; GPU v1 rigid bodies only (SCM).
