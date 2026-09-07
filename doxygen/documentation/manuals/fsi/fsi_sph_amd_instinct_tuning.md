Chrono::FSI-SPH on AMD Instinct (ROCm / HIP) {#manual_fsi_sph_amd_instinct_tuning}
==================================================================================

\tableofcontents

Scope
-----

This page documents **ROCm / HIP tuning guidance** for Chrono::FSI-SPH demos on AMD Instinct GPUs (validated on MI300X, gfx942, and MI350X, gfx950). It covers:

- building HIP-enabled FSI-SPH demos
- running the seven standard benchmark demos from the command line
- environment variables and demo CLI flags that improve wall-clock time
- per-demo recommended settings and qualitative performance guidance

No Chrono physics source changes are required — tuning uses ROCm runtime variables, demo CLI options, and optional HIP compiler flags when rebuilding `Chrono_fsisph`.

Prerequisites
-------------

- ROCm toolkit with `hipcc` (ROCm 6.x / 7.x tested)
- Chrono configured with FSI-SPH and HIP enabled (`BUILD_DEMOS=ON`)
- GPU architecture set at configure time, e.g. `-DCMAKE_HIP_ARCHITECTURES=gfx942` (MI300X) or `gfx950` (MI350X)

Build FSI-SPH demos
-------------------

From a Chrono source tree:

~~~{.bash}
export ROCM_PATH=${ROCM_PATH:-/opt/rocm}

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DEMOS=ON \
  -DENABLE_MODULE_FSI=ON \
  -DCHRONO_HIP=ON \
  -DCMAKE_HIP_ARCHITECTURES=gfx942

cmake --build build --target demo_FSI-SPH_DamBreak demo_FSI-SPH_Kernels \
  demo_FSI-SPH_ObjectDrop demo_FSI-SPH_WaveTank demo_FSI-SPH_BaffleFlow \
  demo_FSI-SPH_AngleRepose demo_FSI-SPH_RassorDrum -j $(nproc)
~~~

Run demos from the **build directory** so relative data paths (`../data/`) resolve correctly:

~~~{.bash}
cd build
export LD_LIBRARY_PATH=$PWD/lib:${ROCM_PATH}/lib:${ROCM_PATH}/lib64:${LD_LIBRARY_PATH}
~~~

Demo binaries and sources
---------------------------

| Demo | Binary | Source |
|------|--------|--------|
| DamBreak | `demo_FSI-SPH_DamBreak` | `src/demos/fsi/sph/demo_FSI-SPH_DamBreak.cpp` |
| Kernel | `demo_FSI-SPH_Kernels` | `src/demos/fsi/sph/demo_FSI-SPH_Kernels.cpp` |
| ObjectDrop | `demo_FSI-SPH_ObjectDrop` | `src/demos/fsi/sph/demo_FSI-SPH_ObjectDrop.cpp` |
| WaveTank | `demo_FSI-SPH_WaveTank` | `src/demos/fsi/sph/demo_FSI-SPH_WaveTank.cpp` |
| BaffleFlow | `demo_FSI-SPH_BaffleFlow` | `src/demos/fsi/sph/demo_FSI-SPH_BaffleFlow.cpp` |
| AngleRepose | `demo_FSI-SPH_AngleRepose` | `src/demos/fsi/sph/demo_FSI-SPH_AngleRepose.cpp` |
| RassorDrum | `demo_FSI-SPH_RassorDrum` | `src/demos/fsi/sph/demo_FSI-SPH_RassorDrum.cpp` |

Running examples (baseline)
---------------------------

Common pattern for fluid demos (no visualization, quiet terminal):

~~~{.bash}
cd build
./bin/demo_FSI-SPH_DamBreak --no_vis --quiet
~~~

**Per-demo baseline commands:**

~~~{.bash}
# Fluid CFD demos
./bin/demo_FSI-SPH_DamBreak    --no_vis --quiet
./bin/demo_FSI-SPH_ObjectDrop  --no_vis --quiet
./bin/demo_FSI-SPH_WaveTank    --no_vis --quiet
./bin/demo_FSI-SPH_BaffleFlow  --no_vis --quiet

# Micro-benchmark (no extra flags)
./bin/demo_FSI-SPH_Kernels

# CRM demos
./bin/demo_FSI-SPH_AngleRepose --no_vis --output false
./bin/demo_FSI-SPH_RassorDrum  --no_vis
~~~

Each demo prints `Simulation time: …` at the end. For benchmarking, compare **wall clock** (time(1) or `/usr/bin/time`) against simulation time.

Recommended baseline ROCm environment
-------------------------------------

Use these defaults before applying demo-specific tuning:

~~~{.bash}
export HSA_XNACK=0
export HIP_LAUNCH_BLOCKING=0
export GPU_MAX_HW_QUEUES=2
export ROCR_VISIBLE_DEVICES=0
export HIP_VISIBLE_DEVICES=0
~~~

Recommended tuning by demo
--------------------------

| Demo | Primary lever | Notes |
|------|---------------|-------|
| DamBreak | Skip I/O + `ps_freq=2` | Fluid demo; search + output dominate default wall time |
| Kernel | `HSA_ENABLE_SDMA=0` | Latency-bound micro-benchmark; strongest on MI350-class GPUs |
| ObjectDrop | `-ffast-math` HIP rebuild | Force-loop compile tuning |
| WaveTank | Skip output only | Use `--output_particle_data false`; do not use `ps_freq=2` |
| BaffleFlow | Skip I/O + `ps_freq=2` | Same pattern as DamBreak |
| AngleRepose | Combined runtime + compile + CLI | Long CRM run; stack queues, SDMA, rebuild, and CLI |
| RassorDrum | Baseline / data path | HBM-bound; verify BCE load and build-dir data path |

Expected improvement (typical ranges)
-------------------------------------

Approximate **wall-clock reduction vs default demo settings** on AMD Instinct. Ranges are indicative — profile on your hardware.

**Per optimization knob**

| Knob | Typical range |
|------|----------------:|
| Skip particle output | 5–15% |
| `--ps_freq 2` | 5–15% |
| Output off + `ps_freq 2` (fluid, where stable) | 10–25% |
| `HSA_ENABLE_SDMA=0` (Kernel) | 0–40% |
| `GPU_MAX_HW_QUEUES=4` | 0–5% |
| HIP rebuild: `-O3` | 0–5% |
| HIP rebuild: `-ffast-math` | 2–8% |
| HIP rebuild: inline-all | 3–8% |
| HIP rebuild: fast-math + inline | 5–12% |
| Combined runtime + compile + CLI | 8–20% |

**Per demo (recommended recipe)**

| Demo | Typical range |
|------|----------------:|
| DamBreak | 10–25% |
| Kernel | 0–40% |
| ObjectDrop | 3–10% |
| WaveTank | 0–8% |
| BaffleFlow | 10–20% |
| AngleRepose | 5–15% |
| RassorDrum | 0–2% |

**Unified profile:** 5–15% on fluid demos; 0–2% on RassorDrum.

Why these changes are faster
----------------------------

FSI-SPH wall time on Instinct is usually split between **GPU kernels** (force integration, neighbor search, activity updates), **CPU orchestration** (MBS coupling, I/O), and **ROCm runtime overhead** (launches, copies, queues). Each tuning knob targets a different slice of that timeline.

| Bottleneck removed | Tuning knob | Why wall clock drops |
|--------------------|-------------|----------------------|
| Host I/O and D2H sync | `--output false` / `--output_particle_data false` | Default demos periodically pack marker state on GPU, transfer to host, and write CSV files. That stalls the GPU pipeline and consumes CPU cycles. Disabling output removes those transfers and file writes entirely. |
| O(N) neighbor grid rebuilds | `--ps_freq 2` | Each proximity search sorts particles into a grid and rebuilds neighbor lists (`SphCollisionSystem`). At default cadence this runs every step; with `--ps_freq 2` it runs every other step. Fluid demos spend a large fraction of time in search + sort — reducing rebuild frequency lowers that subsystem cost. |
| SDMA vs compute contention | `HSA_ENABLE_SDMA=0` | ROCm can route small HSA memory copies through the **SDMA** engine. For tiny, latency-bound kernels (Kernel demo), SDMA setup and engine switching can exceed the copy cost. Forcing copies through the compute path avoids SDMA launch overhead on micro-benchmark-style workloads. |
| Kernel launch latency | `GPU_MAX_HW_QUEUES=4` | SPH advances many short HIP kernels per step. Deeper hardware queue depth lets ROCm queue more work while prior kernels still run, improving overlap between MBS threads and GPU submission. |
| Instruction throughput in force loops | `-O3 -ffast-math` | SPH force accumulation is FP-heavy with reductions over neighbors. `-ffast-math` lets LLVM reassociate adds/multiplies and vectorize more aggressively on CDNA. |
| Launch + call overhead | `-mllvm -amdgpu-early-inline-all=true` | Many SPH device functions are small. Early inlining merges them into fewer kernel entry points, cutting launch count and register spill traffic. |
| PCIe / page-fault path | `HSA_XNACK=0` (baseline) | Keeps unified-memory page faults off the critical path — recommended default for discrete Instinct cards. |
| Sync debugging overhead | `HIP_LAUNCH_BLOCKING=0` (baseline) | Async launches allow CPU and GPU to progress in parallel; blocking waits after every kernel inflate wall time. |

**Why some demos see little benefit**

- **RassorDrum:** Large marker counts and long horizons saturate **HBM bandwidth** on the force kernels. Once memory-bound, env/CLI tweaks hit a bytes/s roofline.
- **Kernel on MI300X:** Very short total runtime — GPU already saturated; SDMA routing changes have limited headroom.
- **WaveTank + `ps_freq=2`:** Reduces search work but can break WCSPH stability — the solver aborts; not a usable tradeoff.

Tuning catalog
--------------

### 1. Skip particle output (demo CLI)

Disables CSV / particle data collection and associated host sync.

| Demo | CLI flag |
|------|----------|
| DamBreak, ObjectDrop, BaffleFlow | `--output false` |
| WaveTank | `--output_particle_data false` |
| AngleRepose | `--output false` |

**Why it helps:** Each output interval the solver gathers per-particle fields from device memory, formats them on the CPU, and writes disk. That introduces **device→host copies** and **host-side serialization** that do not advance the simulation. On long fluid runs, I/O can be a significant fraction of wall time when left enabled.

**Example (DamBreak):**

~~~{.bash}
./bin/demo_FSI-SPH_DamBreak --no_vis --quiet --output false
~~~

**Effect:** Largest win on long fluid demos when combined with reduced neighbor-search frequency (typical **10–25%** stacked).

### 2. Reduced proximity-search frequency (`ps_freq=2`)

Maps to `ChFsiFluidSystemSPH::SetNumProximitySearchSteps(2)` — neighbor lists rebuild every two steps instead of every step. In `ChFsiFluidSystemSPH::OnDoStepDynamics`, rebuild happens when `m_frame % num_proximity_search_steps == 0`. See also @ref manual_fsi_sph_parameter_selection.

~~~{.bash}
./bin/demo_FSI-SPH_DamBreak --no_vis --quiet --output false --ps_freq 2
./bin/demo_FSI-SPH_BaffleFlow --no_vis --quiet --output false --ps_freq 2
~~~

**Why it helps:** Neighbor search is **O(N)** with sorting and hash-grid construction on GPU (`SphCollisionSystem`). DamBreak-class problems rebuild large lists every step by default. Running search every second step reduces that subsystem cost while force integration still runs each step — validate physics for your case.

**Effect:** Major fluid-demo contributor (**5–15%** search cadence; **10–25%** with I/O off).

**Caveat:** Do **not** use `ps_freq=2` on `demo_FSI-SPH_WaveTank` — WCSPH can abort with a force error because stale neighbors break the pressure solve. Use output skipping only on WaveTank.

### 3. Disable SDMA (`HSA_ENABLE_SDMA=0`)

Routes small HSA copies through the compute engine instead of the SDMA engine.

~~~{.bash}
export HSA_ENABLE_SDMA=0
./bin/demo_FSI-SPH_Kernels
~~~

**Why it helps:** Instinct exposes separate **SDMA** (async DMA) and **compute** engines. For micro-benchmarks with many tiny transfers, submitting SDMA jobs adds engine scheduling overhead that can exceed the transfer time. Disabling SDMA forces the runtime to use compute-engine memcpy paths that align better with back-to-back kernel launches.

**Effect:** Strongest on MI350-class Kernel (**25–40%**); limited on MI300X fluid demos (**0–5%**).

### 4. Hardware queue depth (`GPU_MAX_HW_QUEUES=4`)

~~~{.bash}
export GPU_MAX_HW_QUEUES=4
~~~

**Why it helps:** ROCm exposes multiple hardware queues per device. SPH submits bursts of kernels (activity, search, forces, integration). With only two queues, submission can block while the GPU drains prior work. Four queues improve **latency hiding** between dependent kernel chains and allow the host thread to stay ahead of the GPU on long runs (AngleRepose, combined profiles).

**Effect:** Modest alone (**0–5%**); useful stacked (**8–20%**).

### 5. HIP compile flags (requires rebuild)

Pass extra flags when compiling `Chrono_fsisph` and demos:

| Tier | `CMAKE_HIP_FLAGS` append | Why it helps | Typical range |
|------|--------------------------|--------------|---------------:|
| Release | `-O3` | Better loop unrolling and scheduling in force kernels | 0–5% |
| Fast math | `-O3 -ffast-math` | Relaxes FP ordering so LLVM can vectorize SPH accumulations | 2–8% |
| Inline-all | `-O3 -mllvm -amdgpu-early-inline-all=true` | Inlines small `__device__` helpers → fewer kernel boundaries | 3–8% |
| Combined | fast-math + inline-all | Both instruction-level and launch-overhead wins | 5–12% |

Example reconfigure:

~~~{.bash}
cmake -S . -B build -DCMAKE_HIP_FLAGS="-O3 -ffast-math -mllvm -amdgpu-early-inline-all=true"
cmake --build build --target demo_FSI-SPH_ObjectDrop -j $(nproc)
~~~

**Tradeoff:** `-ffast-math` relaxes IEEE semantics — validate against your accuracy requirements.

**Why compile tuning barely moves RassorDrum:** The drum demo is **memory-bandwidth saturated** — kernels already keep HBM busy. Faster instructions do not increase throughput when the roofline limit is bytes/s, not FLOPS.

Per-demo recommended commands
-----------------------------

Copy-paste examples after `cd build` and setting `LD_LIBRARY_PATH` as above.

### DamBreak (fluid — I/O + search cadence)

~~~{.bash}
export HSA_XNACK=0 HIP_LAUNCH_BLOCKING=0
./bin/demo_FSI-SPH_DamBreak --no_vis --quiet --output false --ps_freq 2
~~~

### Kernel (MI350X — disable SDMA)

~~~{.bash}
export HSA_ENABLE_SDMA=0
./bin/demo_FSI-SPH_Kernels
~~~

### ObjectDrop (compile — fast math)

Rebuild with `-O3 -ffast-math`, then:

~~~{.bash}
./bin/demo_FSI-SPH_ObjectDrop --no_vis --quiet --output false
~~~

### WaveTank (I/O only — no ps_freq=2)

~~~{.bash}
./bin/demo_FSI-SPH_WaveTank --no_vis --quiet --output_particle_data false
~~~

For MI300X compile tuning, rebuild with `-mllvm -amdgpu-early-inline-all=true`.

### BaffleFlow (fluid — I/O + search cadence)

~~~{.bash}
./bin/demo_FSI-SPH_BaffleFlow --no_vis --quiet --output false --ps_freq 2
~~~

### AngleRepose (combined stack + rebuild)

~~~{.bash}
export GPU_MAX_HW_QUEUES=4 HSA_ENABLE_SDMA=0 HSA_XNACK=0 HIP_LAUNCH_BLOCKING=0
# rebuild Chrono_fsisph with -O3 -ffast-math -mllvm -amdgpu-early-inline-all=true first
./bin/demo_FSI-SPH_AngleRepose --no_vis --output false --ps_freq 2
~~~

### RassorDrum (HBM-bound — verify data path)

Ensure wheel BCE particles load (log should report a non-zero BCE count). Correct `../data/` path dominates over env/CLI tuning.

~~~{.bash}
./bin/demo_FSI-SPH_RassorDrum --no_vis
~~~

Unified profile (one binary, all demos)
---------------------------------------

When a **single rebuild** must run all seven demos, combine:

~~~{.bash}
export GPU_MAX_HW_QUEUES=4
export HSA_ENABLE_SDMA=0
export HSA_XNACK=0
export HIP_LAUNCH_BLOCKING=0
# Rebuild with: -O3 -ffast-math -mllvm -amdgpu-early-inline-all=true
~~~

Then per demo:

- Fluid demos (except WaveTank): add `--output false` (or `--output_particle_data false` for WaveTank) and `--ps_freq 2`
- WaveTank: **omit** `--ps_freq 2`
- Kernel: no extra CLI flags; keep `HSA_ENABLE_SDMA=0`

Example tuning env files are in `docs/fsi_sph_tuning/` (`.env.example` snippets you can `source` before running).

Validation notes
----------------

- **Physics acceptance:** Reduced `ps_freq` changes neighbor-search cadence — confirm results against your reference time history before production use.
- **WaveTank:** Never combine WCSPH with `ps_freq=2` on Instinct in the tested configuration.
- **RassorDrum:** Performance is memory-bandwidth limited; run from the build directory so Chrono data files resolve.
- **Multi-arch:** Build separate trees per `CMAKE_HIP_ARCHITECTURES`; do not mix gfx942 libraries with gfx950 binaries.

See also
--------

- @ref manual_fsi_sph_parameter_selection — SPH parameter semantics (`ps_freq`, viscosity, time step)
- @ref manual_fsi_sph_class_guide — FSI-SPH class layout
- `docs/FSI_SPH_AMD_TUNING.md` — plain-Markdown copy of this guide
