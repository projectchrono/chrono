# SCM scaling benchmarks

Three Google Benchmark programs bracketing the range SCM is used over, and the recorded baseline for
them. The numbers below exist so a change to SCM can be judged against something without first
re-running six variants on four toolchains.

| | test | patch | spacing | deformed nodes |
|---|---|---|---|---|
| small | `btest_VEH_wheelSCM` | 10 x 1 m | 0.02 / 0.01 m | 1.7k / 7.2k |
| medium | `btest_VEH_hmmwvSCM` | 50 x 50 m | 0.05 m | 5.4k |
| large | `btest_VEH_largeSCM` | 300 x 300 m | 0.02 m | 34.6k / 3.63M |

Node counts are the GPU path's; the CPU loop deforms more (see Work per step). All three declare the same 1.0 x 0.3 x 1.0 m wheel active domains, run with the SCM visualization
mesh off, and report SCM's own per-step timers as counters in ms/step.

# Baseline

i7-13700K (16 cores / 24 threads, 30 MiB L3), RTX 4080, 62 GB. Release, `-O3 -march=native`,
Chrono OpenMP threads 4, container `atk/chrono:orb`. GCC 11.4.0 and AMD clang 22.0 (ships with
ROCm 7.2.4). CUDA 13.2.78; HIP columns are ROCm 7.2.4 with `CMAKE_HIP_PLATFORM=nvidia`.

GPU cells are the mean of 2 processes, `ref` and CPU of 1, each itself Google Benchmark's mean over
5 or 10 internal repetitions. Worst internal cv on `SCM_Total` was 3.21% (clang/CPU `MESH_0`);
every other cell was under 2%.

## SCM_Total, ms/step

| variant | GCC/CUDA | GCC/HIP | clang/CUDA | clang/HIP | GCC/ref | GCC/CPU | clang/CPU |
|---|---|---|---|---|---|---|---|
| `WheelSCM_D20` | 0.1172 | 0.1178 | 0.0899 | 0.0898 | 10.6478 | 3.2013 | 3.3390 |
| `WheelSCM_D10` | 0.6043 | 0.6052 | 0.3903 | 0.3863 | - | 12.3748 | 12.2767 |
| `HmmwvSCM_MESH_0` | 0.1152 | 0.1149 | 0.1063 | 0.1050 | 6.1585 | 0.3506 | 0.4024 |
| `HmmwvSCM_MESH_1` | 0.1780 | 0.1761 | 0.1649 | 0.1632 | 6.2126 | 1.6448 | 1.6400 |
| `LargeSCM_SEED0` | 0.7514 | 0.7493 | 0.6008 | 0.5918 | 34.3628 | 2.3183 | 2.2628 |
| `LargeSCM_SEED16` | 0.8704 | 0.8702 | 0.7264 | 0.7198 | 34.5411 | 2.3447 | 2.3022 |

## SCM_RayCast, ms/step

| variant | GCC/CUDA | GCC/HIP | clang/CUDA | clang/HIP | GCC/ref | GCC/CPU | clang/CPU |
|---|---|---|---|---|---|---|---|
| `WheelSCM_D20` | 0.0557 | 0.0562 | 0.0562 | 0.0565 | 10.5764 | 3.0632 | 3.2545 |
| `WheelSCM_D10` | 0.1924 | 0.1929 | 0.1912 | 0.1916 | - | 11.5155 | 11.8235 |
| `HmmwvSCM_MESH_0` | 0.0628 | 0.0629 | 0.0635 | 0.0634 | 6.0965 | 0.2925 | 0.3513 |
| `HmmwvSCM_MESH_1` | 0.1229 | 0.1214 | 0.1193 | 0.1201 | 6.1485 | 0.8998 | 1.0522 |
| `LargeSCM_SEED0` | 0.3260 | 0.3248 | 0.3288 | 0.3261 | 33.9085 | 1.8237 | 1.9392 |
| `LargeSCM_SEED16` | 0.4431 | 0.4427 | 0.4524 | 0.4516 | 34.0860 | 1.8498 | 1.9782 |

## Work per step

The columns above are only comparable where this table says they are.

| variant | path | SCM_Rays | SCM_Nodes |
|---|---|---|---|
| `WheelSCM_D20` | GPU (all four cells) | 834.0 | 1717 |
| `WheelSCM_D20` | ref | 2615.3 | 1717 |
| `WheelSCM_D20` | CPU | 2616.6 | 2414 |
| `WheelSCM_D10` | GPU (all four cells) | 3485.2 | 7162 |
| `WheelSCM_D10` | CPU | 10432.8 | 9352 |
| `HmmwvSCM_MESH_0` | GPU (all four cells) | 637.2 | 5420 |
| `HmmwvSCM_MESH_0` | ref | 646.3 | 5422 |
| `HmmwvSCM_MESH_0` | CPU | 641.6 | 5474 |
| `HmmwvSCM_MESH_1` | GPU (all four cells) | 645.2-684.1 | 5420 |
| `HmmwvSCM_MESH_1` | ref | 646.3 | 5422 |
| `HmmwvSCM_MESH_1` | CPU | 3572.3-3612.6 | 8811-9020 |
| `LargeSCM_SEED0` | GPU (all four cells) | 3995.7-3997.2 | 34557-34587 |
| `LargeSCM_SEED0` | ref | 4052.5 | 34601 |
| `LargeSCM_SEED0` | CPU | 4024.9 | 35724 |
| `LargeSCM_SEED16` | GPU (all four cells) | 3995.7-3997.2 | 3634797-3634827 |
| `LargeSCM_SEED16` | ref | 4052.5 | 3634841 |
| `LargeSCM_SEED16` | CPU | 4024.9 | 3635964 |

## Memory -- resident set after setup, MiB

| build | `SEED0` | `SEED16` | delta |
|---|---|---|---|
| GCC (any backend) | 1862.1 | 2457.0 | 594.9 |
| clang/HIP | 1864.1 | 2458.9 | 594.9 |
| clang/CUDA | 2085.1 | 2680.0 | 594.8 |

The delta is what 3.60M added nodes cost: 165 B/node, against a 128-byte `NodeRecord` plus
`unordered_map` overhead and allocator rounding. It is identical in every build; only the constant
offset moves. Most of that offset is the base-height matrix, 15001^2 entries at 8 bytes = 1716.8 MiB.

## AMD gfx942

Not yet recorded.

# Reading these numbers

- **Total step time is not a measure of SCM work.** In the wheel test the constraint solver is a
  third of it, and its spread would hide a large change in SCM cost. Use `SCM_Total` and its
  breakdown (`SCM_Domains` / `SCM_RayCast` / `SCM_Patches` / `SCM_Forces` / `SCM_Bulldoze`) from
  `ScmBenchmarkUtils.h`.

- **CUDA and HIP are interchangeable here.** Worst pair is 1.2%, most under 0.5%, and `SCM_Rays` is
  identical between them in every variant.

- **The host compiler moves `SCM_Total` but not `SCM_RayCast`.** clang is 17-35% faster on the
  former and within 1% of GCC on the latter: the ray cast is device code, everything around it is
  not.

- **`SCM_Rays` is a control within a path, never across paths.** The GPU and `ref` counters
  increment differently -- 834 against 2615 on `D20` -- while agreeing on deformed nodes exactly.
  Use `SCM_Nodes` to decide whether two runs did the same work.

- **GPU and CPU are not the same scenario.** The GPU path accepts only triangle-mesh collision
  shapes. On `MESH_1` its 20 falling spheres are primitives, so they deform nothing: 5422 nodes
  against the CPU loop's ~8900. Even on `MESH_0` the node counts differ (5420 vs 5474). Report the
  columns; do not divide them.

- **`ref` is a validation aid, not a fast CPU implementation.** It reproduces the GPU path's physics
  on the CPU -- which is what makes the node-count agreement above meaningful -- but at 3.5x the
  cost of the production CPU loop. It carries no speedup claim.

- **`btest_VEH_largeSCM` seeds its node map rather than driving.** The map only grows, so its size
  reflects ground covered; a 13.7 h two-rover run reached ~891k entries. Seeding isolates map size
  from how it got large. Its patch comes from a height map, not `Initialize(sizeX, sizeY, delta)`,
  because a flat patch allocates no base-height matrix at all and cannot measure one. Seeded ruts
  are 15 nodes wide at rows 2003 + 114k -- neither offset nor pitch is a power of two, so no storage
  scheme is flattered by the layout.

# Reproducing

    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCH_ENABLE_MODULE_VEHICLE=ON -DCH_ENABLE_MODULE_VEHICLE_MODELS=ON \
      -DBUILD_BENCHMARKING=ON -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF
    cmake --build build -j --target btest_VEH_wheelSCM btest_VEH_hmmwvSCM btest_VEH_largeSCM

- Run from `build/bin`; Chrono resolves its data by relative path.
- `SCM_BENCH_RAYCAST=gpu|ref|cpu` picks the path, `gpu` by default. Every run prints which one it
  got -- check that line, because a GPU run reports CPU if the backend or the domains did not take.
- One variant per process (`--benchmark_filter=SEED0`). Google Benchmark rebuilds the fixture between
  repetitions and glibc does not return the freed matrix to the OS, so a combined run reports
  allocator retention as live data.
- Nothing else on the machine. A concurrent build moves these numbers by more than the effects they
  are meant to catch.

# Gaps

1. One machine for the NVIDIA columns. L3 size is implicated in every ray-cast figure.
2. `D10` has no `ref` cell: it costs ~50 min for a column that carries no comparison.
3. `SEED1` (259.6k) and `SEED4` (934.6k) were dropped to keep the grid at 60 runs, so growth is
   bracketed by two points rather than curved.
4. No physics-parity test. These measure cost, not correctness.
5. `SEED16` at 3.63M nodes exceeds any run actually performed; read it as trend, not workload.
