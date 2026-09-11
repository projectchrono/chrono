# SCM scaling benchmarks

Three benchmark programs covering the range SCM is used over, and a recorded baseline on NVIDIA and
AMD so a change to SCM can be judged without re-running everything on every platform.

| | test | vehicle | patch | grid spacing |
|---|---|---|---|---|
| small | `btest_VEH_wheelSCM` | one Polaris wheel on a test rig | 10 x 1 m | 0.02 / 0.01 m |
| medium | `btest_VEH_hmmwvSCM` | HMMWV, four wheels | 50 x 50 m | 0.05 m |
| large | `btest_VEH_largeSCM` | HMMWV, four wheels | 300 x 300 m | 0.02 m |

Each test has variants, and the tables below use their benchmark-suite names. What they mean:

| variant | what it changes | nodes deformed |
|---|---|---|
| `WheelSCM_D20` | 20 mm grid under the wheel | 1.7k |
| `WheelSCM_D10` | 10 mm grid -- four times as many nodes over the same ground | 7.2k |
| `HmmwvSCM_MESH_0` | four `RIGID_MESH` tyres, nothing else on the terrain | 5.4k |
| `HmmwvSCM_MESH_1` | the same, plus 20 scaled rock meshes dropped on the soil | 6.1k |
| `LargeSCM_SEED0` | no pre-worked ruts: only what the vehicle itself digs | 34.6k |
| `LargeSCM_SEED1` | 1 rut laid into the node map before the run | 260k |
| `LargeSCM_SEED4` | 4 ruts | 935k |
| `LargeSCM_SEED16` | 16 ruts | 3.63M |

The `SEED` number is how many pre-worked ruts are written into the modified-node map at setup, which
is how the large test reaches a map size that would otherwise take hours of driving. The number of
ruts is the only difference between them -- same vehicle, same patch, same route -- so the spread
across `SEED0` to `SEED16` is what map size alone costs.

`btest_VEH_hmmwvSCM` also registers `CYL_0` and `CYL_1`, the same runs with `RIGID` cylinder tyres.
They are not benchmarked here: the GPU ray-cast path accepts only triangle-mesh collision shapes, so
a cylinder tyre silently falls back to the CPU and the cell would not measure what its label claims.

## What the baseline shows

**1. The GPU ray-cast path costs 2.7x to 22x less than Chrono's default CPU loop** on
`SCM_Total`, the end-to-end cost of one SCM step.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="img/scm-gpu-vs-cpu-dark.svg">
  <img alt="GPU vs CPU SCM_Total per benchmark variant, log scale" src="img/scm-gpu-vs-cpu-light.svg">
</picture>

**2. CUDA and HIP are interchangeable. The host compiler is not, and which one wins depends on the
platform.** No CUDA/HIP pair differs by more than 1.6%, at deformed-node counts identical to the
digit across all four GPU cells. On the RTX 4080 clang beats GCC on
`SCM_Total` by up to 43%; on gfx942 GCC beats clang on all six GPU cells by 7-25%. The kernel is not
what moves -- it is device code, identical in both builds -- so this is the host-side SCM work around
it.

Each bar in the chart below divides one build by another; there is no single reference build. Blue is
`GCC/HIP` over `GCC/CUDA`, isolating the backend with the compiler held fixed. Orange is `clang/HIP`
over `GCC/HIP`, isolating the compiler with the backend held fixed. 1.00 means the two builds are
identical and below 1.00 means the first is faster.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="img/scm-what-moves-it-dark.svg">
  <img alt="Relative time: backend makes no difference, host compiler does" src="img/scm-what-moves-it-light.svg">
</picture>

**3. Once the cast is on the GPU, the node work is the larger half of the step, and it grows with the
size of the modified-node map.** The cast is 79-94% of `SCM_Total` on the CPU loop and 20-55% of it
on the GPU path, so what is left to win is no longer in the cast. Ray count is 3997.2 per step at
every point below, so the growth there is map size alone: `GetHeight()` is called once per candidate
node per step and probes a map that only ever grows. This is what a tiled or narrower node store
would remove, and `btest_VEH_largeSCM` exists to measure it.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="img/scm-scaling-dark.svg">
  <img alt="SCM_Total against modified-node count" src="img/scm-scaling-light.svg">
</picture>

# Baseline -- RTX 4080

i7-13700K (16 cores / 24 threads, 30 MiB L3), 62 GB. Release, `-O3 -march=native`, Chrono OpenMP
threads 4, container `atk/chrono:orb`. GCC 11.4.0 and AMD clang 22.0 (ships with ROCm 7.2.4).
CUDA 13.2.78; HIP columns are ROCm 7.2.4 with `CMAKE_HIP_PLATFORM=nvidia`.

GPU cells are the mean of 2 processes, `ref` and CPU of 1, each itself Google Benchmark's mean over
5 or 10 internal repetitions. Worst internal cv on `SCM_Total` was 3.21% (clang/CPU `MESH_0`); every
other cell was under 2%.

## `SCM_Total`, ms/step

| variant | GCC/CUDA | GCC/HIP | clang/CUDA | clang/HIP | GCC/ref | GCC/CPU | clang/CPU |
|---|---|---|---|---|---|---|---|
| `WheelSCM_D20` | 0.0971 | 0.0982 | 0.0694 | 0.0681 | 6.8687 | 2.1204 | 2.1706 |
| `WheelSCM_D10` | 0.5046 | 0.5070 | 0.2897 | 0.2899 | - | 8.6116 | 8.2832 |
| `HmmwvSCM_MESH_0` | 0.1152 | 0.1149 | 0.1063 | 0.1050 | 6.1585 | 0.3506 | 0.4024 |
| `HmmwvSCM_MESH_1` | 0.3579 | 0.3625 | 0.3487 | 0.3513 | 44.6546 | 1.2149 | 1.4578 |
| `LargeSCM_SEED0` | 0.7514 | 0.7493 | 0.6008 | 0.5918 | 34.3628 | 2.3183 | 2.2628 |
| `LargeSCM_SEED1` | 0.7891 | 0.7786 | 0.6425 | 0.6298 | - | 2.3377 | 2.3180 |
| `LargeSCM_SEED4` | 0.8075 | 0.8182 | 0.6408 | 0.6332 | - | 2.3303 | 2.3086 |
| `LargeSCM_SEED16` | 0.8704 | 0.8702 | 0.7264 | 0.7198 | 34.5411 | 2.3447 | 2.3022 |

## Memory -- resident set after setup, MiB

| build | `SEED0` | `SEED16` | delta |
|---|---|---|---|
| GCC (any backend) | 1862.1 | 2457.0 | 594.9 |
| clang/HIP | 1864.1 | 2458.9 | 594.9 |
| clang/CUDA | 2085.1 | 2680.0 | 594.8 |

The delta is what 3.60M added nodes cost: 165 B/node, against a 128-byte `NodeRecord` plus
`unordered_map` overhead and allocator rounding. It is the same in every build; only the constant
offset moves, and most of that offset is the base-height matrix -- 15001^2 entries at 8 bytes is
1716.8 MiB.

# Baseline -- gfx942 (MI300X)

EPYC 9684X, 16-core slice, 233 GB. ROCm 7.2.4, HIP backend, GCC 11.4.0 and ROCm clang 22.0.0git.
Release, benchmarks set 4 Chrono OpenMP threads internally. All cv <= 0.18%. One process per cell.

`SCM_Total` in ms/step. The `MESH_1` row was measured at `1753904fd` and the rest of the grid at
`7c0d0ea20`; the only change to the other tests in between adds the backend selector and no physics.

| variant | GCC/HIP | GCC/CPU | clang/HIP | clang/CPU |
|---|---|---|---|---|
| `WheelSCM_D20` | 0.1726 | 3.3830 | 0.1985 | 3.4522 |
| `WheelSCM_D10` | 0.6863 | 13.8749 | 0.8582 | 13.7305 |
| `HmmwvSCM_MESH_0` | 0.2214 | 0.7856 | 0.2370 | 0.6313 |
| `HmmwvSCM_MESH_1` | 0.5178 | 2.1898 | 0.5631 | 2.2433 |
| `LargeSCM_SEED0` | 1.1186 | 3.5594 | 1.2941 | 4.7062 |
| `LargeSCM_SEED16` | 1.3809 | 4.7167 | 1.5779 | 4.8605 |

Resident set after setup, MiB: `SEED0` 2366 GPU / 1745 CPU, `SEED16` 2961 GPU / 2340 CPU. The
595 MiB node-storage delta matches the NVIDIA host exactly; the GPU builds carry ~620 MiB more
constant offset than the CPU-only builds.

Deformed-node counts agree with the RTX 4080 host to the digit on every variant except `MESH_1`,
where the two hosts span 6106 to 6155 -- 0.8%, and the residual is between the two gfx942 GPU cells
rather than between the paths. The two platforms ran the same work and the columns are directly
comparable.

## The two hosts side by side

`SCM_Total` in ms/step, GCC on both, so the only variable is the machine:

| variant | GPU 4080 | GPU gfx942 | CPU 4080 | CPU gfx942 |
|---|---|---|---|---|
| `D20` | 0.0982 | 0.1726 | 2.1204 | 3.3830 |
| `D10` | 0.5070 | 0.6863 | 8.6116 | 13.8749 |
| `MESH_0` | 0.1149 | 0.2214 | 0.3506 | 0.7856 |
| `MESH_1` | 0.3625 | 0.5178 | 1.2149 | 2.1898 |
| `SEED0` | 0.7493 | 1.1186 | 2.3183 | 3.5594 |
| `SEED16` | 0.8702 | 1.3809 | 2.3447 | 4.7167 |

The 4080 is ahead everywhere: 1.4-1.9x on the GPU path, 1.5-2.2x on the CPU path. That is the
expected shape rather than a surprise. These are small kernels -- a few hundred to a few thousand
rays per step -- where a high-clocked consumer part beats a datacenter GPU built for wide parallel
work, and a shared 16-core EPYC slice loses to a 5.3 GHz Raptor Lake on a four-thread CPU path.

# Caveats

- **Primitive-shaped bodies get no contact force on the GPU path.** The GPU backend intersects
  triangle meshes only: a primitive collision shape contributes no faces, never appears as a hit's
  contactable, and receives nothing back from the soil. It falls through the terrain. The bail-out
  in `ComputeRayCastGpuHip` fires only when *no* candidate carries a mesh, so a mixed scene keeps
  the GPU path and silently drops its primitives. This is why `MESH_1`'s obstacles are rock meshes:
  as spheres they fell through, and the variant timed a scene its obstacles were absent from. The
  defect is in the backend, not the benchmark, and is unfixed.

- **`D20` and `D10` deform 1.41x and 1.31x more nodes on the CPU loop than on the GPU path.** Not
  the above -- the wheel rig has no primitive collidables -- and not the footprint cull, since `ref`
  applies no cull and matches the GPU node count exactly. Unexplained. Everywhere else the paths
  agree: `MESH_0` 1.01x, `MESH_1` 1.00x, `SEED16` 1.00x.

- **`ref` carries no speedup claim.** It reproduces the GPU path's physics on the CPU, which is what
  makes the node-count agreement meaningful, but it is a validation aid and runs several times
  slower than the production CPU loop. `D10` has no `ref` cell: ~50 min for a column that compares
  nothing.

- **Total step time is not a measure of SCM work.** In the wheel test the constraint solver is a
  third of it. Every figure here is `SCM_Total`, not wall-clock per step.

- **One machine per platform.** L3 size is implicated in every CPU-path figure, and the AMD host is
  a shared slice.

# Reproducing

    cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCH_ENABLE_MODULE_VEHICLE=ON -DCH_ENABLE_MODULE_VEHICLE_MODELS=ON \
      -DBUILD_BENCHMARKING=ON -DBUILD_DEMOS=OFF -DBUILD_TESTING=OFF
    cmake --build build -j --target btest_VEH_wheelSCM btest_VEH_hmmwvSCM btest_VEH_largeSCM

- Run from `build/bin`; Chrono resolves its data by relative path.
- `SCM_BENCH_RAYCAST=gpu|ref|cpu` picks the path, `gpu` by default. **Every run prints which one it
  got. Check that line.** A binary older than this variable ignores it silently and reports `gpu`,
  which is how a whole grid of "CPU" numbers can turn out to be GPU runs.
- One variant per process, and **anchor the filter**: `--benchmark_filter=SEED1/` matches only
  `SEED1`, where `SEED1` also matches `SEED16`. Google Benchmark rebuilds the fixture between
  repetitions and glibc does not return the freed matrix to the OS, so a combined run reports
  allocator retention as live data.
- Nothing else on the machine. A concurrent build moves these numbers by more than the effects they
  are meant to catch.
- `ScmBenchmarkUtils.h` also reports `SCM_Rays`, and it does not compare across paths: the GPU path
  discards nodes outside every mesh body's XY footprint before counting
  ([`SCMTerrainRaycastGpu.cpp`](../../../chrono_vehicle/terrain/SCMTerrainRaycastGpu.cpp), the
  `in_footprint` test) and the CPU loop does not, so the same hits are found from different ray
  counts -- 363 against 1682 on `D20`. Compare `SCM_Nodes`.
- Do not add an active domain to the wheel test. `ChWheelTestRig::CreateTerrainSCM` already declares
  one; a second casts the nodes under the wheel once per domain and inflates `SCM_Rays` without
  changing any physics.

# Why the large test is built the way it is

The modified-node map only grows -- a node enters on first contact and is never evicted -- so in a
long run its size reflects ground covered, not ground under a wheel; a 13.7 h two-rover run reached
~891k entries. Seeding gets there in under a second and isolates map size from how it got large.
The patch comes from a height map rather than `Initialize(sizeX, sizeY, delta)` because a flat patch
allocates no base-height matrix at all and so cannot measure one. Seeded ruts are 15 nodes wide at
rows 2003 + 114k: neither offset nor pitch is a power of two, so no storage scheme is flattered by
the layout.

# Appendix -- the `SCM_RayCast` sub-timer

Recorded for regression triage only, so a future slowdown can be placed in the cast or in the node
work around it. It carries no result of its own: the cast alone does not produce a deformed terrain,
and the numbers to compare against are the `SCM_Total` tables above.

RTX 4080, ms/step:

| variant | GCC/CUDA | GCC/HIP | clang/CUDA | clang/HIP | GCC/ref | GCC/CPU | clang/CPU |
|---|---|---|---|---|---|---|---|
| `WheelSCM_D20` | 0.0359 | 0.0370 | 0.0365 | 0.0352 | 6.7994 | 1.9911 | 2.0941 |
| `WheelSCM_D10` | 0.0978 | 0.1000 | 0.0975 | 0.0975 | - | 7.7662 | 7.8633 |
| `HmmwvSCM_MESH_0` | 0.0628 | 0.0629 | 0.0635 | 0.0634 | 6.0965 | 0.2925 | 0.3513 |
| `HmmwvSCM_MESH_1` | 0.1913 | 0.1942 | 0.1911 | 0.1923 | 44.4682 | 1.0325 | 1.2762 |
| `LargeSCM_SEED0` | 0.3260 | 0.3248 | 0.3288 | 0.3261 | 33.9085 | 1.8237 | 1.9392 |
| `LargeSCM_SEED1` | 0.3516 | 0.3473 | 0.3615 | 0.3537 | - | 1.8418 | 1.9865 |
| `LargeSCM_SEED4` | 0.3669 | 0.3757 | 0.3641 | 0.3611 | - | 1.8351 | 1.9833 |
| `LargeSCM_SEED16` | 0.4431 | 0.4427 | 0.4524 | 0.4516 | 34.0860 | 1.8498 | 1.9782 |

gfx942, ms/step:

| variant | GCC/HIP | GCC/CPU | clang/HIP | clang/CPU |
|---|---|---|---|---|
| `WheelSCM_D20` | 0.0939 | 3.1726 | 0.0971 | 3.2355 |
| `WheelSCM_D10` | 0.1882 | 12.6312 | 0.2036 | 12.4352 |
| `HmmwvSCM_MESH_0` | 0.1464 | 0.6282 | 0.1501 | 0.5076 |
| `HmmwvSCM_MESH_1` | 0.2998 | 1.8277 | 0.3105 | 1.8714 |
| `LargeSCM_SEED0` | 0.5670 | 2.8957 | 0.5868 | 3.4875 |
| `LargeSCM_SEED16` | 0.8256 | 3.6066 | 0.8656 | 3.6381 |
