# SCM scaling benchmarks

Three tests bracket the range SCM is used over. They share a vehicle, soil model, driver, active
domain sizes and step size wherever they can, so that what differs between them is scale:

| | test | patch | spacing | modified nodes |
|---|---|---|---|---|
| small | `btest_VEH_wheelSCM` | 10 x 1 m | 0.02 / 0.01 m | 1.7k / 7.2k |
| medium | `btest_VEH_hmmwvSCM` | 50 x 50 m | 0.05 m | 5.4k |
| large | `btest_VEH_largeSCM` | 300 x 300 m | 0.02 m | 34.6k - 3.63M |

All three run with the SCM visualization mesh disabled. It is not free when it is never drawn:
`m_trimesh_shape` gates a per-node vertex update inside the modified-node loop of
`ComputeInternalForces`, and that cost lands in the node loop rather than in the visualization
timer, so a headless run charges it to soil physics.

## Reading the numbers

Total step time is not, on its own, a usable measure of SCM work. In the single-wheel test the
constraint solver is a third of it and its run-to-run spread would hide a large change in SCM cost.
Each test therefore accumulates SCM's own per-step timers over the timed window and reports them as
counters in **milliseconds per step** (`ScmBenchmarkUtils.h`): `SCM_Total` and its breakdown
`SCM_Domains` / `SCM_RayCast` / `SCM_Patches` / `SCM_Forces` / `SCM_Bulldoze`, plus `SCM_Rays` and
`SCM_Nodes`.

`SCM_Rays` is the control. Two builds that cast the same number of rays per step are doing the same
physical work, and any difference in `SCM_RayCast` is then about how that work is done.

## Why the large test looks the way it does

**The patch comes from a height map, not `Initialize(sizeX, sizeY, delta)`.** A flat patch allocates
no base-height matrix at all -- `SCMLoader::GetInitHeight` returns 0 for `PatchType::FLAT` -- so a
flat patch cannot measure what that matrix costs at any size. Here it is 15001^2 entries and is the
dominant allocation.

**The modified-node map is pre-seeded.** That map only grows: a node enters on first contact and is
never evicted, so in a long run its size reflects ground covered, not ground currently under a
wheel. A 13.7 h two-rover run at 0.02 m spacing reached ~891k entries. Reaching that by driving
takes hours; seeding puts the test there in under a second, and it is also the cleaner experiment,
because the quantity under test is the size of the map and not how it got large. `SEED0` through
`SEED16` differ in nothing else, so the spread across them is what map size costs.

Seeded ruts are 15 nodes (0.30 m) wide, span the patch, and start at rows 2003 + 114k. Neither the
first row nor the pitch is a multiple of any power of two: a rut falls where a vehicle drove, and a
layout that happened to line up with some internal block size would flatter one storage scheme over
another.

**Measure memory one variant per process** (`--benchmark_filter=SEEDn`). Google Benchmark rebuilds
the fixture between repetitions and glibc does not return a freed multi-hundred-MB matrix to the OS,
so in a full sweep every variant after the first reports allocator retention as much as live data.

---

# Baseline

RTX 4080 host, 24 x 5300 MHz, 30 MiB L3, 62 GB RAM. GCC 11.4.0, Release, `-march=native`.
Chrono OpenMP threads 4. Runs are inside the `atk/chrono:orb` container.

The GPU ray-cast backend is **not** enabled in any of these runs -- this is the CPU path, which is
what a default Chrono build uses and what the node-storage code sits in.

Two builds:

| label | branch | commit |
|---|---|---|
| `main` | `feature/scm-benchmarks` (= `main` + these benchmarks) | `824ecd822` |
| `fp32` | `bench/scm-fp32` (= the above + `scm-fp32-and-index-overflow`) | `851444eae` |

The only difference between them inside SCM is `SCMTerrain.{h,cpp}`: single-precision node storage
and the 16x16 tiled node map.

Each figure is the mean of independent processes (3 for small and medium, 2 for large), each of
which is itself Google Benchmark's mean over 5 or 10 internal repetitions. Within-process cv was
0.1-0.5% except where noted.

## Time

### Large -- `SCM_Total`, ms/step

| variant | nodes | `main` | `fp32` | change |
|---|---|---|---|---|
| `SEED0` | 34.6k | 0.7539 | 0.7227 | **-4.1%** |
| `SEED1` | 259.6k | 0.7756 | 0.7105 | **-8.4%** |
| `SEED4` | 934.6k | 0.7921 | 0.7114 | **-10.2%** |
| `SEED16` | 3.63M | 0.8886 | 0.7144 | **-19.6%** |
| | growth 34.6k -> 3.63M | **+17.9%** | **-1.2%** | |

### Large -- `SCM_RayCast`, ms/step

| variant | nodes | `main` | `fp32` | change |
|---|---|---|---|---|
| `SEED0` | 34.6k | 0.3284 | 0.3003 | **-8.6%** |
| `SEED1` | 259.6k | 0.3467 | 0.2915 | **-15.9%** |
| `SEED4` | 934.6k | 0.3638 | 0.2925 | **-19.6%** |
| `SEED16` | 3.63M | 0.4573 | 0.2951 | **-35.5%** |
| | growth 34.6k -> 3.63M | **+39.3%** | **-1.7%** | |

`SCM_Rays` is 3997.2 per step in every one of these runs, `main` and `fp32` alike, so both builds
cast identical work. `SCM_Patches` is flat on `main` across the sweep (0.3661 -> 0.3695, +0.9%):
none of the growth is in contact-patch assembly. It is all in ray casting, which is where
`GetHeight()` is called once per active-domain node per step. That is the mechanism the tiling
change targets, isolated.

The result to take from this table is not the percentage -- it is that the `fp32` column is flat.
Cost on `main` grows with ground covered and keeps growing for the length of a run; on `fp32` a
105x increase in map size costs nothing measurable.

Whole-step time follows: at `SEED16`, 1.042 ms/step on `main` against 0.854 on `fp32`, -18.0%.

### Medium and small -- `SCM_Total`, ms/step

| test | nodes | `main` | `fp32` | change |
|---|---|---|---|---|
| `HmmwvSCM_MESH_1` | 5.4k | 0.1764 | 0.1712 | **-2.9%** |
| `HmmwvSCM_MESH_0` | 5.4k | 0.1157 | 0.1182 | **+2.1%** |
| `WheelSCM_D10` | 7.2k | 0.5105 | 0.5114 | **+0.2%** |
| `WheelSCM_D20` | 1.7k | 0.0983 | 0.1001 | **+1.8%** |

**There is a small-scale regression, and it is real.** On `WheelSCM_D20` the three `main` runs are
all at or below 0.0994 and the three `fp32` runs all at or above 0.0996, so the ranges do not
overlap; the same holds for `MESH_0`. It is what tiling costs where tiling does not pay: a map of
1.7k nodes allocates whole 16x16 tiles for ruts a fraction of a tile wide, so it does more
allocation and touches more memory than a flat map would to hold the same data.

`MESH_1` going the other way is consistent with this. It differs from `MESH_0` only by 20 falling
spheres, which take the active-domain count from 4 to 24 and roughly double `SCM_RayCast`
(0.1217 vs 0.0637 on `main`). More probes per step is where tiling starts paying, at the same node
count.

`main`'s third `MESH_1` process was an outlier (cv 5.43%, and a node count of 5688 against 5413 in
the other two). Dropping it puts `main` at 0.1764 and the change at -2.9%; keeping it gives -3.5%.
The table uses the conservative figure.

So the crossover is somewhere between roughly 10k and 250k nodes, and depends on probes per step as
well as node count. Below it the change costs 1-2%; above it the change saves 8-20% and, more
importantly, stops the growth.

## Memory

Resident set after setup, one variant per process, MiB:

| variant | nodes | `main` | `fp32` | saved |
|---|---|---|---|---|
| `SEED0` | 34.6k | 1863.6 | 1005.2 | 858.4 (-46.1%) |
| `SEED1` | 259.6k | 1900.6 | 1034.7 | 865.9 (-45.6%) |
| `SEED4` | 934.6k | 2011.9 | 1123.2 | 888.7 (-44.2%) |
| `SEED16` | 3.63M | 2458.5 | 1447.7 | 1010.8 (-41.1%) |

The base-height matrix is 15001^2 entries: 1716.8 MiB at 8 bytes, 858.4 MiB at 4. The measured
`SEED0` difference is 858.4 MiB, i.e. the analytic value exactly.

Node storage is the more interesting column. Taking `SEED16` against `SEED0`, `main` spends
594.9 MiB on 3.60M added nodes (165 B/node: a 128-byte `NodeRecord` plus `unordered_map` node
overhead and allocator rounding) and `fp32` spends 442.6 MiB (123 B/node).

123 B/node is well above the 64-byte record, and the reason is worth stating plainly: **tiles are
allocated whole.** A 15-node-wide rut occupies one or two rows of 16x16 tiles along its length, so
roughly half of every tile it touches is allocated and never used. At this fill density tiling gives
back about half of what single precision saves on the node records. It is still a net reduction
against `main` at every size measured, and the base-height matrix -- which dominates until several
million nodes -- halves outright.

## Physics

Not bit-identical, as expected from a change of storage precision. Deformed-node counts after the
same run: 34,587 vs 34,597 on `SEED0` (10 nodes in 34.6k), 5413 vs 5408 on `MESH_0`, identical on
both wheel variants. Trajectories diverge at the level float resolution allows and contact decisions
near a threshold can land differently; nothing here suggests more than that, but this set does not
check trajectories directly and a reviewer asking for a physics-parity test is asking for something
these tests do not provide.

## Gaps

1. **One machine, one compiler.** GCC 11.4.0 on one Raptor Lake host with a 30 MiB L3. Cache size is
   directly implicated in every result above, so a machine with a materially different L3 could move
   the crossover point in either direction.
2. **The crossover is bracketed, not located.** 7.2k nodes regresses, 259.6k improves. Nothing was
   run in between.
3. **No physics-parity test**, per the section above.
4. **CPU path only.** The GPU ray-cast backend also inserts into the node map and would show a
   different balance, since its ray casting does not go through `GetHeight()` per node the same way.
5. `SEED16` at 3.63M nodes is beyond any run actually performed; it is there to show the trend, and
   should be read as extrapolation, not as a reported workload.
