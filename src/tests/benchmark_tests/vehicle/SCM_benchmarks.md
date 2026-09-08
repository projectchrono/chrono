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
what a default Chrono build uses.

Each figure is the mean of independent processes (3 for small and medium, 2 for large), each of
which is itself Google Benchmark's mean over 5 or 10 internal repetitions. Within-process cv was
0.1-0.5%, with one exception: a third `MESH_1` process ran at cv 5.43% and reported 5688 nodes
against 5413 in the other two. It is excluded from the figure below.

## Time

`SCM_Total`, ms/step:

| test | nodes | ms/step |
|---|---|---|
| `WheelSCM_D20` | 1.7k | 0.0983 |
| `HmmwvSCM_MESH_0` | 5.4k | 0.1157 |
| `HmmwvSCM_MESH_1` | 5.4k | 0.1764 |
| `WheelSCM_D10` | 7.2k | 0.5105 |
| `LargeSCM_SEED0` | 34.6k | 0.7539 |
| `LargeSCM_SEED1` | 259.6k | 0.7756 |
| `LargeSCM_SEED4` | 934.6k | 0.7921 |
| `LargeSCM_SEED16` | 3.63M | 0.8886 |

`MESH_1` differs from `MESH_0` only by 20 falling spheres, which take the active-domain count from
4 to 24 and roughly double the ray casting (`SCM_RayCast` 0.1217 against 0.0637) at an unchanged
node count. Probes per step, not node count alone, is what sets ray-cast cost.

Over the large sweep the cost is all in ray casting:

| variant | nodes | `SCM_RayCast` | `SCM_Patches` |
|---|---|---|---|
| `SEED0` | 34.6k | 0.3284 | 0.3661 |
| `SEED1` | 259.6k | 0.3467 | |
| `SEED4` | 934.6k | 0.3638 | |
| `SEED16` | 3.63M | 0.4573 | 0.3695 |

`SCM_Rays` is 3997.2 per step in every one of these runs, so every variant casts identical work,
and contact-patch assembly is flat (+0.9% end to end). The 39.3% growth in `SCM_RayCast`, and with
it the 17.9% growth in `SCM_Total`, comes from map size alone: `GetHeight()` is called once per
active-domain node per step and probes a map that has grown 105x. Whole-step time at `SEED16` is
1.042 ms/step.

That growth is why the large test seeds the map rather than driving. Cost here rises with ground
covered and keeps rising for the length of a run, which a short flat-patch benchmark cannot show.

## Memory

Resident set after setup, one variant per process, MiB:

| variant | nodes | RSS |
|---|---|---|
| `SEED0` | 34.6k | 1863.6 |
| `SEED1` | 259.6k | 1900.6 |
| `SEED4` | 934.6k | 2011.9 |
| `SEED16` | 3.63M | 2458.5 |

The base-height matrix accounts for most of it: 15001^2 entries at 8 bytes is 1716.8 MiB. Node
storage is the rest -- `SEED16` against `SEED0` is 594.9 MiB for 3.60M added nodes, or 165 B/node,
against a 128-byte `NodeRecord` plus `unordered_map` node overhead and allocator rounding.

Deformed-node counts, for anything comparing against these figures: 34,587 on `SEED0`, 5413 on
`MESH_0`.

## Gaps

1. **One machine, one compiler.** GCC 11.4.0 on one Raptor Lake host with a 30 MiB L3. Cache size
   is directly implicated in the ray-cast figures, so a machine with a materially different L3
   will not reproduce them.
2. **CPU path only.** The GPU ray-cast backend does not call `GetHeight()` per node the way the CPU
   path does and would show a different balance.
3. **No physics-parity check.** These tests measure cost, not correctness; a change that alters
   trajectories will not be caught here.
4. `SEED16` at 3.63M nodes is beyond any run actually performed. It is there to show the trend and
   should be read as extrapolation, not as a reported workload.
