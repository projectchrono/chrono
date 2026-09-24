Chrono::preCICE Reference Manual {#manual_precice}
==========================================

The [Chrono::preCICE](group__precice__module.html) module lets a Chrono solver take part in a partitioned
multi-physics simulation driven by [preCICE](https://precice.org/), an open-source coupling library. Each
solver runs as its own process, and preCICE takes care of process discovery, data mapping between
non-matching meshes, and advancing the coupled time stepping.

The module provides adapters for three Chrono solvers:

| Adapter | Chrono solver | Typical role |
|---------|---------------|--------------|
| [ChPreciceAdapterMbs](@ref chrono::ch_precice::ChPreciceAdapterMbs) | Chrono multibody system | solid phase |
| [ChPreciceAdapterSph](@ref chrono::ch_precice::ChPreciceAdapterSph) | [Chrono::FSI-SPH](@ref manual_fsi) fluid solver | fluid phase |
| [ChPreciceAdapterTdpf](@ref chrono::ch_precice::ChPreciceAdapterTdpf) | [Chrono::FSI-TDPF](@ref manual_fsi) fluid solver | fluid phase |

The partner participant need not be a Chrono solver: a Chrono multibody system can equally be coupled to an
external CFD code, which is the point of using preCICE rather than the monolithic
[Chrono::FSI](@ref manual_fsi) framework.

* [Install and build Chrono::preCICE](@ref module_precice_installation)
* [Chrono::preCICE overview](@ref module_precice_overview)
* [The Chrono preCICE adapters](@ref precice_adapters)
* [Coupling schemes and time stepping](@ref precice_coupling)

## Relation to Chrono::FSI

Chrono::FSI and Chrono::preCICE solve overlapping problems by different means, and the choice between them
is usually straightforward:

- [Chrono::FSI](@ref manual_fsi) couples a Chrono multibody system to a Chrono fluid solver inside a single
  process. The two phases share one executable, exchange data through in-memory structures, and are advanced
  by a single `ChFsiSystem`. This is the simpler and faster option whenever both phases are Chrono solvers.
- Chrono::preCICE couples a Chrono solver to any other preCICE-enabled solver, each in its own process. Use
  it when the other phase is not a Chrono solver, when the two phases must run on different machines or with
  different parallel decompositions, or when you want to swap fluid solvers without rebuilding the solid
  participant.

The `sphere_drop` example distributed with the module illustrates the second point: the same Chrono
multibody participant is coupled, unchanged, to a Chrono::FSI-SPH solver, to a Chrono::FSI-TDPF solver, or
to a small mock-up fluid solver, by changing only the preCICE configuration and launching a different fluid
participant.
