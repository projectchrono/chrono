CRM Active Domains and Neighbor-Search Frequency {#vehicle_terrain_crm_performance}
===============================================================================

\tableofcontents

Scope
-----

This page explains two Chrono::CRM performance features for vehicle-terrain simulation:

- active domains
- reduced neighbor-search frequency (`num_proximity_search_steps`)

Both are available in the Chrono::FSI-SPH backend used by [CRMTerrain](@ref chrono::vehicle::CRMTerrain).

What Active Domains Do
----------------------

Active domains restrict per-step SPH work to particles near interacting bodies.

- **Active** particles are inside the active box.
- **Extended-active** particles are outside the active box, but within an additional solver-defined margin around it.
- **Inactive** particles are not processed in the current step.

In practice, only active and extended-active particles are sorted, included in neighbor search, and integrated.

<img src="http://www.projectchrono.org/assets/manual/vehicle/terrain/FSI-active_domain.png" width="700" />

What The Active Box Means
-------------------------

In `CRMTerrain`, `SetActiveDomain(ChVector3d(Lx, Ly, Lz))` defines a **fixed box size** used by the solver.

- The same box dimensions are applied to FSI objects registered for CRM interaction.
- With the high-level `CRMTerrain` API, this is typically done through `AddRigidBody(...)` and `AddFeaMesh(...)`.
- At the lower level, this corresponds to `AddFsiBody(...)` / `AddFsiMesh...(...)` calls.
- The active region is the **union** of these fixed-size boxes around all registered interacting entities.

So from a user perspective: it is one fixed box definition that is applied around all bodies/meshes added to the FSI
side of the CRM terrain problem.

Code-Level Implementation
-------------------------

Vehicle-side API:

- `CRMTerrain::SetActiveDomain(...)` and `CRMTerrain::SetActiveDomainDelay(...)`
- implemented as wrappers in:
  - `src/chrono_vehicle/terrain/CRMTerrain.h`
  - `src/chrono_vehicle/terrain/CRMTerrain.cpp`

SPH-side implementation:

- `ChFsiFluidSystemSPH::OnDoStepDynamics(...)` updates particle activity, optionally rebuilds neighbor lists, and then
  advances dynamics (`src/chrono_fsi/sph/ChFsiFluidSystemSPH.cpp`).
- Activity flags are computed in `UpdateActivityD(...)`
  (`src/chrono_fsi/sph/physics/SphFluidDynamics.cu`).
- The activity test uses positions of registered FSI bodies and FEA nodes (from FSI state arrays on device).
- In `UpdateActivityD(...)`, the extended region is computed as:
  `ExAcdomain = bodyActiveDomain + 2 * h_multiplier * h`.
- Neighbor-list rebuild cadence is controlled by:
  `m_frame % num_proximity_search_steps == 0`
  in `OnDoStepDynamics(...)`.
- Neighbor lists are built through cell hashing/sorting in
  `src/chrono_fsi/sph/physics/SphCollisionSystem.cu`.

How To Enable
-------------

Set active domains in `CRMTerrain`:

~~~{.cpp}
CRMTerrain terrain(*sysMBS, spacing);

// ... register interacting FSI bodies first (wheels, shoes, tools, etc.)
terrain.SetActiveDomain(ChVector3d(0.6, 0.6, 0.8));  // full box size
terrain.SetActiveDomainDelay(1.0);                   // optional settling period
~~~

Set neighbor-search frequency through SPH parameters:

~~~{.cpp}
ChFsiFluidSystemSPH::SPHParameters sph_params;
// ... set other SPH fields as needed
sph_params.num_proximity_search_steps = 10;  // 1 = rebuild every step
terrain.SetSPHParameters(sph_params);
~~~

Equivalent low-level call on the SPH system:

~~~{.cpp}
sysSPH.SetNumProximitySearchSteps(10);
~~~

Practical Tuning Guidance
-------------------------

1. Start with `num_proximity_search_steps = 1`.
2. Increase to `4`, then `10`, and compare force/sinkage/traction metrics for your case.
3. Use active domains whenever soil disturbance is localized around moving bodies.
4. Use `SetActiveDomainDelay(...)` if you run an initial settling phase.
5. Keep active boxes large enough to contain the full interaction region.

For broader SPH parameter guidance, see
[Chrono::FSI-SPH Parameter Selection Guide](@ref manual_fsi_sph_parameter_selection).

Important note:
- Active domains are intended for CRM terramechanics simulations (not general CFD usage).

Performance Reported in the Paper
---------------------------------

Reference:  
[A Physics-Based Continuum Model for Versatile, Scalable, and Fast Terramechanics Simulation](https://arxiv.org/pdf/2507.05643)  

Key reported results:

- Persistent neighbor lists (`ps_freq = 10`) provided about `1.28x` to `1.36x` speedup
  (23-27% wall-clock reduction), with no significant loss in accuracy in the validation benchmarks.
- Active domains alone (`ps_freq = 1`) provided:
  - `2.93x` speedup (MGRU3 wheel)
  - `2.11x` speedup (RASSOR drum)
  with no significant loss in accuracy.
- Combined (`ps_freq = 10` + active domains), speedups above `7x` were reported in multiple benchmarks, with up to
  `15.9x` in the tracked-vehicle benchmark (relative to the baseline implementation in the paper).

Related demos:

- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_WheeledVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_TrackedVehicle.cpp`
- `src/demos/fsi/sph/demo_FSI-SPH_RassorDrum.cpp`
