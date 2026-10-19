The Chrono preCICE Adapters {#precice_adapters}
==========================================

\tableofcontents

## Common structure

All three adapters derive from [ChPreciceAdapter](@ref chrono::ch_precice::ChPreciceAdapter) and are
constructed in one of two ways:

- from an existing Chrono solver object, with the coupling interface then declared through `AddCoupling...`
  calls in application code;
- from a Chrono YAML specification file, in which case the adapter creates the solver, populates the model,
  and makes those same calls itself from the `precice_adapter_config` object of that file. This requires
  Chrono::Parsers with YAML support.

Both constructors also take the path to the shared preCICE configuration. The YAML route is what the shipped
demos use, and it is the reason those demos contain almost no model-building code.

What a concrete adapter supplies is the translation between preCICE data and Chrono state: building the
coupling mesh vertices from Chrono modeling objects, turning received data into loads or prescribed motion,
extracting the data to send, advancing its solver, and — where supported — saving and restoring state for
implicit coupling.

The table below summarizes where the three differ. The rest of this page takes them in turn.

| | MBS / Solid | Fluid_SPH | Fluid_TDPF |
|---|---|---|---|
| Chrono solver | `ChSystem` | `ChFsiFluidSystemSPH` | `ChFsiFluidSystemTDPF` |
| Reads | forces, torques | body poses, velocities, accelerations | body poses, velocities |
| Writes | poses, displacements, velocities, accelerations | forces, torques | forces, torques |
| `RIGID_BODY_REFS` | yes | yes | yes |
| `RIGID_BODY_POINTS` | yes | not implemented | not implemented |
| `FEA_MESH_NODES` | yes | not implemented | — |
| `FEA_MESH_POINTS` | not implemented | not implemented | — |
| Checkpointing | yes | no | no |
| Solver step | `min(step, window)` | `min(step, window)` | the whole window |
| Added mass | consumes | — | produces |
| Soft real time | yes | no | no |
| Step callbacks | yes | no | no |

"Not implemented" above means the code path exists and throws a clear runtime error, not that it silently
does nothing.

## The MBS (solid) adapter

[ChPreciceAdapterMbs](@ref chrono::ch_precice::ChPreciceAdapterMbs) exposes a Chrono multibody system as a
participant. It is the adapter used by every shipped example, and the one most likely to be paired with a
non-Chrono solver.

**Coupling objects.** Bodies are added with `AddCouplingBody`, each with an optional set of points expressed
in the body frame. With no points the body is exchanged through its reference frame on a `RIGID_BODY_REFS`
mesh; with points it contributes them to a `RIGID_BODY_POINTS` mesh. In a YAML specification the points may
be given as a plain list of coordinates, or as a Wavefront `.obj` or `.stl` file whose vertices are used, so
a surface mesh can drive the exchange directly. FEA meshes are added with `AddCouplingFEAMesh` (or listed by
name under `meshes` in a YAML specification) and are exchanged through a `FEA_MESH_NODES` mesh whose vertices
are the mesh nodes with position degrees of freedom (`ChNodeFEAxyz` and derived types, and
`ChNodeFEAxyzrot`), in the order in which the meshes were added and, within each mesh, in node order. FEA
nodes can receive forces (but not torques) and send positions, displacements (relative to the initial node
position), and velocities.

**Applying received loads.** Forces and torques arriving from the fluid participant are applied through a
Chrono force accumulator held per coupling body, and forces on FEA nodes through a force accumulator held per
coupling node. The accumulators are emptied at the start of each read, so the loads replace rather than
accumulate across windows, and they remain applied while the multibody system takes its step. Since they are
dedicated accumulators, they add to (and do not overwrite) any other loads applied on the same bodies and nodes.

**Data it can send.** `POSITIONS`, `ROTATIONS` (as rotation vectors), `DISPLACEMENTS` (relative to the
initial pose), `LINEAR_VELOCITIES`, `ANGULAR_VELOCITIES`, `LINEAR_ACCELERATIONS`, and `ANGULAR_ACCELERATIONS`.
Rotations and angular quantities are only available on a `RIGID_BODY_REFS` mesh; the linear quantities on a
`RIGID_BODY_POINTS` mesh are those of the individual body points. Which of these are actually exchanged is
decided by the preCICE XML; a CFD partner driving a moving wall typically wants displacements, while a
potential-flow partner wants absolute positions and velocities, and an SPH partner also uses accelerations.

**Added mass.** This is the one adapter that consumes added mass. When the preCICE configuration declares
the added mass meshes, the adapter builds a [ChLoadHydrodynamics](@ref chrono::ChLoadHydrodynamics) so that
the coefficients enter the system mass matrix rather than being applied as a force. The blocks come either
from an HDF5 hydrodynamic file or from values given inline in the YAML, and may additionally be updated over
the coupling interface every window. Those blocks are then the baseline to which the increments received from
fluid participants are added (see [Added mass over the interface](@ref precice_coupling)). The ordering
requirement added mass imposes on the coupling bodies, and why it cannot be treated as a force, are covered in
[Added mass in Chrono::FSI-TDPF](@ref manual_fsi_tdpf_added_mass).

**Checkpointing.** The adapter implements it, gathering and scattering the full system state. It is the only
one of the three that does, which is what makes implicit coupling schemes conceivable for this participant.

**Control hooks.** `EnforceRealtime` paces the run against the wall clock, useful when a human is watching
the visualization. `RegisterBeforeStepDynamicsCallback` and `RegisterAfterStepDynamicsCallback` let
application code run just before or after each `DoStepDynamics`; the `flap_openfoam` demo uses the former to
vary a spring coefficient during the run.

## The Fluid_SPH adapter

[ChPreciceAdapterSph](@ref chrono::ch_precice::ChPreciceAdapterSph) exposes a Chrono::FSI-SPH fluid solver
as a participant. It reads the poses, velocities, and (optionally) accelerations of the solid bodies,
prescribes them on its own representation of those bodies, advances the SPH solution, and writes back the
resulting forces and torques. Solid accelerations enter the boundary treatment of the SPH solver (the pressure
extrapolation of the Adami boundary method); if they are not exchanged, they are taken as zero.

**Coupling objects.** Bodies are added by name, initial pose, and geometry. The adapter creates the BCE
markers that represent the body to the SPH solver, either from a `ChBodyGeometry` description — the usual
case, and what a YAML `shapes` entry produces — or from an explicit list of marker positions. Note that the
SPH participant builds its *own* proxies for the solid bodies; it never sees the partner's multibody model,
only the states arriving over the interface.

**Exchange granularity.** Only `RIGID_BODY_REFS` is implemented: the solid states arrive as one pose and
velocity per body, and the forces are returned as one resultant per body. The BCE-level exchange that would
correspond to a `RIGID_BODY_POINTS` mesh is not implemented.

**Time stepping.** The adapter advances by `min(its own step, the coupling window)`. Because the SPH step is
usually far smaller than a sensible coupling window, the normal situation is that preCICE sub-cycles: the
loop runs several iterations inside one window, exchanging data at each. See
[time stepping](@ref precice_coupling) for what that implies.

**Checkpointing.** Not available; the SPH state is not captured and restored, so implicit coupling schemes
cannot be used with this participant.

## The Fluid_TDPF adapter

[ChPreciceAdapterTdpf](@ref chrono::ch_precice::ChPreciceAdapterTdpf) exposes a Chrono::FSI-TDPF
time-domain potential flow solver as a participant. Its external behavior matches the SPH adapter — read
solid states, write forces and torques — but the solver underneath is different in ways that show through.

**Coupling objects.** Bodies are added by name and initial pose only; no geometry is needed, because the
hydrodynamic behavior comes from the coefficients in the HDF5 hydrodynamic file rather than from a
discretization of the body. The bodies created internally carry no visual or collision shapes, which is why
the participant's run-time visualization shows the water surface and the body reference frames but no hull
geometry.

**Body order is significant.** The association between a coupling body and its hydrodynamic coefficients is
by index: the i-th body listed corresponds to `/body{i+1}` in the HDF5 file. Nothing is matched by name, and
the solid participant's body list must be in the same order. This matters as soon as there is more than one
hydrodynamic body; see [Added mass in Chrono::FSI-TDPF](@ref manual_fsi_tdpf_added_mass) and the
`rm3_reg_waves` example.

**Time stepping.** Unlike the other two, this adapter advances the whole coupling window in a single call.
A TDPF solver has no internal time step of its own: it evaluates the hydrodynamic loads for the given
interval directly, so there is nothing to sub-cycle.

**Added mass.** This adapter can take part in a dynamic added mass exchange when the preCICE configuration
declares the added mass meshes. A fluid participant sends increments relative to the baseline added mass held
by the solid participant, and the TDPF added mass is the constant infinite-frequency added mass that the solid
participant reads from the same hydrodynamic file as its baseline. The increments sent are therefore zero;
the path exists for fluid solvers whose added mass does change during the simulation.

**Checkpointing.** Not available, for the same reason as the SPH adapter.

## Writing a custom adapter

A solver that is not one of the three can be exposed by deriving from
[ChPreciceAdapter](@ref chrono::ch_precice::ChPreciceAdapter) and implementing its pure virtual interface:
`InitializeParticipant` to build and register the coupling meshes, `OnReadData` and `OnWriteData` to
translate between preCICE data and solver state, `AdvanceParticipant` to take a step, `OnReadCheckpoint` and
`OnWriteCheckpoint` for implicit schemes, and `OnWriteOutput` for simulation output. `GetSolverTimeStep` may
be overridden to cap the step at what the solver can take.

The buoyancy participant in `demo_PRECICE_sphere_drop` is a complete, short example: it registers a
single-vertex mesh, computes a buoyancy and drag force from the received position and velocity, and writes
the result back. It is a useful starting point, and a useful stand-in when debugging a coupled setup, since
it removes the fluid solver from the picture without changing anything on the solid side.
