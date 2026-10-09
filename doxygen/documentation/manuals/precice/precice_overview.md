Chrono::preCICE Overview {#module_precice_overview}
==========================================

\tableofcontents

## What the module provides

preCICE couples independent solvers into one simulation without merging them into one program. Every solver
becomes a *participant*: a separate process that, once per coupling time window, reads the data the other
participants produced, advances its own state, and writes the data they need.

Chrono::preCICE supplies the code that makes a Chrono solver behave like such a participant.
[ChPreciceAdapter](@ref chrono::ch_precice::ChPreciceAdapter) is an abstract base class that owns the
preCICE participant object, the coupling meshes and their data, and the simulation loop. Each concrete
adapter fills in what is specific to one Chrono solver: how to build the coupling meshes from Chrono
modeling objects, how to turn received data into loads or prescribed states, how to extract the data to
send, and how to advance the solver. The three adapters are described in
[The Chrono preCICE adapters](@ref precice_adapters).

The module requires preCICE 3.0 or newer and is enabled with the CMake option `CH_ENABLE_MODULE_PRECICE`,
which is off by default.

## The two configuration files

A coupled run is described by two kinds of file, and the division between them matters.

**The preCICE configuration (XML)** is shared by all participants and is authoritative on everything that
concerns more than one of them: which meshes and data blocks exist and their dimensions, which participant
provides or receives each mesh, how data is mapped between non-matching meshes, the coupling scheme, the
time window size, and how many windows are run. Every participant is given the same file.

**The Chrono participant specification (YAML)** describes one participant only: the Chrono model, the solver
settings, output and run-time visualization, and a `precice_adapter_config` object that ties the Chrono
model to the meshes named in the XML. Its keys are:

| Key | Required | Meaning |
|-----|----------|---------|
| `participant_name` | yes | must match a `<participant>` name in the preCICE XML |
| `interfaces` | yes | list of coupling meshes; each names a `mesh_name`, a `mesh_type`, and its `read_data` and `write_data` blocks |
| `bodies` | adapter-specific | the Chrono bodies exposed on the interface, in a significant order |
| `data_path` | no | file handler for any data files referenced (meshes, hydrodynamic data) |
| `angle_degrees` | no | angle units used in this file |
| `read_data_time` | no | when within the window to sample data, see [coupling schemes](@ref precice_coupling) |
| `added_mass` | no | added mass specification, consumed by the MBS adapter only |

The adapter cross-checks the two files during initialization: meshes and data blocks declared in the YAML
must exist in the XML with matching dimensions, and a mesh the XML says this participant provides must be
specified in the YAML. Mismatches are reported as errors rather than silently ignored.

Because a participant never reads the others' YAML files, anything that must agree between phases and is not
carried over the coupling interface has to be set consistently by hand. Gravity is the usual example: each
participant reads its own, and nothing reconciles them.

## Coupling meshes and data

A coupling mesh is a set of vertices with an agreed meaning. The mesh type states that meaning:

| `mesh_type` | Vertices are |
|-------------|--------------|
| `RIGID_BODY_REFS` | one vertex per rigid body, at its reference frame |
| `RIGID_BODY_POINTS` | points distributed over rigid bodies |
| `FEA_MESH_NODES` | nodes of an FEA mesh |
| `FEA_MESH_POINTS` | points on FEA meshes |
| `GENERIC` | anything else, interpreted by the participant |

The choice between the first two decides the fidelity of the exchange. With `RIGID_BODY_REFS` the fluid
sends one resultant force and torque per body and receives one pose and velocity; with `RIGID_BODY_POINTS`
the exchange is distributed over a point cloud on the body surface, which is what a CFD code coupled through
a moving wall needs. The `sphere_drop` and `rm3_reg_waves` examples use the former, `flap_openfoam` the
latter.

Each data block declared on a mesh carries a type, which is how the adapter knows what to do with the
numbers: `POSITIONS`, `ROTATIONS`, `DISPLACEMENTS`, `LINEAR_VELOCITIES`, `ANGULAR_VELOCITIES`, `FORCES`,
`TORQUES`, or `GENERIC`. The name of the block is arbitrary and only has to match the XML; the type is what
gives it meaning. A block declared in the YAML but not referenced by this participant in the XML is marked
unused and skipped, so one YAML file can serve configurations that exchange different subsets of data.

## The simulation loop

An adapter is driven through three calls:

- `InitializeSimulation()` creates the preCICE participant, validates the configuration, lets the concrete
  adapter build and register its coupling meshes, writes initial data if preCICE asks for it, and completes
  the preCICE handshake with the other participants.
- `RunSimulation()` runs the coupled loop until preCICE reports that coupling is over.
- `FinalizeSimulation()` shuts the solver down and finalizes preCICE.

Each iteration of the loop performs, in order: write a checkpoint if preCICE requests one; agree on the step
size; read data from the other participants; advance the Chrono solver by that step; write data; advance the
preCICE coupling; and either restore a checkpoint or move time forward. Applications that need to interleave
their own work can drive the individual steps instead of calling `RunSimulation()`.

Note that an adapter terminates on preCICE's coupling status alone. The simulated duration is set by the
`max-time-windows` and `time-window-size` entries of the preCICE XML; an `end_time` in a participant YAML is
never read, which is why the shipped participant files set it to `-1`.

## Running a coupled simulation

Each participant is a separate process, launched with its own participant name and the shared preCICE
configuration. The demos accept both on the command line, so a Chrono-to-Chrono run is started as two
commands, in either order, from the same working directory:

~~~{.sh}
demo_PRECICE_sphere_drop -p Solid      -c ../data/precice/sphere_drop/precice_config_AM_explicit.xml
demo_PRECICE_sphere_drop -p Fluid_TDPF -c ../data/precice/sphere_drop/precice_config_AM_explicit.xml
~~~

The two processes find each other through the mechanism declared in the XML — the shipped examples use
`<m2n:sockets>` with an exchange directory, so both must be started from the same directory. The first
process to start waits for the other. If one participant fails during startup, the other will wait
indefinitely rather than exit, so a failed run should be cleaned up before the next attempt.

## Examples

The module ships three examples under `data/precice`, with drivers in `src/demos/precice`:

- **`sphere_drop`** — a sphere released into water, with one Chrono multibody participant and a choice of
  fluid participant: Chrono::FSI-SPH, Chrono::FSI-TDPF, or a mock-up solver that applies only buoyancy and
  drag. The mock-up is implemented inside the demo itself and is a compact example of writing a custom
  adapter by deriving from `ChPreciceAdapter` directly.
- **`rm3_reg_waves`** — the RM3 two-body point absorber in regular waves, coupling a Chrono multibody
  participant to Chrono::FSI-TDPF. Its two hydrodynamic bodies exercise the ordering and added mass
  considerations discussed in [added mass](@ref manual_fsi_tdpf_added_mass).
- **`flap_openfoam`** — a flap in channel flow, coupling a Chrono multibody participant to OpenFOAM. This is
  the example to read for coupling to a non-Chrono solver and for a distributed surface load exchanged on a
  `RIGID_BODY_POINTS` mesh.
