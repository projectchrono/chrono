Physics Item Lifecycle {#physics_item_lifecycle}
==========================

This manual describes how a ChPhysicsItem is initialized, set up, and updated during a simulation, and how checkpoints restore its state.
It is written for developers who implement new physics items, FEA elements, or FEA nodes, and for anyone who changes ChSystem, ChAssembly, ChMesh, or the checkpoint classes.

It is also the design document for a redesign of the lifecycle.
Sections [Current lifecycle](@ref lifecycle_current) and [Problems](@ref lifecycle_problems) describe Chrono as it is today.
Sections from [New design](@ref lifecycle_design) onward describe the target design, which is being implemented in stages; until then, the code follows the current lifecycle.

[TOC]

# Terminology {#lifecycle_terms}

Several unrelated functions in Chrono have "initialize" or "setup" in their name. This manual uses the following terms:

- **Construction-time initialization**: functions such as `ChLinkLock::Initialize(body1, body2, frame)`, `ChLinkNodeFace::Initialize(...)`, or the `Initialize(...)` functions of Chrono::Vehicle subsystems. They connect an item to other items and set its parameters while the model is being built. They are part of building the model, not of the lifecycle described here, and are not affected by this design.
- **Reference configuration**: the stress-free configuration of an item, for example the undeformed shape of an FEA element. Elements record it from the current node positions the first time they are initialized.
- **Initial conditions**: the values of an item's coordinates (positions, velocities) and internal states at the start of the simulation.
- **Internal state**: data an item carries from step to step that is not part of the system state vectors, for example the plastic strains of a ChElementBeamIGA or the internal state of an FMU.
- **Item**: a ChPhysicsItem. **Container**: an item that owns other items or sub-objects: ChAssembly (and ChModalAssembly), ChMesh (nodes and elements), ChFEModel, ChPeridynamics.

# Current lifecycle {#lifecycle_current}

## Hooks

Each item takes part in the simulation through three hooks:

| Hook | Called | Purpose today |
|------|--------|---------------|
| `SetupInitial()` (private virtual) | from `ChSystem::Initialize()` and from several other places (see below) | Record the reference configuration, precompute constants (for example element stiffness matrices), allocate data, count active coordinates |
| `Setup()` | at the start of every step, by `ChSystem::Setup()` | Count coordinates and constraints; assign offsets in the system state vectors |
| `Update(time, flags)` | at least once per step, often many times (once per Newton iteration) | Recompute auxiliary data from the current state and time (rotation matrices, forces, visual assets) |

FEA nodes and elements are not physics items. ChMesh calls their own `SetupInitial(ChSystem*)` from its `SetupInitial()`, and their update functions from its `Setup()` and `Update()`.

## ChSystem::Initialize

`ChSystem::Initialize()` runs once, guarded by the system-wide flag `ChSystem::is_initialized`. It calls `assembly.SetupInitial()`, then initializes the collision system and the visualization system. All analysis functions (`DoStepDynamics`, `DoAssembly`, `DoStaticLinear`, and so on) and `ChSystem::Update()` call it, so a user does not have to.

`ChAssembly::SetupInitial()` visits its lists in a fixed order: bodies, shafts, links, meshes, then other physics items.

## Initial conditions

There is no common hook for initial conditions. Each item type that needs them has its own mechanism:

- ChExternalDynamicsODE and ChExternalDynamicsDAE require the user to call `Initialize()`, which sizes the item and evaluates the virtual `SetInitialConditions()`.
- ChHydraulicActuatorBase adds `Initialize()` and the virtual `OnInitialize(...)`, which computes the initial pressures from the cylinder length.
- ChExternalFmu initializes its FMU in `Initialize()` and keeps a private `m_initialized` flag to reject parameter changes afterwards.
- Other items use the order in which `ChAssembly::SetupInitial()` visits them. For example, an item in the "other physics items" list can rely on all meshes having recorded their reference configuration, and then stretch the mesh to pretension it.

# Problems {#lifecycle_problems}

1. **A system-wide flag controls a per-item operation.**
   `ChAssembly::AddMesh`, `ChAssembly::AddBatch`, `ChMesh::AddNode` and `ChMesh::AddElement` reset `ChSystem::is_initialized`. Adding one mesh (or one node) after the simulation started therefore reruns `SetupInitial()` on every item in the system. Meshes that are already deformed record their deformed shape as their new reference configuration, which silently removes their elastic forces.

2. **`SetupInitial()` is not idempotent, yet it is called from several places.**
   - `ChSystem::Setup()` calls `assembly.SetupInitial()` whenever `is_initialized` is false, but does not set the flag, so it repeats on every call to `Setup()` before `Initialize()`. This includes the `Setup()` in `ChCheckpointASCII::LoadState()`.
   - `ChMesh::ComputeMassProperties()` calls it.
   - `ChModalAssembly::GetSubassemblyMatrices()` and `ChModalAssembly::DoModalReduction()` call it.
   - `ChPeridynamics::SetupInitialBonds()` calls it, on purpose, a second time after bonds are created.
   - `SCMLoader::Initialize(...)` calls its own `SetupInitial()`, which appends a default active domain on every call.

3. **One hook does four jobs.** `SetupInitial()` records the reference configuration, precomputes constants, sizes data, and counts active coordinates. These jobs need to run at different times: counting active coordinates depends on which nodes are fixed, and that can change at any time.
   This caused a stale active coordinate count in ANCF gradient nodes (ChNodeFEAxyzD, DD, DDD) after `SetFixed()`, fixed with `UpdateActiveCoords()` in those nodes.
   ChMesh also skips `SetupInitial()` of fixed nodes, so a node that is fixed during initialization and released later never runs it.

4. **Ordering between items is implicit.** Item code that depends on another item being initialized relies on the list order in `ChAssembly::SetupInitial()`, which is not documented and differs between containers (ChModalAssembly visits its internal lists after all of ChAssembly's lists).

5. **Initial conditions are inconsistent.** Each item type has its own initialization call (see above). Forgetting the call, or calling it twice, is not detected uniformly.

6. **Checkpoint loading ignores the lifecycle.** `ChCheckpointASCII::LoadState()` calls only `Setup()` before it scatters the saved state. In a freshly built system that has never been initialized, `SetupInitial()` runs inside that `Setup()` (by accident, through problem 2), before the state is scattered, so the reference configuration happens to be correct. Any change to problem 2 without a deliberate design would make elements record the loaded, deformed state as their reference.

# New design {#lifecycle_design}

The redesign keeps the existing hook names, `SetupInitial()`, `Setup()` and `Update()`, to minimize the change for the ~55 FEA classes and for user code that overrides them. Names that should be consolidated later are collected in [Future naming cleanup](@ref lifecycle_naming).

## Overview

Every step runs, in this order:

1. **Initialize what is new**, in two phases, each across all items before the next phase starts:
   - Phase 1, `SetupInitial()`: structure and reference configuration.
   - Phase 2, `SetupInitialState()` (new): initial conditions.
2. **`Setup()`**: counts, offsets, active coordinates.
3. **`Update()`**: auxiliary data from the current state.

Step 1 does nothing for items that are already initialized, so in a running simulation its cost is a flag check.

## A. Each item initializes itself, once

- ChPhysicsItem gets a private initialization stage, `m_init_stage`, with values `NONE`, `STRUCTURE` (phase 1 done) and `STATE` (phase 2 done). A copy (copy constructor or `Clone()`) and an item restored with `ArchiveIn()` start at `NONE`.
- A non-virtual wrapper runs the hooks. For a given phase, it runs the hook only if the item has not completed that phase, then forwards to the item's children:

~~~{.cpp}
void ChPhysicsItem::InitializePhase(InitPhase phase) {
    if (phase == InitPhase::STATE && m_init_stage == InitPhase::NONE)
        InitializePhase(InitPhase::STRUCTURE);  // phase 2 requires phase 1
    if (m_init_stage < phase) {
        if (phase == InitPhase::STRUCTURE)
            SetupInitial();
        else
            SetupInitialState();
        m_init_stage = phase;
    }
    InitializeChildren(phase);  // protected virtual, empty by default; containers override it
}
~~~

  A container's own hook runs before its children's hooks. A container that needs results from its children (for example totals) computes them in `Setup()`, where such counts belong anyway.

- `ChSystem::Initialize()` becomes cheap and safe to call anywhere:

~~~{.cpp}
void ChSystem::Initialize() {
    InitializeItems();  // both phases, for items not yet initialized
    if (!is_initialized) {
        // one-time system work: Eigen threads, collision system, visualization system
        is_initialized = true;
    }
}

void ChSystem::InitializeItems() {
    if (!m_items_pending)
        return;
    m_items_pending = false;
    assembly.InitializePhase(InitPhase::STRUCTURE);  // phase 1, all items
    assembly.InitializePhase(InitPhase::STATE);      // phase 2, all items
}
~~~

  `m_items_pending` is only an optimization that avoids walking the item tree on every step. `ChAssembly::Add*()` (and `FlushBatch()` through them), `ChModalAssembly::AddInternal*()`, `ChMesh::AddNode()`, `ChMesh::AddElement()` and `ResetInitialization()` set it, the latter two through the protected helper `ChPhysicsItem::SetInitializationPending()`. They no longer reset `is_initialized`, and nothing reruns the hooks of items that are already initialized.

- Everything that needs initialized items calls a wrapper, never a raw hook:
  - `ChSystem::Setup()` calls `InitializeItems()` first (not the one-time system work, which stays in `Initialize()`).
  - `ChMesh::ComputeMassProperties()` runs phase 1 on the mesh (only nodes and elements not yet initialized run their hooks).
  - `ChModalAssembly::GetSubassemblyMatrices()` and `DoModalReduction()` run both phases on the modal assembly (`ChModalAssembly::InitializeItems()`).
  - Checkpoint loading calls `ChSystem::Setup()`, which initializes items first (see [Checkpoints](@ref lifecycle_checkpoint)).
  - `SCMLoader::Initialize(...)` stops calling `SetupInitial()` itself, and `SCMLoader::SetupInitial()` creates its default active domain only if there is none.
  - Exception: the beam extruders (`ChExtruderBeamEuler`, `ChExtruderBeamIGA`) still call `SetupInitial(ChSystem*)` directly on elements they create or modify while the simulation runs. A new element is initialized again by its mesh at the next step, as before; an existing element whose knots changed keeps the reference the extruder recorded.

- Public API on ChPhysicsItem:
  - `void InitializePhase(InitPhase phase)`: the wrapper above. Requesting phase 2 on an item that has not completed phase 1 runs phase 1 first.
  - `bool IsInitialized() const`: true once both phases ran.
  - `void ResetInitialization()`: sets the stage back to `NONE` (for containers, also for all children, including FEA nodes and elements), so the next `ChSystem::Initialize()` reruns both phases and the item records a new reference configuration from its current state. This is the only supported way to re-record a reference on purpose.
  - ChExternalFmu replaces its private `m_initialized` with `IsInitialized()`.

- Containers forward through `InitializeChildren(phase)`:
  - **ChAssembly**: all its lists (bodies, shafts, links, meshes, other items). Order between items is not part of the contract (see B).
  - **ChModalAssembly**: ChAssembly's lists, then its internal lists.
  - **ChMesh**: nodes, then elements. FEA nodes and elements are not physics items, so ChNodeFEAbase and ChElementBase each get their own initialization flag (private, accessed by ChMesh as a friend), and ChMesh calls their `SetupInitial(ChSystem*)` only if the flag is not set. A node or element added to an initialized mesh is therefore initialized alone, and existing elements keep their reference configuration. Fixed nodes are no longer skipped.
  - **ChFEModel** (multiphysics) keeps its own `SetupInitial()`, which sets up its data maps and initializes its elements. It now runs once, like any other item's hook.
  - **ChPeridynamics** is unchanged. Its `SetupInitial()` recomputes its data from scratch, so running it once through the wrapper and again from `ChPeridynamics::SetupInitialBonds()` (after collision detection has created the bonds) gives the same result as before.

## B. Two phases

| Phase | Hook | What belongs here | May assume |
|-------|------|-------------------|------------|
| 1 | `SetupInitial()` | Allocate and size data (including internal state such as plasticity data); precompute constants (for example element stiffness matrices); record the reference configuration from the current state | The item's own children have completed phase 1 if the container's documented order says so (for example nodes before elements in ChMesh). Nothing about other items. |
| 2 | `SetupInitialState()` (new, private virtual, empty by default) | Set initial conditions: initial coordinates and velocities, initial values of internal states (for example ODE states, hydraulic pressures). May also set the current coordinates of items it is coupled to, for example stretching a mesh to pretension it. | All items in the system have completed phase 1, so every reference configuration is recorded. |

Rules:

- Phase 1 must not change the current coordinates of any item. Phase 2 must not change any reference configuration.
- The order of items within a phase is not part of the contract. An item that needs another item's phase 1 results does that work in phase 2. If two items' phase 2 hooks write the same coordinates, the result is undefined; such items must be combined or coupled explicitly.
- Running phase 1 early on part of the system (for example `ChMesh::ComputeMassProperties()`) is allowed. Phase 2 runs only through `ChSystem::Initialize()`, apart from the deprecated `Initialize()` calls below.
- Neither phase counts coordinates or constraints; that is `Setup()`'s job (see C).

Migration of the existing initial-condition mechanisms:

- **ChExternalDynamicsODE / ChExternalDynamicsDAE**: sizing and allocating `ChVariables` and Jacobian storage moves to `SetupInitial()`; the call to `SetInitialConditions(...)` moves to `SetupInitialState()`. For a DAE not in explicit form, the mass matrix is still evaluated once, right after the initial conditions are set (in `SetupInitialState()`), as before. Users no longer call `Initialize()`. `Initialize()` stays as a deprecated function that runs both phases on this item through the wrappers, so existing code keeps working and the system does not run them again.
- **ChHydraulicActuatorBase**: `OnInitialize(...)` is called from `SetupInitialState()`, before the parent's call to `SetInitialConditions(...)`. Construction-time `Initialize(body1, body2, ...)` keeps connecting the bodies but no longer initializes the dynamics. A using-declaration keeps the deprecated `Initialize()` visible next to that overload.
- **ChExternalFmu**: its `Initialize()` override is removed; instantiating and initializing the FMU (with the user's initial conditions and parameters) moves to `SetupInitial()`; reading the FMU's continuous states into the ODE states happens in `SetupInitialState()` through `SetInitialConditions(...)`. Setters that must be called before initialization check `IsInitialized()`.
- **Pretensioning and similar setups** (for example spokes stretched after the mesh recorded its reference): an item overrides `SetupInitialState()` instead of relying on list order.

## C. Active coordinates are counted in Setup()

- `Setup()` reads the fixed status of every node and body each time it runs. ChMesh calls a new virtual `ChNodeFEAbase::UpdateActiveCoords()` (empty by default; the existing function in ChNodeFEAxyzD, DD and DDD becomes its override) for every node in `ChMesh::Setup()`, before counting.
- The calls to `UpdateActiveCoords()` in `SetFixed()` and related setters stay as a backstop, so the count is current between steps as well.
- Counting code in `ChMesh::SetupInitial()` is removed (ChPeridynamics is left as it is).

## What belongs in each hook

| Hook | Runs | Must | Must not |
|------|------|------|----------|
| `SetupInitial()` | once per item, until `ResetInitialization()` | allocate, precompute, record the reference configuration | change current coordinates; count coordinates; read other items' phase 2 results |
| `SetupInitialState()` | once per item, after all items finished phase 1 | set initial conditions | change a reference configuration |
| `Setup()` | at least once per step, and before analyses | count coordinates and constraints, assign offsets, update active coordinates | change the state; allocate data whose contents must persist |
| `Update(time, flags)` | many times per step | recompute auxiliary data from the current state | change the reference configuration or the initial conditions |

End-of-step commit functions, such as `ChElementBase::EleDoIntegration()` (which commits the plastic strains of ChElementBeamIGA), are not part of the lifecycle and keep their current role.

## Implementing a new item

A new physics item with a reference configuration and internal state:

~~~{.cpp}
class MyItem : public ChPhysicsItem {
  private:
    // Phase 1: structure and reference configuration.
    virtual void SetupInitial() override {
        m_history.resize(m_num_points);    // allocate internal state
        m_ref_length = ComputeLength();    // record the reference from the current state
    }

    // Phase 2: initial conditions. All items finished phase 1.
    virtual void SetupInitialState() override {
        m_history.setZero();
        m_mesh->StretchTo(m_pretension);  // allowed: changes current coordinates, not references
    }

    // Counts and offsets; called every step.
    virtual void Setup() override { ... }

    // Internal state for checkpoints (see below).
    virtual bool SupportsCheckpoint() const override { return true; }
    virtual void SaveInternalState(ChCheckpointWriter& writer) const override {
        ChPhysicsItem::SaveInternalState(writer);
        writer.Write(m_history);
    }
    virtual void LoadInternalState(ChCheckpointReader& reader) override {
        ChPhysicsItem::LoadInternalState(reader);
        reader.Read(m_history);
    }
};
~~~

A new FEA element overrides `SetupInitial(ChSystem*)` to record its reference configuration and allocate its data, exactly as today. It no longer needs to worry about being called more than once. If it carries internal state, it overrides the element-level `SaveInternalState()`/`LoadInternalState()` (see below).

A new container overrides `InitializeChildren(phase)` and calls the wrappers (or, for non-item children, the hook guarded by the child's flag), and forwards `ResetInitialization()`, `SaveInternalState()` and `LoadInternalState()` to its children.

# Checkpoints {#lifecycle_checkpoint}

Checkpoints are built on top of the lifecycle. The classes are ChCheckpoint and ChCheckpointASCII in `src/chrono/input_output/`.

## What a checkpoint contains

- A checkpoint saves **current** state only: coordinates, velocities, and internal state. It does not save reference configurations; those are rebuilt by phase 1 in the system that loads the checkpoint, which must be built the same way as the system that wrote it.
- An item whose reference configuration can change during a run (because of `ResetInitialization()`, or by design) must save its reference as part of its internal state.

Today, SYSTEM checkpoints gather and scatter the full position and velocity vectors, so FEA node states are already included. COMPONENT checkpoints handle bodies, shafts, joints, couples, TSDA/RSDA, bushings and motors only.

## Loading order

Loading a checkpoint into a system, either freshly built or already running:

1. Item initialization (`ChSystem::InitializeItems()`, run by `ChSystem::Setup()`): both phases for every item not yet initialized. In a fresh system, this records the reference configurations from the construction state and sets the initial conditions. In a running system, it does nothing for existing items, so their references are kept.
2. The rest of `Setup()`, then check that the coordinate counts match the checkpoint.
3. Scatter the saved coordinates and velocities. This overwrites what phase 2 set.
4. Load the internal state blocks (`LoadInternalState()`). This overwrites internal state that phase 2 set, for example ODE states or FMU state.
5. `Update()`.

Because phase 2 runs before the state is loaded, an item must not assume that the state it set in phase 2 survives; anything it needs to keep must be part of the coordinates or the internal state.

## Internal state hooks

- ChPhysicsItem gets virtual `SaveInternalState(ChCheckpointWriter&) const` and `LoadInternalState(ChCheckpointReader&)`, empty by default. Overrides call the parent first.
- `virtual bool SupportsCheckpoint() const` returns true by default for items without internal state. Items whose internal state cannot be saved (for example an FMU without `canGetAndSetFMUstate`) return false, and saving a checkpoint that includes them fails with an error instead of silently producing a checkpoint that restarts from the wrong state.
- The writer and reader accept doubles, vectors of doubles, and opaque byte blocks (for example a serialized FMU state). The ASCII format writes byte blocks as hex or base64. Each item's block records the item identifier and the block size, so a mismatch between the saved and loaded systems is detected.
- ChAssembly forwards the calls to all its items. ChMesh forwards them to its nodes and elements; ChNodeFEAbase and ChElementBase get the same optional virtuals.
- SYSTEM checkpoints write the internal state blocks after the state vectors.

## COMPONENT checkpoints

A generic per-item path replaces the per-type functions for new item types: the item's coordinates through `IntStateGather()`/`IntStateScatter()` with local offsets, optionally its reactions through `IntStateGatherReactions()`/`IntStateScatterReactions()`, then its internal state block. `ChAssembly::Components` gets a `meshes` list (whole meshes), separate from the output-only `mesh_nodes`. The existing per-type functions stay so that old checkpoint files still load.

## Known limits

- Lagrange multipliers and the history of multistep timesteppers (for example HHT) are not saved, so restarts with such integrators are close to, but not exactly, a continuous run. Saving them is a separate, optional extension.
- Contact history (for example tangential displacement in SMC contact) and the internal state of controllers outside the physics items (vehicle drivers, setpoint functions) need to be audited item by item.

# Behavior changes {#lifecycle_changes}

- Adding a mesh, node, element, or batch after the first step no longer re-records the reference configuration of existing items. Use `ResetInitialization()` on an item to re-record its reference on purpose.
- `ChMesh::ComputeMassProperties()` no longer re-records the reference configuration of the mesh elements on every call. Before, any call during a simulation (for example through the inertia properties of a Chrono::Vehicle FEA tire or ANCF track shoe) made the elements treat their current, deformed shape as stress free.
- `ChSystem::Setup()` called before `ChSystem::Initialize()` no longer reruns `SetupInitial()` on every call.
- ChExternalDynamicsODE, ChExternalDynamicsDAE, ChHydraulicActuator2/3 and ChExternalFmu no longer need a call to `Initialize()`; existing calls keep working.
- Item code that relied on the visiting order of `ChAssembly::SetupInitial()` should move cross-item work to `SetupInitialState()`.
- Loading a checkpoint into a freshly built, never-initialized system initializes it first, so reference configurations come from the construction state, not from the loaded state.

# Future naming cleanup {#lifecycle_naming}

The redesign deliberately keeps existing names. The following candidates are collected for a later, separate cleanup; none of them is renamed now:

- `SetupInitial()` and `SetupInitialState()` describe neither phase well. Candidates: `InitializeStructure()`/`InitializeState()`, or `SetupReference()`/`SetupInitialConditions()`.
- `SetupInitial()` takes no argument on ChPhysicsItem but `SetupInitial(ChSystem*)` on FEA nodes and elements.
- Initial-condition functions with divergent names: `ChExternalDynamicsODE::SetInitialConditions()`, `ChHydraulicActuatorBase::OnInitialize()`, `ChExternalFmu::SetInitialCondition()` (a user setter, not a hook).
- "Initialize" means three different things: construction-time connection (`ChLinkLock::Initialize(...)`, `ChLinkNodeFace::Initialize(...)`), one-time system setup (`ChSystem::Initialize()`), and item initialization (`ChExternalDynamicsODE::Initialize()`, deprecated by this design).
- Initialization flags: `ChSystem::is_initialized` versus `m_initialized` in ChExternalFmu, ChLinkNodeFace, ChVisualSystem, ChCollisionSystem, and Chrono::Vehicle's ChPart.
- `ChPeridynamics::SetupInitialBonds()` and `SCMLoader::SetupInitial()` call lifecycle hooks directly.
- Active-coordinate queries: `GetNumCoordsPosLevelActive()`, `m_dof_actual`, `UpdateActiveCoords()`.
- Checkpoint functions: `ChCheckpoint::SaveState()/LoadState()` versus `Save()/Load()` in the class documentation, and the per-type `SaveBodies()`, `SaveJoints()`, ... functions once the generic COMPONENT path exists.

# Design decisions {#lifecycle_decisions}

- The names of the new functions (`InitializePhase()`, `InitializeChildren()`, `InitPhase`, `IsInitialized()`, `ResetInitialization()`, `SetupInitialState()`, `SaveInternalState()`/`LoadInternalState()`, `SupportsCheckpoint()`, `ChCheckpointWriter`/`ChCheckpointReader`) are accepted for now; they are candidates for the [future naming cleanup](@ref lifecycle_naming).
- FEA nodes and elements get no phase 2 hook. Phase 2 on the containing mesh, or on a separate item, covers the known cases; a node or element hook can be added later if a need appears.
- Chrono::Peridynamics keeps its current initialization, including `ChPeridynamics::SetupInitialBonds()`, and its behavior must not change.
