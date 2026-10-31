Physics Item Lifecycle {#physics_item_lifecycle}
==========================

This manual describes how a ChPhysicsItem is initialized, set up, and updated during a simulation, and how checkpoints save and restore its state.
It is written for developers who implement new physics items, FEA elements, or FEA nodes, and for anyone who changes ChSystem, ChAssembly, ChMesh, or the checkpoint classes.

[TOC]

# Terminology {#lifecycle_terms}

Several unrelated functions in Chrono have "initialize" or "setup" in their name. This manual uses the following terms:

- **Construction-time initialization**: functions such as `ChLinkLock::Initialize(body1, body2, frame)`, `ChLinkNodeFace::Initialize(...)`, or the `Initialize(...)` functions of Chrono::Vehicle subsystems. They connect an item to other items and set its parameters while the model is being built. They are part of building the model, not of the lifecycle described here.
- **Reference configuration**: the stress-free configuration of an item, for example the undeformed shape of an FEA element. Elements record it from the current node positions when they are initialized.
- **Initial conditions**: the values of an item's coordinates (positions, velocities) and internal states at the start of the simulation.
- **Internal state**: data an item carries from step to step that is not part of the system state vectors, for example the plastic strains of a ChElementBeamIGA.
- **Item**: a ChPhysicsItem. **Container**: an item that owns other items or sub-objects: ChAssembly (and ChModalAssembly), ChMesh (nodes and elements), ChFEModel, ChPeridynamics.

# Overview {#lifecycle_overview}

Every step runs, in this order:

1. **Initialize what is new**, in two phases, each across all items before the next phase starts:
   - Phase 1, `SetupInitial()`: structure and reference configuration.
   - Phase 2, `SetupInitialState()`: initial conditions.
2. `Setup()`: counts, offsets, active coordinates.
3. `Update()`: auxiliary data from the current state.

Step 1 does nothing for items that are already initialized, so in a running simulation its cost is a flag check.

The design rests on three principles:

- **Each item initializes itself, once.** Initialization is tracked per item (and per FEA node and element), so an item added at any time is initialized alone, and the reference configuration of an item that is already initialized is never recorded again by accident.
- **Two phases, with a clear contract.** Phase 1 records every reference configuration before phase 2 sets any initial condition, so an item can set initial conditions that deform other items (for example, pretension a mesh) without depending on the order in which items are visited.
- **One-time work and per-step work are separate.** Anything that can change during a simulation (for example, which nodes are fixed) is evaluated in `Setup()`, every time it runs.

# Initialization {#lifecycle_initialization}

## Per-item initialization

- ChPhysicsItem has a private initialization stage, `m_init_stage`, with values `NONE`, `STRUCTURE` (phase 1 done) and `STATE` (phase 2 done). A copy (copy constructor or `Clone()`) and an item restored with `ArchiveIn()` start at `NONE`.
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

  A container's own hook runs before its children's hooks. A container that needs results from its children (for example totals) computes them in `Setup()`.

- `ChSystem::Initialize()` is cheap and safe to call at any time:

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

  `m_items_pending` avoids walking the item tree on every step. `ChAssembly::Add*()` (and `FlushBatch()` through them), `ChModalAssembly::AddInternal*()`, `ChMesh::AddNode()`, `ChMesh::AddElement()` and `ResetInitialization()` set it, the latter two through the protected helper `ChPhysicsItem::SetInitializationPending()`.

- Code that needs initialized items calls a wrapper, never a raw hook:
  - `ChSystem::Setup()` calls `InitializeItems()` first (but not the one-time system work, which is in `Initialize()`).
  - `ChMesh::ComputeMassProperties()` runs phase 1 on the mesh (only nodes and elements not yet initialized run their hooks).
  - `ChModalAssembly::GetSubassemblyMatrices()` and `DoModalReduction()` run both phases on the modal assembly (`ChModalAssembly::InitializeItems()`).
  - Checkpoint loading calls `ChSystem::Setup()`, which initializes items first (see [Checkpoints](@ref lifecycle_checkpoint)).
  - The beam extruders (`ChExtruderBeamEuler`, `ChExtruderBeamIGA`) call `SetupInitial(ChSystem*)` directly on elements they create or modify during a simulation. A new element is initialized again by its mesh at the next step; an existing element whose knots changed keeps the reference the extruder recorded.

- Public API on ChPhysicsItem:
  - `void InitializePhase(InitPhase phase)`: the wrapper above. Requesting phase 2 on an item that has not completed phase 1 runs phase 1 first.
  - `bool IsInitialized() const`: true once both phases ran.
  - `void ResetInitialization()`: sets the stage back to `NONE` (for containers, also for all children, including FEA nodes and elements), so the next `ChSystem::Initialize()` reruns both phases and the item records a new reference configuration from its current state. This is the supported way to re-record a reference on purpose.

## Containers

Containers forward through `InitializeChildren(phase)`:

- **ChAssembly**: all its lists (bodies, shafts, links, meshes, other items). The order between items is not part of the contract.
- **ChModalAssembly**: ChAssembly's lists, then its internal lists.
- **ChMesh**: nodes, then elements. FEA nodes and elements are not physics items, so ChNodeFEAbase and ChElementBase each have their own initialization flag (private, accessed by ChMesh as a friend), and ChMesh calls their `SetupInitial(ChSystem*)` only if the flag is not set. A node or element added to an initialized mesh is therefore initialized alone, and the other elements keep their reference configuration. Fixed nodes are initialized like any other node.
- **ChFEModel** (multiphysics) sets up its data maps and initializes its elements in its own `SetupInitial()`.
- **ChPeridynamics** sets up its nodes and materials in its own `SetupInitial()`, which recomputes its data from scratch. `ChPeridynamics::SetupInitialBonds()` calls it again after collision detection has created the bonds.

## Two phases

| Phase | Hook | What belongs here | May assume |
|-------|------|-------------------|------------|
| 1 | `SetupInitial()` (private virtual, empty by default) | Allocate and size data (including internal state such as plasticity data); precompute constants (for example element stiffness matrices); record the reference configuration from the current state | The item's own children have completed phase 1 if the container's documented order says so (for example nodes before elements in ChMesh). Nothing about other items. |
| 2 | `SetupInitialState()` (private virtual, empty by default) | Set initial conditions: initial coordinates and velocities, initial values of internal states (for example ODE states, hydraulic pressures). May also set the current coordinates of items it is coupled to, for example stretching a mesh to pretension it. | All items in the system have completed phase 1, so every reference configuration is recorded. |

Rules:

- Phase 1 must not change the current coordinates of any item. Phase 2 must not change any reference configuration.
- The order of items within a phase is not part of the contract. An item that needs another item's phase 1 results does that work in phase 2. If two items' phase 2 hooks write the same coordinates, the result is undefined; such items must be combined or coupled explicitly.
- Running phase 1 early on part of the system (for example `ChMesh::ComputeMassProperties()`) is allowed. Phase 2 runs through `ChSystem::Initialize()`, or for a single item through the `Initialize()` and `GetInitialStates()` functions of the external dynamics items below.
- Neither phase counts coordinates or constraints; that is the job of `Setup()`.

Items with initial conditions:

- **ChExternalDynamicsODE / ChExternalDynamicsDAE**: `SetupInitial()` sizes the states and allocates the `ChVariables` and Jacobian storage; `SetupInitialState()` calls `SetInitialConditions(...)`. For a DAE not in explicit form, the mass matrix is evaluated once, right after the initial conditions are set. These items do not need an explicit initialization call; the deprecated `Initialize()` runs both phases on the item through the wrapper, and later calls have no effect. `GetInitialStates()` (and `GetInitialStateDerivatives()` for a DAE) initializes the item first if needed.
- **ChHydraulicActuatorBase**: `SetupInitialState()` calls `OnInitialize(...)`, then the parent's `SetupInitialState()`. Construction-time `Initialize(body1, body2, ...)` connects the bodies only.
- **ChExternalFmu**: `SetupInitial()` initializes the FMU (with the user's initial conditions and parameters) and then the parent; `SetupInitialState()` reads the FMU's continuous states into the ODE states through `SetInitialConditions(...)`. Setters that must be called before initialization check `IsInitialized()`.
- **Pretensioning and similar setups** (for example spokes stretched after the mesh recorded its reference): an item overrides `SetupInitialState()`.

## Active coordinates

- `Setup()` reads the fixed status of every node and body each time it runs. ChMesh calls the virtual `ChNodeFEAbase::UpdateActiveCoords()` (empty by default, overridden by ChNodeFEAxyzD, DD and DDD) for every node in `ChMesh::Setup()`, before counting.
- `SetFixed()` and related setters of those nodes also call `UpdateActiveCoords()`, so the count is current between steps as well.

## What belongs in each hook

| Hook | Runs | Must | Must not |
|------|------|------|----------|
| `SetupInitial()` | once per item, until `ResetInitialization()` | allocate, precompute, record the reference configuration | change current coordinates; count coordinates; read other items' phase 2 results |
| `SetupInitialState()` | once per item, after all items finished phase 1 | set initial conditions | change a reference configuration |
| `Setup()` | at least once per step, and before analyses | count coordinates and constraints, assign offsets, update active coordinates | change the state; allocate data whose contents must persist |
| `Update(time, flags)` | many times per step | recompute auxiliary data from the current state | change the reference configuration or the initial conditions |

End-of-step commit functions, such as `ChElementBase::EleDoIntegration()` (which commits the plastic strains of ChElementBeamIGA), are not part of the lifecycle.

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

A new FEA element overrides `SetupInitial(ChSystem*)` to record its reference configuration and allocate its data; its mesh calls it once. If the element carries internal state, it overrides the element-level `SaveInternalState()`/`LoadInternalState()` (see below).

A new container overrides `InitializeChildren(phase)` and calls the wrappers (or, for children that are not physics items, the hook guarded by the child's flag), and forwards `ResetInitialization()`, `SaveInternalState()` and `LoadInternalState()` to its children.

# Checkpoints {#lifecycle_checkpoint}

Checkpoints are built on top of the lifecycle. The classes are ChCheckpoint and ChCheckpointASCII in `src/chrono/input_output/`; the format-independent internal-state blocks are ChCheckpointWriter and ChCheckpointReader in `ChCheckpointBlock.h`.

## What a checkpoint contains

- A checkpoint saves **current** state only: coordinates, velocities, accelerations, Lagrange multipliers, and internal state. It does not save reference configurations; those are rebuilt by phase 1 in the system that loads the checkpoint, which must be built the same way as the system that wrote it.
- Accelerations are saved because some items read them when computing forces or Jacobians (for example the inertial terms of ChElementBeamIGA use node accelerations). The timestepper stores them in the items at the end of each step.
- The Lagrange multipliers of the assembly constraints are saved because some integrators use them as part of their history (for example HHT). Contact multipliers are not saved, since contacts are recreated at each step.
- Bodies and rotational FEA nodes (ChNodeFEAxyzrot) save the derivatives of their rotation quaternion as internal state. Their coordinates hold the angular velocity and acceleration, which do not convert back to the same quaternion derivatives exactly.
- The timestepper saves its own internal state (`ChTimestepper::SaveInternalState()`); for the implicit integrators, this is the step size control state (internal step size and number of successive successful steps).
- An item whose reference configuration can change during a run (because of `ResetInitialization()`, or by design) must save its reference as part of its internal state.
- The checkpoint time and states are written with full precision, so that time-dependent inputs (motor functions, loads) see exactly the same time after a restart.

A SYSTEM checkpoint (ASCII) stores the position and velocity vectors of the whole system, followed by one line holding a block with the system acceleration vector, the internal state of all items (from `ChSystem::SaveInternalState()`), the Lagrange multipliers of the assembly constraints, and the timestepper type and internal state. The timestepper state is restored only into a timestepper of the same type. A file that ends earlier loads without the missing data.

## Loading order

Loading a checkpoint into a system, either freshly built or already running:

1. Item initialization (`ChSystem::InitializeItems()`, run by `ChSystem::Setup()`): both phases for every item not yet initialized. In a fresh system, this records the reference configurations from the construction state and sets the initial conditions. In a running system, it does nothing for existing items, so their references are kept.
2. The rest of `Setup()`, then check that the coordinate counts match the checkpoint.
3. Load the timestepper state.
4. Scatter the saved coordinates and velocities (which also updates the items), then the accelerations and the Lagrange multipliers. This overwrites what phase 2 set.
5. Load the internal state (`LoadInternalState()`). This overwrites internal state that phase 2 set (for example ODE states) and may refine data set by the scatter (for example the quaternion derivatives of bodies).
6. Update the system with the complete state.

Because phase 2 runs before the state is loaded, an item must not assume that the state it set in phase 2 survives; anything it needs to keep must be part of the coordinates or the internal state.

## Internal state hooks

- ChPhysicsItem has virtual `SaveInternalState(ChCheckpointWriter&) const` and `LoadInternalState(ChCheckpointReader&)`, empty by default. Overrides call the parent first.
- `virtual bool SupportsCheckpoint() const` returns true by default. An item whose internal state cannot be saved returns false, and saving a checkpoint that includes it fails with an error instead of silently producing a checkpoint that restarts from the wrong state. Containers return false if any child does; a reduced ChModalAssembly returns false (its floating frame and modal data are not saved).
- ChCheckpointWriter accepts numbers (integral values are stored exactly), ChVector3d, ChQuaterniond, `std::vector<double>`, Eigen matrices and vectors (with their dimensions), opaque byte sequences, and nested blocks. ChCheckpointReader reads them back in the same order; every read checks the type of the next entry and throws on a mismatch or on missing data, and the caller checks that a block was read completely. The ASCII format writes a block as one line of space-separated entries, with byte sequences as hexadecimal tokens.
- Containers write their children's blocks with `ChCheckpointWriter::WriteIndexedBlocks()`: each non-empty child block is tagged with the child's index, and loading checks the index range and that each child consumes its whole block.
- ChAssembly forwards the calls to all its items (ChModalAssembly also to its internal items). ChMesh writes its numbers of nodes, elements and fixed nodes (checked when loading), then the blocks of its nodes and elements; ChNodeFEAbase and ChElementBase have the same optional virtuals.
- ChTimestepper has the same pair of virtual functions for the integrator's own state; ChTimestepperImplicit saves its step size control state.
- Items and nodes with internal state: ChBody and ChNodeFEAxyzrot (quaternion derivatives).
- Elements with internal state: ChElementBeamIGA (plasticity data at the integration points, through `ChBeamMaterialInternalData::SaveCheckpoint()`/`LoadCheckpoint()`), ChElementHexaANCF_3813_9 (plastic strains and hardening parameters), ChElementHexaANCF_3813 (enhanced assumed strain parameters).

## COMPONENT checkpoints

A generic per-item path complements the per-type functions: for each item, its coordinates, velocities and accelerations through `IntStateGather()`/`IntStateScatter()` and `IntStateGatherAcceleration()`/`IntStateScatterAcceleration()` with local offsets, its reactions through `IntStateGatherReactions()`/`IntStateScatterReactions()` (for example, the multipliers of a ChExternalDynamicsDAE), then its internal state block, all on one line. When loading, the internal state is loaded after the coordinates, accelerations and reactions, and the item is then updated. `ChAssembly::Components` has a `meshes` list (whole meshes, separate from the output-only `mesh_nodes`) and an `items` list (any other physics item); both use this path, after the per-type lists. Loading sets up the containing systems first, which initializes items that are not yet initialized.

Chrono::Vehicle deformable tires (ChFEATire and derived) add their mesh to the `meshes` list of their components, and implement `WriteCheckpoint()`/`ReadCheckpoint()` with a COMPONENT checkpoint of those components.

## Integrators

A restart reproduces a continuous run exactly when everything the integrator carries from one step to the next is part of the checkpoint: the state, the accelerations, the Lagrange multipliers (held by the items) and the timestepper state. For this reason, HHT evaluates the constraint Jacobians at the state at the beginning of the step when it forms the residual term with the previous multipliers, rather than using the Jacobians cached in the constraint objects by the last linear solve.

## Known limits

- ChExternalFmu saves its continuous states (they are part of the system state vectors), but not the internal state of the FMU itself (discrete states, event indicators, and outputs cached from the last evaluation, which may have been a perturbed state used for a finite-difference Jacobian). Restarts with FMUs are therefore close to, but not exactly, a continuous run.
- The state of the collision system is not saved. With contact, a restart in a freshly built system can process contacts in a different order than the continuous run (a reload into the running system does not), which gives roundoff-level differences that grow over time.
- The matrices and factorizations kept by the solver are not saved. With a Jacobian update strategy that reuses a Jacobian across steps (`JacobianUpdate::NEVER` or `AUTOMATIC`), a restart evaluates a new Jacobian and is close to, but not exactly, a continuous run.
- Items with hidden state that is not saved: ChPeridynamics (bonds and their broken status), ChFunctionSetpoint and other stateful functions, contact history (for example tangential displacement in SMC contact), the SCM terrain grid, Chrono::Vehicle drivers and powertrain internals outside the physics items, and the shared web mesh of ANCF band tracks (which is not in any component list).

# Design decisions {#lifecycle_decisions}

- FEA nodes and elements have no phase 2 hook. Phase 2 on the containing mesh, or on a separate item, covers the cases that need initial conditions on nodes or elements.
- Chrono::Peridynamics uses its own initialization, including `ChPeridynamics::SetupInitialBonds()`, which reruns `SetupInitial()` after the bonds are created.
