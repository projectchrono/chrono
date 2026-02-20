Chrono::FSI-SPH Class Guide {#manual_fsi_sph_class_guide}
===========================================================

Scope
-----

This document covers the **SPH implementation** in Chrono FSI (not TDPF), with focus on:

- the primary classes in `src/chrono_fsi/` and `src/chrono_fsi/sph/`
- who inherits from whom
- how these classes are used in `src/demos/fsi/sph/` and `src/demos/vehicle/`
- when each class is the right choice for users

Quick Mental Model
------------------

For SPH-FSI, think in layers:

1. Generic FSI coupling layer (`ChFsiSystem`, `ChFsiFluidSystem`, `ChFsiInterface`)
2. SPH concrete layer (`ChFsiSystemSPH`, `ChFsiFluidSystemSPH`, `ChFsiInterfaceSPH`)
3. Problem-builder layer (`ChFsiProblemSPH` + Cartesian/Cylindrical/Wavetank specializations)
4. Vehicle terrain integration (`CRMTerrain`)

Most users should start from **layer 3** or **layer 4**.

Primary Classes
---------------

### 1) Generic FSI layer (solver-independent)

- `chrono::fsi::ChFsiSystem`:
  Owns one multibody system and one fluid system, advances them, and performs CFD&harr;MBD data exchange.
- `chrono::fsi::ChFsiFluidSystem`:
  Abstract base for fluid-side solvers that participate in FSI.
- `chrono::fsi::ChFsiInterface`:
  Abstract base for state/force exchange between multibody and fluid systems.
- `chrono::fsi::ChFsiInterfaceGeneric`:
  Generic, buffer-copy interface implementation.

### 2) SPH FSI layer (concrete implementation)

- `chrono::fsi::sph::ChFsiFluidSystemSPH`:
  SPH solver with CFD mode (`SetCfdSPH`) and CRM mode (`SetElasticSPH`), BCE generation helpers, and SPH parameter API.
- `chrono::fsi::sph::ChFsiSystemSPH`:
  SPH concrete FSI system. By default uses `ChFsiInterfaceSPH` (custom, direct SPH data-manager coupling).
- `chrono::fsi::sph::ChFsiInterfaceSPH`:
  SPH-optimized interface (fewer copy hops than generic interface).

### 3) SPH problem-builder layer (recommended user API)

- `chrono::fsi::sph::ChFsiProblemSPH`:
  High-level setup API for particles, BCE markers, rigid bodies, FEA meshes, domains, initialization, stepping.
- `chrono::fsi::sph::ChFsiProblemCartesian`:
  Cartesian-grid setup helper (`Construct` from box, heightmap, or marker files).
- `chrono::fsi::sph::ChFsiProblemCylindrical`:
  Cylindrical-grid setup helper (`Construct` for annulus/tank-like domains).
- `chrono::fsi::sph::ChFsiProblemWavetank`:
  Wavetank builder with wavemaker body/motor and optional beach profile.
- `chrono::fsi::sph::ChFsiProblemSPH::ParticlePropertiesCallback`:
  Hook for custom initial particle pressure/density/viscosity/velocity.
- `chrono::fsi::sph::DepthPressurePropertiesCallback`:
  Ready-made hydrostatic initialization callback.

### 4) Vehicle integration layer

- `chrono::vehicle::CRMTerrain`:
  Dual role class:
  - `ChTerrain` (vehicle terrain API)
  - `ChFsiProblemCartesian` (SPH CRM builder)
  Adds vehicle-centric features such as moving patch and vehicle-advance callback registration.

### 5) Optional utilities

- `chrono::fsi::sph::ChSphVisualizationVSG`:
  VSG plugin to visualize SPH/BCE markers.
- `chrono::fsi::sph::ChFsiSplashsurfSPH`:
  Optional particle-to-surface reconstruction utility (splashsurf).

Inheritance Map
---------------

### Main API hierarchy

```text
chrono::fsi::ChFsiFluidSystem
  └─ chrono::fsi::sph::ChFsiFluidSystemSPH

chrono::fsi::ChFsiSystem
  └─ chrono::fsi::sph::ChFsiSystemSPH

chrono::fsi::ChFsiInterface
  ├─ chrono::fsi::ChFsiInterfaceGeneric
  └─ chrono::fsi::sph::ChFsiInterfaceSPH

chrono::fsi::sph::ChFsiProblemSPH
  ├─ chrono::fsi::sph::ChFsiProblemCartesian
  │   └─ chrono::fsi::sph::ChFsiProblemWavetank
  └─ chrono::fsi::sph::ChFsiProblemCylindrical

chrono::fsi::sph::ChFsiProblemSPH::ParticlePropertiesCallback
  └─ chrono::fsi::sph::DepthPressurePropertiesCallback

chrono::fsi::sph::ChFsiProblemWavetank::Profile
  ├─ chrono::fsi::sph::WaveTankRampBeach
  └─ chrono::fsi::sph::WaveTankParabolicBeach

chrono::vehicle::ChTerrain
  └─ chrono::vehicle::CRMTerrain
       + also inherits chrono::fsi::sph::ChFsiProblemCartesian
```

### SPH internal physics hierarchy

```text
chrono::fsi::sph::SphForce
  ├─ chrono::fsi::sph::SphForceWCSPH
  └─ chrono::fsi::sph::SphForceISPH
```

SPH internal composition (not inheritance):

```text
ChFsiFluidSystemSPH
  ├─ FsiDataManager
  ├─ SphBceManager
  └─ SphFluidDynamics
       ├─ SphCollisionSystem
       └─ SphForce (WCSPH or ISPH depending on IntegrationScheme)
```

Interface Clarity
-----------------------

### ChFsiInterface vs. ChFsiInterfaceGeneric vs. ChFsiInterfaceSPH

- `chrono::fsi::ChFsiInterface` is an **abstract base class**.
  It provides shared utilities:
  - registration of FSI solids (`AddFsiBody`, `AddFsiMesh1D`, `AddFsiMesh2D`)
  - extraction of solid states from MBS (`StoreSolidStates`)
  - loading fluid forces back to solids (`LoadSolidForces`)
  - buffer size/consistency checks
- `chrono::fsi::ChFsiInterfaceGeneric` is a **concrete implementation** of that base interface.
  It exchanges data through intermediate CPU buffers and calls:
  - `ChFsiFluidSystem::LoadSolidStates(...)`
  - `ChFsiFluidSystem::StoreSolidForces(...)`
- `chrono::fsi::sph::ChFsiInterfaceSPH` is the **SPH-specific optimized implementation**.
  It bypasses the generic buffer-copy route and writes/reads directly to/from SPH data-manager host/device structures.

For SPH, `ChFsiSystemSPH` uses `ChFsiInterfaceSPH` by default.
`ChFsiInterfaceGeneric` is used only if `ChFsiSystemSPH(..., use_generic_interface=true)` is requested.

SPH Particle Initialization control when using ChFsiProblemSPH class
---------------------------------

### ParticlePropertiesCallback

- Declared inside `ChFsiProblemSPH`.
- Purpose: define initial per-particle properties before particles are inserted into `ChFsiFluidSystemSPH`.
- Fields you control:
  - `p0` (pressure)
  - `rho0` (density)
  - `mu0` (viscosity)
  - `v0` (velocity)
  - `pre_pressure_scale0` (CRM consolidation-pressure scale used by MCC rheology)
- Important rule: if you override `set(...)`, set **all** fields.

Execution timing:

- Called once per particle during `ChFsiProblemSPH::Initialize()`.
- Not called during time stepping.

### DepthPressurePropertiesCallback

- A ready-made callback derived from `ParticlePropertiesCallback`.
- It sets hydrostatic initial pressure using particle depth:
  - `p0 = rho * g * (zero_height - z)`
  - `rho0 = rho + p0/c^2`
  - `mu0 = default fluid/system viscosity`
  - `v0 = 0`
- Use this when you want a hydrostatic initial state (common in tank/drop/wave-type initializations).

### Do you use these with ChFsiFluidSystemSPH + ChFsiSystemSPH directly?

- Usually **no**, because these callbacks belong to the `ChFsiProblemSPH` setup API.
- With direct low-level setup, you set properties yourself when calling `AddSPHParticle(...)`.
- For simple global hydrostatic pressure setup in direct mode, `SetInitPressure(...)` is another option.

Using ChSphVisualizationVSG with VSG
--------------------------------------

`ChSphVisualizationVSG` is a **VSG plugin**, not a standalone visual system.
You use it in addition to a regular VSG visual system.

### Standard VSG (vsg3d::ChVisualSystemVSG)

Use this sequence:

1. Create `ChSphVisualizationVSG` from `ChFsiSystemSPH*` or `ChFsiFluidSystemSPH*`.
2. Configure marker visibility/color callbacks.
3. Create a regular `vsg3d::ChVisualSystemVSG`.
4. Attach the plugin with `AttachPlugin(visFSI)`.
5. Attach your simulation system (`AttachSystem(&sysMBS)` for standard FSI demos).
6. Call `Initialize()`.
7. In the loop, call only `vis->Run()` / `vis->Render()` on the regular visual system.

### Vehicle VSG visualizers

With `ChWheeledVehicleVisualSystemVSG` / `ChTrackedVehicleVisualSystemVSG`, usage is the same plugin pattern:

- `AttachVehicle(...)` as usual
- `AttachPlugin(visFSI)`
- `Initialize()`
- in the simulation loop, still call the host visual system `vis->Run()` and `vis->Render()`
  (and typically `vis->Synchronize(...)` / `vis->Advance(...)` in vehicle demos)

No separate render loop is needed for `ChSphVisualizationVSG` itself; plugin hooks (`OnAttach`, `OnInitialize`,
`OnRender`) are invoked automatically by the hosting VSG visual system during that host system's normal render loop.

Note: SPH run-time visualization copies particle data each render call, so it can be expensive for large particle
counts.

When To Use Which Class
-----------------------

### Use ChFsiProblemCartesian when

- you want the simplest SPH-FSI setup path
- your domain is box-like, heightmap-based, or loaded from marker files
- you still need rigid bodies and/or FEA coupling without managing raw SPH arrays

This is the most common entry point in `src/demos/fsi/sph/`.

### Use ChFsiProblemWavetank when

- you need a wave tank with piston/flap wavemaker
- you want built-in beach profile support and optional lateral periodic BC

### Use ChFsiProblemCylindrical when

- your natural geometry is cylindrical/annular
- you want cylindrical discretization helpers

### Use direct ChFsiFluidSystemSPH + ChFsiSystemSPH when

- you need full control over particle creation (`AddSPHParticle`)
- you need explicit/manual BCE generation and placement
- you are doing method development or low-level debugging

### Use CRMTerrain when

- your application is a Chrono vehicle terramechanics problem (CRM soil)
- you need vehicle-facing terrain API plus SPH CRM
- you need moving patch behavior for long traverses

Demo-Grounded Usage Patterns
----------------------------

### Pattern A: High-level problem builders

Seen in:

- `src/demos/fsi/sph/demo_FSI-SPH_Compressibility.cpp` (Cartesian, CFD)
- `src/demos/fsi/sph/demo_FSI-SPH_ObjectDrop.cpp` (Cartesian, CFD)
- `src/demos/fsi/sph/demo_FSI-SPH_CylindricalTank.cpp` (Cylindrical, CFD)
- `src/demos/fsi/sph/demo_FSI-SPH_WaveTank.cpp` (Wavetank, CFD)
- `src/demos/fsi/sph/demo_FSI-SPH_FEAdirections.cpp` (Cartesian + FEA mesh + node directions)

Best when you want fast setup with fewer low-level details.

### Pattern B: Direct SPH+FSI systems

Seen in:

- `src/demos/fsi/sph/demo_FSI-SPH_DamBreak.cpp` (manual SPH/BCE setup)
- `src/demos/fsi/sph/demo_FSI-SPH_AngleRepose.cpp` (manual CRM setup)
- `src/demos/fsi/sph/demo_FSI-SPH_BCE.cpp` (BCE generation patterns)
- `src/demos/vehicle/fsi/demo_VEH_FSI_FloatingBlock.cpp` (vehicle-side FSI example)

Best when you need explicit control of particle/boundary construction.

### Pattern C: Vehicle CRM integration

Seen in:

- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_WheeledVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_TrackedVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_MovingPatch.cpp`
- `src/demos/vehicle/test_rigs/demo_VEH_TireTestRig_CRM.cpp`

Best when the main object is a vehicle and terrain is CRM-SPH.

Recommended Starting Workflow (User View)
-----------------------------------------

For new users, this is usually the best sequence:

1. Create Chrono MBS system.
2. Create `ChFsiProblemCartesian`.
3. Set gravity and time steps.
4. Choose physics mode:
   - `SetCfdSPH(...)` for fluids
   - `SetElasticSPH(...)` for CRM soil
5. Set `SPHParameters`.
6. Build particle/BCE domain with `Construct(...)`.
7. Add interacting solids:
   - `AddRigidBody(...)` for rigid objects
   - `AddFeaMesh(...)` for flexible structures
8. `Initialize()`.
9. Simulation loop with `DoStepDynamics(step)` (and normal MBS synchronize/driver logic around it).

For vehicle terramechanics, start from `CRMTerrain` instead of directly using `ChFsiProblemCartesian`.
