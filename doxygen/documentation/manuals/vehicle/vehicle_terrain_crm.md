CRM deformable terrain (SPH) for Chrono::Vehicle simulations {#vehicle_terrain_crm_api_}
======================================================================

\tableofcontents

This page is a compact user guide to [CRMTerrain](@ref chrono::vehicle::CRMTerrain), with direct comparison to
[SCMTerrain](@ref chrono::vehicle::SCMTerrain).

Quick choice
------------

- Use **SCM** if you want fast deformable-soil simulation, simple terramechanics parameterization, and minimal setup.
- Use **CRM** if you need SPH continuum soil flow/deformation and are willing to pay a slightly higher computational cost.

Difference in Physics
---------------------

- **SCM terrain** is a deformable height-field model: soil deformation is represented on a grid through node deflection
  along the terrain normal, with Bekker/Janosi/Mohr-style terramechanics parameters.
- **CRM terrain** is an SPH continuum representation of deformable soil: the terrain is represented by particles and
  uses CRM rheology (e.g., `MU_OF_I` or `MCC`).

From a user perspective: SCM is the lighter-weight choice, while CRM is the higher-fidelity choice.

API differences that matter most
--------------------------------

1. Class role:
   - `SCMTerrain` is a `ChTerrain` implementation.
   - `CRMTerrain` is both `ChTerrain` and `ChFsiProblemCartesian` (an SPH-FSI problem builder).
2. Terrain construction:
   - SCM: call `Initialize(...)` (flat grid, heightmap, or mesh).
   - CRM: call `Construct(...)` (box, heightmap, or particle/BCE files), then `Initialize()`.
3. Soil model parameters:
   - SCM: `SetSoilParameters(...)` or `RegisterSoilParametersCallback(...)`.
   - CRM: `SetElasticSPH(...)` and `SetSPHParameters(...)` (SPH material + solver settings).
4. Vehicle/solid coupling:
   - SCM: vehicle interaction comes through regular Chrono contact shapes (no explicit FSI registration).
   - CRM: you must explicitly add interacting solids to FSI (`AddRigidBody(...)`, `AddFeaMesh(...)`), so BCE markers are
     generated.
5. Time stepping ownership:
   - SCM: usually the vehicle/system step advances terrain effects through contact.
   - CRM: call `terrain.Advance(step)` (or `DoStepDynamics(step)`) to advance coupled SPH+MBS.
6. Moving-domain features:
   - SCM: `AddActiveDomain(...)` reduces ray-casting work.
   - CRM: `SetActiveDomain(...)` limits active SPH particles and `ConstructMovingPatch(...)` can relocate soil forward.
7. Terrain query semantics:
   - SCM: `GetHeight/GetNormal/GetCoefficientFriction` are meaningful.
   - CRM: these currently return placeholder values (`0`, world-up normal, `0` friction), so CRM is not intended for
     height/normal/friction-based tire models.

Minimal API patterns
--------------------

SCM pattern:

~~~{.cpp}
SCMTerrain terrain(sys);
terrain.SetSoilParameters(...);                   // or RegisterSoilParametersCallback(...)
terrain.AddActiveDomain(spindle, VNULL, size);   // optional
terrain.Initialize(sizeX, sizeY, delta);         // or heightmap/mesh initialize
~~~

CRM pattern:

~~~{.cpp}
CRMTerrain terrain(*sys, spacing);
terrain.SetGravitationalAcceleration({0, 0, -9.81});
terrain.SetStepSizeCFD(step_size);
terrain.RegisterVehicle(vehicle.get());           // recommended in vehicle demos

terrain.SetElasticSPH(mat_props);
terrain.SetSPHParameters(sph_params);

terrain.AddRigidBody(spindle, geometry, false);  // or AddFeaMesh(...)
terrain.SetActiveDomain(active_box_dim);         // optional

terrain.Construct(...);                          // box / heightmap / marker files
terrain.Initialize();
~~~

Simulation-loop difference (important)
--------------------------------------

With SCM, your normal vehicle step drives the contact-based terrain response:

~~~{.cpp}
driver.Synchronize(time);
terrain.Synchronize(time);      // optional API call (keeps subsystem sequence uniform)
vehicle.Synchronize(time, in, terrain);
driver.Advance(step);
vehicle.Advance(step);          // Chrono contact + SCM loader work happens here
~~~

With CRM, the coupled SPH-FSI step is explicit:

~~~{.cpp}
driver.Synchronize(time);
terrain.Synchronize(time);      // moving-patch / active-domain updates
vehicle->Synchronize(time, in, terrain);
driver.Advance(step);
terrain.Advance(step);          // coupled SPH + MBS step
~~~

If you called `RegisterVehicle(...)`, the vehicle dynamics are advanced inside CRM's FSI step (see CRM vehicle demos).

VSG visualization plugins
-------------------------

- SCM visualization plugin: [ChScmVisualizationVSG](@ref chrono::vehicle::ChScmVisualizationVSG)
- CRM visualization plugin: [ChSphVisualizationVSG](@ref chrono::fsi::sph::ChSphVisualizationVSG)

Both are VSG plugins attached to a host visual system with `AttachPlugin(...)`.  
You render only through the host visual system (`Run()`/`Render()`); there is no separate render loop for the plugin.

Notes:
- CRM run-time visualization can be expensive at large particle counts because SPH/BCE marker data is copied each
  render call.
- `SCMTerrain` requires a collision system to be associated with the Chrono system before terrain creation.

Demo map (recommended starting points)
--------------------------------------

CRM terrain demos:
- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_WheeledVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_TrackedVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_CRMTerrain_MovingPatch.cpp`
- `src/demos/vehicle/test_rigs/demo_VEH_TireTestRig_CRM.cpp`

SCM terrain demos:
- `src/demos/vehicle/terrain/demo_VEH_SCMTerrain_WheeledVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_SCMTerrain_TrackedVehicle.cpp`
- `src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp`
- `src/demos/vehicle/terrain/demo_VEH_SCMTerrain_FEATire.cpp`
- `src/demos/vehicle/terrain/demo_VEH_RigidSCMTerrain_MixedPatches.cpp`
