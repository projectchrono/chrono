YAML schema for Chrono::Vehicle single-wheel test rig terrain specification {#YAML_schema_wheel_rig_terrain}
========================================

The terrain of a [wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation) is specified in a separate file,
independent of the rig mechanism and of the test scenario. Depending on the terrain type, this file is either:
- a wheel test rig terrain file, following the schema described on this page, for RIGID, SCM, or GRANULAR terrain; or
- a Chrono::FSI-SPH simulation file for CRM terrain (see [CRM terrain](@ref YAML_wheel_rig_terrain_crm) below), which uses
  exactly the same SPH simulation, model, and solver schemas as an FSI simulation.

A wheel test rig terrain file consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The terrain `type`.
- [required] The `patch` object, which specifies the dimensions of the terrain patch.
- [optional] The terrain parameter object matching the terrain type (`rigid`, `scm`, or `granular`).

## Terrain type and patch

| `type` | Terrain | Parameter object |
|--------|---------|------------------|
| `RIGID` | Rigid terrain patch | `rigid` |
| `SCM` | Soil Contact Model (SCM) deformable terrain | `scm` |
| `GRANULAR` | Granular terrain, made of rigid spheres | `granular` |

Only the parameter object matching the terrain type is read; the others are ignored. Parameters that are not
specified take their default values.

The `patch` object specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `length` | Patch length (m), along the direction of travel | double | -- | Yes | -- |
| `width` | Patch width (m) | double | -- | Yes | -- |
| `depth` | Patch depth (m), `GRANULAR` terrain only | double | -- | No | 0 |

For all terrain types (including CRM), the terrain patch is placed by the rig relative to the wheel: its surface is
0.1 m below the wheel (with a suspension, in the loaded configuration) and it starts 2 wheel radii behind the initial
wheel location, extending along the positive X axis. With a test longitudinal speed, the patch length therefore
determines the duration of a test (see the `stop_at_patch_end` simulation option). Granular terrain uses a moving patch,
which is relocated ahead of the wheel as the wheel advances.

## Rigid terrain

The `rigid` key, if present, specifies the contact material of the terrain patch:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `friction` | Coefficient of friction | double | -- | No | 0.8 |
| `restitution` | Coefficient of restitution | double | -- | No | 0 |
| `Young_modulus` | Young's modulus (Pa), used with SMC contact | double | -- | No | 2e7 |

## SCM terrain

The `scm` key, if present, specifies the soil parameters of the SCM terrain:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `Bekker_Kphi` | Frictional modulus in the Bekker model | double | -- | No | 2e6 |
| `Bekker_Kc` | Cohesive modulus in the Bekker model | double | -- | No | 0 |
| `Bekker_n` | Exponent of sinkage in the Bekker model (usually 0.6 to 1.8) | double | -- | No | 1.1 |
| `Mohr_cohesion` | Cohesion for shear failure (Pa) | double | -- | No | 50 |
| `Mohr_friction` | Friction angle for shear failure, always in degrees | double | -- | No | 20 |
| `Janosi_shear` | Shear parameter J in the Janosi-Hanamoto formula (m) | double | -- | No | 0.01 |
| `grid_spacing` | SCM grid spacing (m) | double | -- | No | 0.125 |
| `visualize_mesh` | Whether to create the SCM visualization mesh | boolean | -- | No | `true` |

The rig attaches an SCM active domain to the rig chassis, so that terrain deformation is only computed near the
wheel.
The visualization mesh can be disabled for a run without run-time visualization.

## Granular terrain

The `granular` key, if present, specifies the particle parameters of the granular terrain:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `radius` | Particle radius (m) | double | -- | No | 0.02 |
| `density` | Particle density (kg/m<sup>3</sup>) | double | -- | No | 2000 |
| `friction` | Coefficient of friction | double | -- | No | 0.9 |
| `cohesion` | Inter-particle cohesion pressure (Pa) | double | -- | No | 0 |
| `Young_modulus` | Young's modulus (Pa), used with SMC contact | double | -- | No | 1e7 |

The number of particle layers is set by the patch depth. Note that a granular terrain consists of many sphere bodies
and practical run times require a Chrono::Multicore system, which is not yet supported by the wheel test rig YAML
parser.

## CRM terrain {#YAML_wheel_rig_terrain_crm}

CRM (Continuum Representation Model) terrain is specified through a Chrono::FSI-SPH simulation file (`type: SPH`),
following the [FSI-SPH simulation schema](@ref YAML_schema_fsisph_simulation), which references an SPH model file
([FSI-SPH model schema](@ref YAML_schema_fsisph_model)) and an SPH solver file
([FSI-SPH solver schema](@ref YAML_schema_fsisph_solver)). The same files can therefore be used in FSI and wheel test
rig simulations. The SPH model must use CRM physics (`physics_problem: CRM`) and a Cartesian box fluid domain
(`geometry_type: CARTESIAN`, no wave tank).

The parts of these specifications that apply to a test rig terrain are used as follows:

| Specification | Use for a test rig terrain |
|---------------|----------------------------|
| model `soil_properties` | CRM soil properties, including the parameters of the MCC rheology model |
| model `fluid_domain.dimensions` | Patch length, width, and depth |
| model `fluid_domain.box_walls` | Patch sides with boundary walls; if no walls are specified, all sides but the top |
| model `initial_states.depth_based_pressure` | Initialize particle pressures from their depth below the patch surface |
| solver file | SPH solver settings; the particle spacing is set by `kernel.initial_spacing` and the SPH (CFD) step size by `integration.time_step` |
| simulation `visualization` | Rendering of SPH particles and BCE markers |

The following parts are ignored, since the test rig places the patch relative to the wheel and the simulation settings
are specified in the wheel test rig simulation file:

| Specification | Reason |
|---------------|--------|
| model `fluid_domain.box_origin` | The rig places the patch relative to the wheel |
| model `initial_states.zero_height` | Pressures are zero at the patch surface |
| model `initial_states.initial_velocity` | The terrain is initially at rest |
| model `container`, `computational_domain` | The rig defines the patch boundaries and the computational domain |
| simulation `simulation`, `output` | Set in the wheel test rig simulation file |

A note is printed for ignored model settings that are present. Note that the multibody step size is set in the MBS
solver file of the rig model, and the FSI coupling step by `simulation.time_step` in the wheel test rig simulation file.

The SPH domain spans the entire terrain patch. An active domain attached to the wheel, which restricts the SPH
computation to the vicinity of the wheel, can be specified through `coupling.crm.active_domain` in the simulation file.

## Examples

Below is an example of an SCM terrain specification:

\include data/yaml/vehicle/wheel_rig/terrain_scm.yaml

Below is an example of a CRM terrain specification (an SPH simulation file):

\include data/yaml/vehicle/wheel_rig/terrain_crm.yaml

with the SPH model file it references:

\include data/yaml/vehicle/wheel_rig/terrain_crm_model.yaml

## YAML schema

The YAML wheel test rig terrain specification file must follow the ``data/yaml/schema/wheel_rig_terrain.schema.yaml`` provided in the Chrono data directory:

\include data/yaml/schema/wheel_rig_terrain.schema.yaml
