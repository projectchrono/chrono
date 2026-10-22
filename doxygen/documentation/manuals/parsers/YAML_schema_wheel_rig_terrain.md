YAML schema for Chrono::Vehicle single-wheel test rig terrain specification {#YAML_schema_wheel_rig_terrain}
========================================

A Chrono YAML wheel test rig terrain file defines the terrain of a
[wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation): the terrain type, the dimensions of the terrain
patch, and the terrain parameters. It is independent of the rig mechanism and of the test scenario. For CRM terrain,
it plays the role of the fluid specification in an FSI simulation, with the soil properties specified here and the
SPH solver in a separate file. It consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The terrain `type`.
- [required] The `patch` object, which specifies the dimensions of the terrain patch.
- [optional] The terrain parameter object matching the terrain type (`rigid`, `scm`, `granular`, or `crm`).

## Terrain type and patch

| `type` | Terrain | Parameter object |
|--------|---------|------------------|
| `RIGID` | Rigid terrain patch | `rigid` |
| `SCM` | Soil Contact Model (SCM) deformable terrain | `scm` |
| `GRANULAR` | Granular terrain, made of rigid spheres | `granular` |
| `CRM` | Continuum Representation Model (CRM) deformable terrain, using SPH | `crm` (required) |

Only the parameter object matching the terrain type is read; the others are ignored. Parameters that are not
specified take their default values.

The `patch` object specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `length` | Patch length (m), along the direction of travel | double | -- | Yes | -- |
| `width` | Patch width (m) | double | -- | Yes | -- |
| `depth` | Patch depth (m), `GRANULAR` and `CRM` terrain only | double | -- | No | 0 |

The terrain patch is placed by the rig relative to the wheel: its surface is 0.1 m below the wheel (with a suspension,
in the loaded configuration) and it starts 2 wheel radii behind the initial wheel location, extending along the positive X axis. With a test longitudinal speed,
the patch length therefore determines the duration of a test (see the `stop_at_patch_end` simulation option). Granular
terrain uses a moving patch, which is relocated ahead of the wheel as the wheel advances.

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
| `Young_modulus` | Young's modulus (Pa) | double | -- | No | 1e7 |

The number of particle layers is set by the patch depth. Note that a granular terrain consists of many sphere bodies
and practical run times require a Chrono::Multicore system, which is not yet supported by the wheel test rig YAML
parser.

## CRM terrain

The `crm` key (required for CRM terrain) specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `soil_properties` | Soil properties | object | -- | No | -- |
| `solver` | SPH solver specification file | string | -- | Yes | -- |
| `visualization` | Rendering of SPH particles and BCE markers | object | -- | No | -- |

The `soil_properties` object has the same keys and defaults as `model.soil_properties` in the
[FSI-SPH model schema](@ref YAML_schema_fsisph_model), including the parameters of the MCC rheology model.

The `solver` entry must specify the path (relative to the location of this YAML file) to the YAML file with an SPH
solver specification, which must follow the [FSI-SPH solver schema](@ref YAML_schema_fsisph_solver). Values not
specified there take the defaults of that schema. The SPH particle spacing is set by `kernel.initial_spacing` and the
SPH (CFD) step size by `integration.time_step`. The multibody step size is set in the MBS solver file of the rig
model, and the FSI coupling step by `simulation.time_step` in the simulation file.

The `visualization` object has the same settings as the `visualization` object of the
[FSI-SPH simulation schema](@ref YAML_schema_fsisph_simulation). The most commonly used ones are:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `sph_markers` | Render SPH particles | boolean | -- | No | `true` |
| `bndry_bce_markers` | Render boundary BCE markers | boolean | -- | No | `false` |
| `rigid_bce_markers` | Render BCE markers on rigid solids | boolean | -- | No | `true` |

The SPH domain spans the entire terrain patch. An active domain attached to the wheel, which restricts the SPH
computation to the vicinity of the wheel, can be specified through `coupling.crm.active_domain` in the simulation file.

## Examples

Below is an example of an SCM terrain specification:

\include data/yaml/vehicle/wheel_rig/terrain_scm.yaml

Below is an example of a CRM terrain specification:

\include data/yaml/vehicle/wheel_rig/terrain_crm.yaml

## YAML schema

The YAML wheel test rig terrain specification file must follow the ``data/yaml/schema/wheel_rig_terrain.schema.yaml`` provided in the Chrono data directory:

\include data/yaml/schema/wheel_rig_terrain.schema.yaml
