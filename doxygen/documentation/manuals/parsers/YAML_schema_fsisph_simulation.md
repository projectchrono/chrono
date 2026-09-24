YAML schema for Chrono::FSI-SPH simulation specification {#YAML_schema_fsisph_simulation}
=======================================

A Chrono YAML SPH simulation file defines the setup for a Chrono::FSI-SPH simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The simulation `type`, which must be `SPH` here.
- [required] The SPH `model` object which defines YAML specification of SPH problem.
- [required] The SPH `solver` object which defines YAML specification of SPH solver algorithms.
- [optional] The `output` object which specifies output options from the SPH simulation.
- [optional] The `visualization` object which specifies SPH-specific run-time visualization settings.

## FSI-SPH simulation specification

An FSI-SPH simulation must specify the SPH model to be simulated, SPH solver settings, as well as optional output and SPH-specific run-time visualization settings.

#### Model and solver specification

The `model` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an SPH model specification (which must follow the [SPH model schema](@ref YAML_schema_fsisph_model)).

The `solver` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an SPH solver specification (which must follow the [SPH solver schema](@ref YAML_schema_fsisph_solver)).


#### Output options

If the `output` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `format` | Output DB format | enum | `NONE`,`ASCII`,`HDF5`  | Yes | -- |
| `mode` | Output mode (one file per output frame, or a single time-series file) | enum | `FRAMES`,`SERIES`  | No | `FRAMES` |
| `fps` | Output frequency (FPS or Hz) | double | -- | No | 100 |

Note that `format` is required whenever the `output` key is present.
`HDF5` output silently falls back to `NONE` in a build without HDF5 support.

#### Visualization options

If the `visualization` key is present, run-time visualization of the fluid phase is enabled.
The following SPH-specific properties can be set:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `sph_markers` | Render SPH particles | boolean | -- | No | `true` |
| `bndry_bce_markers` | Render boundary BCE markers | boolean | -- | No | `true` |
| `rigid_bce_markers` | Render BCE markers on rigid solids | boolean | -- | No | `true` |
| `flex_bce_markers` | Render BCE markers on flexible solids | boolean | -- | No | `true` |
| `active_boxes` | Render the active domain boxes associated with FSI solids | boolean | -- | No | `false` |
| `color_map` | Colormap-based coloring of SPH particles | object | -- | No | no coloring |
| `visibility` | Marker visibility control through a set of half-spaces | object | -- | No | all markers visible |
| `splashsurf` | Fluid surface reconstruction | object | -- | No | disabled |

Note that camera and general rendering settings for a coupled FSI simulation are specified in the
[FSI simulation file](@ref YAML_schema_fsi_simulation), not here.

##### Particle coloring

The `color_map` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Particle quantity used for coloring | enum | `NONE`,`HEIGHT`,`VELOCITY`,<br>`DENSITY`,`PRESSURE` | Yes | -- |
| `map` | Colormap | enum | `BLACK_BODY`,`BLUE`,`BROWN`,<br>`COPPER`,`FAST`,`INFERNO`,<br>`JET`,`KINDLMANN`,`PLASMA`,<br>`RED_BLUE` | No | `JET` |
| `min` | Lower end of the color data range | double | -- | Yes, unless `type` is `NONE` | -- |
| `max` | Upper end of the color data range | double | -- | Yes, unless `type` is `NONE` | -- |
| `up` | Direction along which height is measured | array[3] | -- | No | [0, 0, 1] |
| `bimodal` | Use a bimodal (two-sided) color scale | boolean | -- | Yes, for `PRESSURE` | -- |

`up` is used only for the `HEIGHT` type and `bimodal` only for the `PRESSURE` type.

##### Marker visibility

The `visibility` key, if present, hides markers based on their position relative to a set of planes:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `planes` | List of planes defining the half-spaces | array | -- | Yes | -- |
| `mode` | Whether a marker must satisfy all half-spaces or any one of them | enum | `ALL`,`ANY` | No | `ALL` |
| `SPH` | Apply the visibility criterion to SPH particles | boolean | -- | No | `true` |
| `BCE` | Apply the visibility criterion to BCE markers | boolean | -- | No | `true` |

Each entry of `planes` specifies the following properties (both required):

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `point` | A point on the plane | array[3] | -- | Yes | -- |
| `normal` | Plane normal direction | array[3] | -- | Yes | -- |

##### Surface reconstruction

The mere presence of the `splashsurf` key enables fluid surface reconstruction.
The following optional properties can be set:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `smoothing_length` | Smoothing length for the SPH kernel, in multiples of the particle radius | double | -- | No | 1.5 |
| `cube_size` | Marching-cubes edge length, in multiples of the particle radius | double | -- | No | 0.5 |
| `surface_threshold` | Iso-surface density threshold, in multiples of the rest density | double | -- | No | 0.6 |


## Example

Below is an example of an FSI-SPH simulation configuration:

\include data/yaml/fsi/sphere_decay/sph.yaml

## YAML schema

The YAML SPH simulation specification file must follow the ``data/yaml/schema/fsisph_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsisph_simulation.schema.yaml
