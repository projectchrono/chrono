YAML schema for Chrono::FSI-TDPF simulation specification {#YAML_schema_fsitdpf_simulation}
=======================================

A Chrono YAML TDPF simulation file defines the setup for a Chrono::FSI-TDPF simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The simulation `type`, which must be `TDPF` here.
- [required] The `model` entry which names the YAML specification of the TDPF problem.
- [required] The `solver` entry which names the YAML specification of the TDPF solver settings.
- [optional] The `output` object which specifies output options from the TDPF simulation.
- [optional] The `visualization` object which specifies TDPF-specific run-time visualization settings.


## FSI-TDPF simulation specification

An FSI-TDPF simulation must specify the TDPF model to be simulated, as well as optional output and TDPF-specific run-time visualization settings.

#### Model and solver specification

The `model` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with a TDPF model specification (which must follow the [TDPF model schema](@ref YAML_schema_fsitdpf_model)).

The `solver` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with a TDPF solver specification (which must follow the [TDPF solver schema](@ref YAML_schema_fsitdpf_solver)).

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

If the `visualization` key is present, run-time visualization of the water surface is enabled.
The following TDPF-specific properties can be set:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `update_fps` | Frequency (FPS or Hz) at which the wave mesh is updated | double | -- | No | 30 |
| `color_map` | Colormap-based coloring of the wave mesh | object | -- | No | no coloring |

The `color_map` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Wave mesh quantity used for coloring | enum | `NONE`,`HEIGHT`,`VELOCITY` | Yes | -- |
| `map` | Colormap | enum | `BLACK_BODY`,`BLUE`,`BROWN`,<br>`COPPER`,`FAST`,`INFERNO`,<br>`JET`,`KINDLMANN`,`PLASMA`,<br>`RED_BLUE` | No | `FAST` |
| `min` | Lower end of the color data range | double | -- | No | -1 |
| `max` | Upper end of the color data range | double | -- | No | 1 |

Note that camera and general rendering settings for a coupled FSI simulation are specified in the
[FSI simulation file](@ref YAML_schema_fsi_simulation), not here.


## Example

Below is an example of an FSI-TDPF simulation configuration:

\include data/yaml/fsi/sphere_decay/tdpf.yaml


## YAML schema

The YAML TDPF simulation specification file must follow the ``data/yaml/schema/fsitdpf_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsitdpf_simulation.schema.yaml

