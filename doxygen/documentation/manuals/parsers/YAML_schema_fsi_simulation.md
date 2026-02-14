YAML schema for Chrono::FSI simulation specification {#YAML_schema_fsi_simulation}
========================================

A Chrono YAML FSI problem specification file defines the multibody and fluid problems and co-simulation parameters required to run a Chrono::FSI co-simulation.
It consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The `mbs` object which defines YAML specification of the MBS simulation.
- [required] The `fluid` object which defines YAML specification of the fluid specification.
- [required] The `fsi` object which defines YAML specification of coupled FSI problem.
- [required] The `simulation` object which defines the co-simulation metastep and the simulation duration.
- [optional] The `visualization` object which enables run-time visualization and defines rendering settings.

## FSI simulation specification

An FSI problem couples a Chrono MBS model and simulation to a fluid solver. Any fluid solver that implements the Chrono::FSI API can be used.

The multibody and fluid simulations are specified by referring to the corresponding YAML specification files. These file names must include the path to the files, relative to the location of this Chrono::FSI YAML specification.

#### Multibody and fluid system specification

The `mbs` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an MBS simulation specification (which must follow the [MBS simulation schema](@ref YAML_schema_mbs_simulation)).

The `fluid` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with a fluid simulation specification (which must follow the [SPH simulation schema](@ref YAML_schema_fsisph_simulation) or the [TDPF simulation schema](@ref YAML_schema_fsitdpf_simulation)).

#### Specification of coupled FSI problem

** **TODO** **

#### Co-simulation options

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `time_step` | Co-simulation (meta) step | double | -- | Yes | -- |
| `end_time` | Total simulation time in seconds | double | -- | No | -1 for infinite simulation |
| `gravity` | Gravitational acceleration vector [x, y, z] | array[3] | -- | No | [0, 0, -9.8] |


#### Visualization options

If the `visualization` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Type of visualization shapes | enum | `NONE`,`PRIMITIVES`,`MODEL_FILE`,`COLLISION`  | No | `NONE` |
| `render_fps` | Rendering frequency (FPS or Hz) | double | -- | No | 120 |
| `enable_shadows` | Turn on shadow rendering | boolean | -- | No | `true` |
| `camera` | Camera settings | object | -- | No | -- |

The `camera` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `vertical` | Vertical direction (camera "up") | enum | `Y`,`Z`  | No | `Z` |
| `location` | Camera initial location | array[3] | -- | No | [0,-1,0] |
| `target` | Camera initial target ("look-at" point) | array[3] | -- | No | [0,0,0]  |

## Example

Below is an example of an FSI simulation configuration:

\include data/yaml/fsi/objectdrop/fsi_objectdrop.yaml

## YAML schema

The YAML FSI simulation specification file must follow the ``data/yaml/schema/fsi_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsi_simulation.schema.yaml

