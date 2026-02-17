YAML schema for Chrono MBS simulation specification {#YAML_schema_mbs_simulation}
=======================================

A Chrono YAML MBS simulation file defines the setup for a Chrono multibody simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The simulation `type`, which must be `MBS` here.
- [required] The MBS `model` object which defines YAML specification of multibody problem.
- [required] The MBS `solver` object which defines YAML specification of the solver algorithms.
- [optional] The `simulation` object specifying how simulation is to be performed.
- [optional] The `output` object which specifies output options from the MBS simulation.
- [optional] The `visualization` object which specifies run-time visualization settings.

## MBS simulation specification

An MBS simulation must specify the multibody model to be simulated, integrator and solver settings, as well as optional output and run-time visualization settings.

#### Model and solver specification

The `model` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an MBS model specification (which must follow the [MBS model schema](@ref YAML_schema_mbs_model)).

The `solver` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an MBS solver specification (which must follow the [MBS solver schema](@ref YAML_schema_mbs_solver)).

#### Simulation options

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `enforce_realtime` | Whether to enforce real-time simulation | boolean | -- | No | false |
| `end_time` | Total simulation time in seconds | double | -- | No | -1 for infinite simulation |
| `gravity` | Gravitational acceleration vector [x, y, z] | array[3] | -- | No | [0, 0, -9.8] |


#### Output options

If the `output` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Output DB type | enum | `NONE`,`ASCII`,`HDF5`  | No | `NONE` |
| `mode` | Output mode | enum | `FRAMES`,`SERIES`  | No | `FRAMES` |
| `fps` | Output frequency (FPS or Hz) | double | -- | No | 30 |

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

Below is an example of an MBS simulation configuration:

\include data/yaml/mbs/mbs.yaml

## YAML schema

The YAML MBS simulation specification file must follow the ``data/yaml/schema/mbs_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/mbs_simulation.schema.yaml
