YAML schema for Chrono::Vehicle simulation specification {#YAML_schema_vehicle_simulation}
========================================

A Chrono YAML vehicle simulation file defines the setup for a Chrono::Vehicle simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The simulation `type`, which must be `VEHICLE` here.
- [required] The vehicle `model` object which defines YAML specification of vehicle system.
- [required] The MBS `solver` object which defines YAML specification of the solver algorithms.
- [optional] The `simulation` object specifying how simulation is to be performed.
- [optional] The `output` object which specifies output options from the MBS simulation.
- [optional] The `visualization` object which specifies run-time visualization settings.

## Vehicle simulation specification

A vehicle simulation must specify the vehicle model to be simulated, integrator and solver settings, as well as optional output and run-time visualization settings.

#### Model and solver specification

The `model` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with a vehicle model specification (which must follow the [vehicle model schema](@ref YAML_schema_vehicle_model)).

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
| `format` | Output DB format | enum | `NONE`,`ASCII`,`HDF5`  | Yes | -- |
| `mode` | Output mode (one file per output frame, or a single time-series file) | enum | `FRAMES`,`SERIES`  | No | `FRAMES` |
| `fps` | Output frequency (FPS or Hz) | double | -- | No | 100 |

Note that `format` is required whenever the `output` key is present.
`HDF5` output silently falls back to `NONE` in a build without HDF5 support.

#### Visualization options

If the `visualization` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Type of body visualization shapes | enum | `NONE`,`PRIMITIVES`,`MODEL_FILE`,`COLLISION`  | No | `NONE` |
| `render_fps` | Rendering frequency (FPS or Hz) | double | -- | No | 120 |
| `enable_shadows` | Turn on shadow rendering | boolean | -- | No | `true` |
| `camera` | Camera settings | object | -- | No | -- |
| `output` | Image output settings | object | -- | No | -- |

The `camera` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `vertical` | Vertical direction (camera "up") | enum | `Y`,`Z`  | No | `Z` |
| `location` | Camera initial location | array[3] | -- | No | [0,-1,0] |
| `target` | Camera initial target ("look-at" point) | array[3] | -- | No | [0,0,0]  |

The `output` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `save_images` | Whether to save rendered frames as image files | boolean | -- | No | `false` |
| `image_type` | Image file type, specified through the file extension | string | -- | No | `bmp` |

Run-time visualization is enabled only if the `visualization` key is present *and* specifies a `type` other than
`NONE`. In other words, omitting the `visualization` key and specifying `type: NONE` are equivalent and both result in
a simulation run with no run-time visualization.

Note that `type` controls the visualization shapes created for plain rigid bodies. The visualization types used for
the vehicle sub-systems (chassis, suspension, steering, wheels, tires, and tracked-vehicle components) are currently
fixed by the parser and cannot be set from YAML.

## Example

Below is an example of a simulation configuration:

\include data/yaml/vehicle/vehicle.yaml

## YAML schema

The YAML vehicle simulation specification file must follow the ``data/yaml/schema/vehicle_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/vehicle_simulation.schema.yaml

