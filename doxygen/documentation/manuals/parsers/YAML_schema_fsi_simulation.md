YAML schema for Chrono::FSI simulation specification {#YAML_schema_fsi_simulation}
========================================

A Chrono YAML FSI problem specification file defines the multibody and fluid problems and co-simulation parameters required to run a Chrono::FSI co-simulation.
It consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The `mbs` object which defines YAML specification of the MBS simulation.
- [required] The `fluid` object which defines YAML specification of the fluid specification.
- [required] The `fsi` object which defines YAML specification of coupled FSI problem.
- [required] The `simulation` object which defines the co-simulation metastep and the simulation duration.
- [optional] The `output` object which specifies output options.
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

The `fsi` entry (required) describes the coupling itself: which bodies of the multibody model interact with the fluid,
and what geometry represents them on the fluid side.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Name of the coupled model | string | -- | No | empty string |
| `angle_degrees` | Whether angles are specified in degrees (true) or radians (false) | boolean | -- | No | `true` |
| `data_path` | Location of the data files referenced in this specification | object | -- | No | absolute paths |
| `fsi_bodies` | List of rigid bodies that interact with the fluid | array | -- | No | no FSI bodies |

The `data_path` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Mode for data file location | enum | `ABSOLUTE`,`RELATIVE` | Yes | `ABSOLUTE` |
| `root` | Root of data files, relative to the location of this file | string | -- | No | `.` |

Each entry of `fsi_bodies` specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Name of the body in the multibody model | string | -- | Yes | -- |
| `shapes` | Geometry used to generate the fluid-solid interface markers | array | -- | Yes | -- |

The body `name` is looked up in the MBS model; if no body with that name exists, a warning is issued and the entry is
ignored. Note that the lookup matches all instances of that body name, so a single `fsi_bodies` entry covers every
instance of a multiply-instantiated MBS model.

Each entry of `shapes` specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Geometric shape type | enum | `SPHERE`,`BOX`,`CYLINDER`,`HULL`,`MESH` | Yes | -- |
| `location` | Shape location relative to the body reference frame | array[3] | -- | No | [0, 0, 0] |
| `orientation` | Shape orientation relative to the body reference frame | array[3] or array[4] | -- | No | identity rotation |
| `dimensions` | Length, width, and height | array[3] | -- | Yes, for `BOX` | -- |
| `radius` | Shape radius | double | -- | Yes, for `SPHERE` and `CYLINDER` | -- |
| `length` | Cylinder length | double | -- | Yes, for `CYLINDER` | -- |
| `axis` | Cylinder axis direction, in the body reference frame | array[3] | -- | Yes, for `CYLINDER` | -- |
| `filename` | 3D data file name | string | -- | Yes, for `MESH` and `HULL` | -- |
| `scale` | Scale factor | double | -- | No | 1.0 |

#### Co-simulation options

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `time_step` | Co-simulation (meta) step | double | -- | Yes | -- |
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

If the `visualization` key is present, run-time visualization of the coupled problem is enabled.
It must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `render_fps` | Rendering frequency (FPS or Hz) | double | -- | No | 120 |
| `enable_shadows` | Turn on shadow rendering | boolean | -- | No | `true` |
| `camera` | Camera settings | object | -- | No | -- |
| `output` | Image output settings | object | -- | No | -- |

There is no `type` entry at this level: the mere presence of the `visualization` key enables rendering.
Body visualization shapes are controlled by the `visualization` entry of the referenced
[MBS simulation file](@ref YAML_schema_mbs_simulation), and fluid-phase rendering by the `visualization` entry of the
referenced fluid simulation file
([SPH](@ref YAML_schema_fsisph_simulation) or [TDPF](@ref YAML_schema_fsitdpf_simulation)).

The `camera` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `vertical` | Vertical direction (camera "up") | enum | `Y`,`Z`  | No | `Z` |
| `location` | Camera initial location | array[3] | -- | No | [0,-1,0] |
| `target` | Camera initial target ("look-at" point) | array[3] | -- | No | [0,0,0]  |

The `output` key nested under `visualization`, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `save_images` | Whether to save rendered frames as image files | boolean | -- | No | `false` |
| `image_type` | Image file type, specified through the file extension | string | -- | No | `bmp` |

## Example

Below is an example of an FSI simulation configuration:

\include data/yaml/fsi/cylinder_drop/fsi_cylinder_drop.yaml

## YAML schema

The YAML FSI simulation specification file must follow the ``data/yaml/schema/fsi_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsi_simulation.schema.yaml

