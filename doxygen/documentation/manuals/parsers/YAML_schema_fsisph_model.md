YAML schema for Chrono::FSI-SPH model specification {#YAML_schema_fsisph_model}
=======================================

A Chrono YAML SPH model file defines a fluid system for Chrono::FSI-SPH and contains two main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The `model` object that defines the fluid phase, its material properties, and the problem geometry.

## Model specification

An SPH model defines the fluid (or granular) phase: its material properties, the region initially filled with SPH
particles, the boundaries that confine it, and the extent of the computational domain.
The `model` object supports the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `physics_problem` | Fluid physics type: incompressible fluid (CFD) or homogenized granular dynamics (CRM) | enum | `CFD`,`CRM` | Yes | -- |
| `geometry_type` | Coordinate geometry used for the domain definitions | enum | `CARTESIAN`,`CYLINDRICAL` | Yes | -- |
| `name` | Name of the model | string | -- | No | empty string |
| `angle_degrees` | Whether angles are specified in degrees (true) or radians (false) | boolean | -- | No | `true` |
| `data_path` | Location of the data files referenced in this specification | object | -- | No | absolute paths |
| `fluid_properties` | Physical parameters of the fluid (`CFD` physics) | object | -- | No | see below |
| `soil_properties` | Physical parameters of the soil (`CRM` physics) | object | -- | No | see below |
| `initial_states` | Initialization of the SPH particle states | object | -- | No | zero pressure and velocity |
| `wave_tank` | A wave tank with a wave maker mechanism | object | -- | No | -- |
| `fluid_domain` | Region initially filled with SPH particles | object | -- | Yes, unless `wave_tank` is present | -- |
| `container` | Boundary of the fluid container | object | -- | No | -- |
| `computational_domain` | Extent of the computational domain and its boundary condition types | object | -- | No | unbounded |

A `wave_tank` is an alternative to specifying the domain explicitly: if `wave_tank` is present, then `fluid_domain`,
`container`, and `computational_domain` are all ignored.

The `data_path` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Mode for data file location | enum | `ABSOLUTE`,`RELATIVE` | Yes | `ABSOLUTE` |
| `root` | Root of data files, relative to the location of this file | string | -- | No | `.` |

#### Material properties

For a `CFD` problem, the `fluid_properties` key specifies:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `density` | Fluid density | double | -- | No | 1000.0 |
| `viscosity` | Laminar viscosity | double | -- | No | 0.1 |
| `characteristic_length` | Characteristic length for Reynolds number calculation | double | -- | No | 1.0 |

For a `CRM` problem, the `soil_properties` key specifies:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `density` | Bulk density | double | -- | No | 1000.0 |
| `Young_modulus` | Young's modulus | double | -- | No | 1e6 |
| `Poisson_ratio` | Poisson's ratio | double | -- | No | 0.3 |
| `mu_I0` | Reference inertia number | double | -- | No | 0.03 |
| `mu_fric_s` | Static friction coefficient mu_s in mu = mu(I) | double | -- | No | 0.7 |
| `mu_fric_2` | Limit friction coefficient mu_2 in mu = mu(I) | double | -- | No | 0.7 |
| `average_diam` | Average granular particle diameter | double | -- | No | 0.005 |
| `cohesion_coefficient` | Cohesion coefficient | double | -- | No | 0 |

#### Initial particle states

The `initial_states` key, if present, specifies:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `depth_based_pressure` | Whether to initialize particle pressure from hydrostatic depth | boolean | -- | No | `false` |
| `zero_height` | Height of the zero-pressure level | double | -- | Yes, if `depth_based_pressure` is present | -- |
| `initial_velocity` | Uniform initial velocity applied to all SPH particles | array[3] | -- | No | [0, 0, 0] |

#### Fluid domain and container

The `fluid_domain` key defines the region initially filled with SPH particles.
The `container` key defines a (typically larger) region whose walls confine the fluid.
Both use the same properties, which depend on the model `geometry_type`.

For `CARTESIAN` geometry:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `dimensions` | Dimensions of the box | array[3] | -- | Yes | -- |
| `box_origin` | Origin of the box | array[3] | -- | No | [0, 0, 0] |
| `box_walls` | Boundary walls of the box | object | -- | No | no walls |

For `CYLINDRICAL` geometry:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `inner_radius` | Inner radius of the cylindrical annulus | double | -- | Yes | -- |
| `outer_radius` | Outer radius of the cylindrical annulus | double | -- | Yes | -- |
| `height` | Height of the cylindrical annulus | double | -- | Yes | -- |
| `cyl_origin` | Origin of the cylindrical annulus | array[3] | -- | No | [0, 0, 0] |
| `cyl_walls` | Boundary walls of the annulus | object | -- | No | no walls |

Walls are specified as a pair of flags per direction, ordered [negative side, positive side].
For `box_walls`, all three of the following are required:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `x` | Walls normal to the X direction | array[2] of boolean | -- | Yes | -- |
| `y` | Walls normal to the Y direction | array[2] of boolean | -- | Yes | -- |
| `z` | Walls normal to the Z direction | array[2] of boolean | -- | Yes | -- |

For `cyl_walls`, both of the following are required:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `side` | Cylindrical surfaces [inner, outer] | array[2] of boolean | -- | Yes | -- |
| `z` | End caps normal to the Z direction | array[2] of boolean | -- | Yes | -- |

Note that walls may be defined on the fluid domain *or* on a container, but not both: specifying a `container` after
`fluid_domain` has already declared walls is an error.

#### Computational domain

The `computational_domain` key, if present, bounds the region in which SPH particles are tracked and sets the boundary
condition type in each direction.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `aabb_min` | Min corner of the computational domain AABB | array[3] | -- | Yes | -- |
| `aabb_max` | Max corner of the computational domain AABB | array[3] | -- | Yes | -- |
| `x_bc_type` | BC type in the X direction (negative and positive X) | enum | `NONE`,`PERIODIC`,`INLET_OUTLET` | No | `NONE` |
| `y_bc_type` | BC type in the Y direction (negative and positive Y) | enum | `NONE`,`PERIODIC`,`INLET_OUTLET` | No | `NONE` |
| `z_bc_type` | BC type in the Z direction (negative and positive Z) | enum | `NONE`,`PERIODIC`,`INLET_OUTLET` | No | `NONE` |

For `CARTESIAN` geometry, a boundary condition type other than `NONE` in a given direction requires that no fluid
domain wall was defined normal to that direction.

#### Wave tank

A `wave_tank` is a self-contained alternative to the domain definitions above.
It requires `physics_problem: CFD` and `geometry_type: CARTESIAN`.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Wave maker mechanism type | enum | `PISTON`,`FLAP` | Yes | -- |
| `tank_dimensions` | Dimensions of the wave tank container | array[3] | -- | Yes | -- |
| `water_depth` | Initial water depth in the tank | double | -- | Yes | -- |
| `actuation_function` | Function of time defining actuation of the wave maker | object | -- | Yes | -- |
| `tank_origin` | Origin of the wave tank container | array[3] | -- | No | [0, 0, 0] |
| `end_wall` | Whether to create a wall at the far end of the tank | boolean | -- | No | `true` |
| `profile` | Profile of the tank bottom, as [[x, z], ...] data points | array | -- | No | flat bottom |

The `actuation_function` is a generic function object (see the
[MBS model schema](@ref YAML_schema_mbs_model) for the available function types). In addition, it accepts an optional
`delay` property specifying a time offset in seconds before wave maker actuation begins (default: 0).

## Example

Below is an example of an SPH model configuration:

\include data/yaml/fsi/dam_break/model_sph.yaml

## YAML schema

The YAML SPH model specification file must follow the ``data/yaml/schema/fsisph_model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsisph_model.schema.yaml
