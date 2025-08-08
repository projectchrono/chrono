YAML schema for Chrono model specification {#YAML_schema_models}
=======================================

## Model specification

A YAML model file `model.yaml` defines the mechanical system for the simulation. It defines:

1. **Bodies**: rigid bodies that carry mass and inertia and, optionally, collision and visualization geometry
2. **Joints**: connection between a pair of bodies, specified either as a kinematic (ideal) joint or as a bushing
3. **Passive force elements**: translational and rotational linear or non-linear spring-damper force elements acting between two rigid bodies
4. **Motors and actuators**: 
   - translational and rotational motors acting between two rigid bodies at the position (displacement or angle), velocity (linear or angular speed), or force (force or torque) levels. Motors are specified through a time function for the control input (position, velocity, or force)
   - external actuators
5. (Optional) One can set `angle_degrees` to `true` or `false` to use degrees or radiance for angle units. Default is `true`. 

### Bodies

Each body represents a physical object in the simulation with the following properties:

| Property | Description | Required | Default |
|----------|-------------|----------|---------|
| `name` | Unique identifier for the body | Yes | |
| `fixed` | Indicates if body fixed relative to global frame | No | false |
| `mass` | Mass in kg | Yes, if body not fixed | |
| `com`  | Center of Mass, relative to body reference frame | No | same as body reference frame |
| `inertia`->`moments` | Moments of inertia [Ixx, Iyy, Izz] relative to centroidal frame | Yes, if body not fixed | |
| `inertia`->`products` | Products of inertia [Ixy, Iyz, Izx] relative to centroidal frame | No | [0, 0, 0] |
| `location` | Origin of the body reference frame, relative to model frame | Yes | |
| `orientation` | Orientation of the body reference frame relative to model frame | No | identity rotation |

### Joints

Joints connect two bodies and constrain their relative motion. They can be represented through constraints (kinematic joints) or through stiff compliance (bushings). Supported joint `type` are: `lock`, `pointline`, `pointplane`, `revolute`, `spherical`, `prismatic` and `universal`. Default `type` is `lock`.

| Property | Description | Required |
|----------|-------------|----------|
| `type` | Joint type (revolute, prismatic, spherical, etc.) | Yes |
| `name` | Unique identifier for the joint | Yes |
| `body1` | Name of the first body to connect | Yes |
| `body2` | Name of the second body to connect | Yes |
| `location` | Joint location | Yes |
| `axis` | Axis of motion for revolute/prismatic joints | Yes for revolute/prismatic joints|
| `axis1` | First axis for universal joints | Yes for universal joints |
| `axis2` | Second axis for universal joints | Yes for universal joints |
| `bushing_data` | Bushing compliance data; if not present, the joint is kinematic | No |

The `bushing_data` models compliance behavior along the joint's degrees of freedom, instead of a rigid connection. 
Note that a bushing of `joint` type `prismatic`, `pointline` or `pointplane` is prohibited. For the constrained DOF, one can specify stiffness and damping coefficients, 

| Property | Description | Type |
|----------|-------------|----------|
| `stiffness_linear` | Linear stiffness coefficient for "constrained" translational DOFs | number |
| `damping_linear` | Linear damping coefficient for "constrained" translational DOFs | number |
| `stiffness_rotational` | Rotational stiffness coefficient for "constrained" rotational DOFs | number |
| `damping_rotational` | Rotational damping coefficient for "constrained" rotational DOFs | number |

One can also apply compliance to the unconstrained DOF using a nested `DOF` object, for example,

```yaml
bushing_data:
  stiffness_linear: 7e7
  damping_linear: 0.35e5
  stiffness_rotational: 1e5
  damping_rotational: 5e3
  DOF:
    stiffness_linear: 5000
    damping_linear: 50
    stiffness_rotational: 500
    damping_rotational: 25
```

Note that the bushing formulation is valid only for small relative rotations. For rotational stiffness in the presence of large rotations, use a rotational spring-damper element, `RSDA`.

### Constraints
Constraints connect two bodies and constrain their relative motion. Supported constraint `type` are: `DISTANCE`, `REVOLUTE-SPHERICAL`, `REVOLUTE-TRANSLATIONAL`. 

| Property | Description | Required |
|----------|-------------|----------|
| `type` | Constraint type | Yes |
| `name` | Unique identifier for the constraint | Yes |
| `body1` | Name of the first body to connect | Yes |
| `body2` | Name of the second body to connect | Yes |
| `point1` | Point on body1 expressed in global frame | Yes |
| `point2` | Point on body2 expressed in global frame | Yes |

### Passive spring-damper force elements

There are two types of spring-damper elements, translational and rotational, speceified as `TSDA` or `RSDA`. For a `TSDA` element, one can have the following fields,

| Property | Description | Required |
|----------|-------------|----------|
| `name` | Unique identifier for the force element | Yes |
| `type` | Spring-damper type (`TSDA` or `RSDA`) | Yes |
| `body1` | Name of the first body to connect | Yes |
| `body2` | Name of the second body to connect | Yes |
| `point1` | Point on body1 expressed in global frame [x,y,z] | Yes |
| `point2` | Point on body2 expressed in global frame [x,y,z] | Yes |
| `free_length` | TSDA free length | No |
| `preload` | preload force | No |
| `minimum_length` | Minimum TSDA length | No |
| `maximum_length` | Maximum TSDA length | No |


For linear spring-dampers, use the `spring_coefficient` and `damping_coefficient` properties. For example:

```yaml
spring_dampers:
  - name: linear_spring_damper
    type: TSDA
    body1: first_body
    body2: second_body
    point1: [6.5, 0, 0]
    point2: [5.5, 0, 0]
    spring_coefficient: 50.0
    damping_coefficient: 5.0
    free_length: 1.0
    visualization:
      type: SPRING
      radius: 0.05
      resolution: 80
      turns: 15
```

For nonlinear behavior, use `spring_curve_data` and/or `damping_curve_data` and/or the pair `deformation`/`map_data`. These properties allow specification of spring-damper characteristics as tabular data (which will be linearly interpolated by Chrono at run-time). For example:

```yaml
spring_dampers:
  - name: nonlinear_spring_damper
    type: TSDA
    body1: first_body
    body2: second_body
    point1: [6.5, 0, 0]
    point2: [5.5, 0, 0]
    deformation: [ 0.273304, 0.278384, 0.283464, 0.288544, 0.293624, 0.324104, 0.343002, 0.361899, 0.380797, 0.399694, 0.418592, 0.423672, 0.428752, 0.433832, 0.438912 ],
    map_data: [
      [ -0.666667, -5691.62, -5691.62, -5691.62, -5691.62, -5691.62, -9690.35, -9690.35, -9690.35, -9690.35, -9690.35, -9690.35, -5691.62, -5691.62, -5691.62, -5691.62 ],
      [ -0.333333, -2845.81, -2845.81, -2845.81, -2845.81, -2845.81, -4845.17, -4845.17, -4845.17, -4845.17, -4845.17, -4845.17, -2845.81, -2845.81, -2845.81, -2845.81 ],
      [ 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ],
      [ 0.333333, 21307.1, 21307.1, 21307.1, 21307.1, 21307.1, 11675.1, 11675.1, 11675.1, 11675.1, 11675.1, 11675.1, 21307.1, 21307.1, 21307.1, 21307.1 ],
      [ 0.666667, 42614.2, 42614.2, 42614.2, 42614.2, 42614.2, 23350.2, 23350.2, 23350.2, 23350.2, 23350.2, 23350.2, 42614.2, 42614.2, 42614.2, 42614.2 ]
    ]
```

Optionally, you can add a `visualization` object to render the TSDA element, 

| Property | Description                            | Type |
| -------- | -------------------------------------- | ---- |
| `type`   | Type of visualization geometry (`segment` or `spring`)       | string |
| `color`  | RGB color of the element `[r, g, b]`                         | array[3] |
| `radius` | Radius of the visualized TSDA geometry (if `spring`)         | number  |
| `resolution` | Number of subdivisions along one coil turn (if `spring`) | integer |
| `turns`      | Number of coil turns (if `spring`)                       | integer |

Rotational spring-damper elements, `RSDA`, apply torques between bodies. One can specifiy the following fields, 

| Property | Description | Required |
|----------|-------------|----------|
| `name` | Unique identifier for the element | Yes |
| `body1` | Name of the first body to connect | Yes |
| `body2` | Name of the second body to connect | Yes |
| `location` | RSDA location expressed in global frame [x,y,z] | No |
| `axis` | RSDA axis expressed in global frame [x,y,z] | Yes |
| `free_angle` | RSDA free length | No |
| `preload` | preload torque | No |
| `spring_coefficient`  | Linear spring coefficient | No |
| `damping_coefficient` | Linear damping coefficient | No |
| `spring_curve_data`   | Nonlinear spring curve data [[angle, torque], ...] | No |
| `damping_curve_data`  | Nonlinear damping curve data [[angular velocity, torque], ...] | No |


### Motors

| Property | Description | Required | Type | Values |
|----------|-------------|----------|------|--------|
| `name` | Unique identifier for the motor | Yes | string | |
| `type` | Type of motor | Yes | string | `LINEAR`, `ROTATION` |
| `body1` | Name of the first body to connect | Yes | string | |
| `body2` | Name of the second body to connect | Yes | string | |
| `location` | Motor location expressed in the global frame at [x,y,z] | Yes | array[3] | `[x, y, z]` |
| `axis` | Motor axis expressed in the global frame at [x,y,z] | Yes | array[3] | `[x, y, z]`                           |
| `actuation_type` | Type of actuation | Yes | string | `POSITION`, `SPEED`, `FORCE` |
| `actuation_function` | Function of time defining the actuation | Yes | object | See [Functions](#functions) |
| `guide` | Guide constraint for linear motors | No | string | `FREE`, `PRISMATIC`, `SPHERICAL` |
| `spindle` | Spindle constraint for rotational motors | No | string | `FREE`, `REVOLUTE`, `CYLINDRICAL` |

### Functions

A `function` object is used to specify actuation inputs. The only required field is `type`, which determines which additional fields are valid. Supported function types include:

- `CONSTANT` defines a function with a fixed `value`;
- `POLYNOMIAL` defines a polynomial function of the form `f(x) = a₀ + a₁x + a₂x² + ...` using the `coefficients` field: `[a₀, a₁, a₂, ...]`;
- `SINE` defines a sinusoidal function with `amplitude`, `frequency` and `phase`;
- `RAMP` defines a ramp function with `slope` and `intercept`;
- `DATA` interpolates a series of data points, for example: 

```yaml
type: DATA
data:
  - [0.0, 0.0]
  - [1.0, 5.0]
  - [2.0, 2.5]
```

All function types can include an optional `repeat` field to periodically replicate the function, for example,

```yaml
repeat:
  start: 1.0
  width: 2.0
  shift: 3.0
```
## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/model.schema.yaml
