YAML schema for Chrono MBS model specification {#YAML_schema_mbs_models}
=======================================

A Chrono YAML MBS model file defines a mechanical system and contains two main objects:
- The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- The `model` object that lists all physics items in the Chrono model.

## Model specification

The `model` object in a Chrono YAML model specification file defines:
    1. **Bodies**: (Required)rigid bodies that carry mass and inertia and, optionally, collision and visualization geometry
    2. **Joints**: connection between a pair of bodies, specified either as a kinematic (ideal) joint or as a bushing
    3. **Passive force elements**: translational and rotational linear or non-linear spring-damper force elements acting between two rigid bodies
    4. **Motors and actuators**: 
        - translational and rotational motors acting between two rigid bodies at the position (displacement or angle), velocity (linear or angular speed), or force (force or torque) levels. Motors are specified through a time function for the control input (position, velocity, or force)
        - external actuators
    5. **Constraints**: additional constraint equations between bodies
    6. **External loads**: external loads applied to bodies

<div class="ce-info">
Enum values for various object types in the model description YAML file can be provided using any desired case.
For example, a "point-line" joint can be specified using any of `point_line`, `POINT_LINE`, `Point_Line`, `POinT_liNE`, etc.
</div>

The table below lists the main fields of the `model` object:

| Property | Description | Type | Available Values | Required | Default |
|-------|-------------|------|----------|---------|---------|
| `name` | Optional model name for identification | string | -- | No | 'YAML model' |
| `angle_degrees` | Whether angles are in degrees (true) or radians (false) | boolean | -- | No | true |
| `data_path` | Configuration for data file locations | object, see below | -- | No | using absolute paths |
| `bodies` | Array of body objects | array[`body`] | -- | Yes | -- |
| `joints` | Array of joint objects | array[`joint`] | -- | No | -- |
| `constraints` | Array of constraint objects | array[`constraint`] | -- | No | -- |
| `tsdas` | Array of TSDA (translational spring-damper) objects | array[`tsda`] | -- | No | -- |
| `rsdas` | Array of RSDA (rotational spring-damper) objects | array[`rsda`] | -- | No | -- |
| `motors` | Array of motor objects | array[`motor`] | -- | No | -- |
| `body_loads` | Array of external loads applied to bodies | array[`body_load`] | -- | No | -- | 

The `data_path` object can have the following fields if specified:

| Field | Description | Type | Available Values | Required | Default |
|-------|-------------|------|----------|---------|---------|
| `type` | Type of data path | string | `RELATIVE` or `ABSOLUTE` | Yes | -- |
| `root` | Root directory for data file locations | string | -- | No | `.` (current directory) |

### Bodies

Each body represents a physical object in the simulation with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Unique identifier for the body | string | -- | Yes | -- |
| `fixed` | Indicates if body fixed relative to global frame | boolean | -- | No | false |
| `mass` | Mass in kg | double | -- | Yes, if body not fixed | -- |
| `com`->`location` | Origin of the COM frame relative to body reference frame | array[3] | -- | No | [0, 0, 0] |
| `com`->`orientation` | Orientation of the COM frame relative to body reference frame | array[3] or array[4] | -- | No | identity rotation |
| `inertia`->`moments` | Moments of inertia [Ixx, Iyy, Izz] relative to centroidal frame | array[3] | -- | Yes, if body not fixed | -- |
| `inertia`->`products` | Products of inertia [Ixy, Iyz, Izx] relative to centroidal frame | array[3] | -- | No | [0, 0, 0] |
| `location` | Origin of the body reference frame, relative to model frame | array[3] | -- | Yes | -- |
| `orientation` | Orientation of the body reference frame relative to model frame | array[3] or array[4] | -- | No | identity rotation |
| `initial_linear_velocity` | Initial linear velocity of the body reference frame | array[3] | -- | No | [0, 0, 0] |
| `initial_angular_velocity` | Initial angular velocity of the body reference frame, expressed in local body frame | array[3] | -- | No | [0, 0, 0] |
| `contact` | List of contact materials, `materials` and collision shapes, `shapes` | object, see below | -- | No | no contact |
| `visualization` | List of visualization shapes, `shapes` | object, see below | -- | No | no visualization |

**Note:** `orientation` can be specified as Euler Cardan angles [yaw, pitch, roll] or quaternion [e0, e1, e2, e3]

#### Body Contact Properties

The collision of a body is specified through a list of contact material, `materials` and a list of collision shapes, `shapes`.
The model can have more `shapes` than `materials`, and the user need to specify which `shape` is associated with which `material`.
Depending on the contact method, smooth contact formulation (SMC) or non-smooth contact formulation (NSC), the same contact paramters, such as `coefficient_of_friction` and `coefficient_of_restitution`, can result in different physics. When using the SMC formulation, the user can specify material-based properties, such as `Youngs_modulus` and `Poisson_ratio`, or spring-damper coefficients for modeling contact, such as `normal_stiffness` and `normal_damping`.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `materials` | Contact material properties | array[`material`], see Contact Material below | -- | Yes | -- |
| `shapes` | Collision shapes for contact detection | array[`shape`], see Collision Shape below | -- | Yes | -- |

##### Contact Material

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Unique identifier for the material | string | -- | Yes | -- |
| `coefficient_of_friction` | Friction coefficient | double | -- | No | 0.8 |
| `coefficient_of_restitution` | Coefficient of restitution | double | -- | No | 0.01|
| `properties` | (SMC only) contact material-based properties, such as Young's modulus and Poisson's ratio | object | -- | No | see below |
| `Coefficients` | (SMC only) contact spring-damper coefficients, such as normal stiffness and damping | object | -- | No | see below |
<br>
Properties for `material`->`properties`:
| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `Youngs_modulus` | Young's modulus of the material | double | -- | Yes | 2e7 if `properties` not specified |
| `Poisson_ratio` | Poisson's ratio of the material | double | -- | Yes | 0.3 if `properties` not specified |
<br>
Properties for `material`->`Coefficients`:
| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `normal_stiffness` | Normal stiffness coefficient | double | -- | Yes | 2e5 if `Coefficients` not specified |
| `normal_damping` | Normal damping coefficient | double | -- | Yes | 40 if `Coefficients` not specified |
| `tangential_stiffness` | Tangential stiffness coefficient | double | -- | Yes | 2e5 if `Coefficients` not specified |
| `tangential_damping` | Tangential damping coefficient | double | -- | Yes | 20 if `Coefficients` not specified |

##### Collision Shape

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Collision shape type | string | `SPHERE`, `BOX`, <br> `CYLINDER`, `MESH`, <br> `HULL` | Yes | -- |
| `material` | Name of the contact `material` used for the shape, see above | string | -- | Yes | -- |
| `location` | Shape location relative to body reference frame | array[3] | -- | Yes , except for `MESH` | [0, 0, 0] for `MESH` |
| `orientation` | Shape orientation relative to body reference frame | array[3] or array[4] | -- | Yes , except for `MESH` | identity rotation for `MESH` |
| `radius` | Radius for `SPHERE` and `CYLINDER` shapes | double | -- | Yes |-- |
| `dimensions` | Dimensions [length, width, height] for `BOX` shape | array[3] | -- | Yes | -- |
| `axis` | Axis direction for `CYLINDER` shape | array[3] | -- | Yes | -- |
| `length` | Length for `CYLINDER` shape | double | -- | Yes | -- |
| `filename` | Filename for `HULL` and `MESH` shapes | string | -- | Yes | -- |
| `contact_radius` | Contact radius for `MESH` shape | double | -- | No | 0 |
| `scale` | Scale factor for `MESH` shape | double | -- | No | 1.0 |

#### Body Visualization Properties

The visualization of the body can either be one single `model_file` and/or a list of `shapes` (of type `SPHERE`, `BOX`, `CYLINDER`,or `MESH`).
If provided, a `model_file` is passed directly, as-is to the run-time visualization system.
On the other hand, a `MESH` shape is assumed to be provided only through an OBJ Wavefront file and also provides support for translating, rotating, and scaling.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `model_file` | Path to the model file for visualization | string | -- | No | -- |
| `shapes` | List of shapes for visualization | array, see below | -- | No | -- |

##### Visualization Shapes

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Visualization shape type | string | `SPHERE`, `BOX`, <br> `CYLINDER`, `MESH` | Yes | -- |
| `location` | Shape location relative to body reference frame | array[3] | -- | Yes , except for `MESH` | [0, 0, 0] for `MESH` |
| `orientation` | Shape orientation relative to body reference frame | array[3] or array[4] | -- | Yes , except for `MESH` | identity rotation for `MESH` |
| `radius` | Radius for `SPHERE` and `CYLINDER` shapes | double | -- | Yes | -- |
| `dimensions` | Dimensions [length, width, height] for `BOX` shape | array[3] | -- | Yes | -- |
| `axis` | Axis direction for `CYLINDER` shape | array[3] | -- | Yes | -- |
| `length` | Length for `CYLINDER` shape | double | -- | Yes | -- |
| `filename` | Filename for `MESH` shapes | string | -- | Yes | -- |
| `scale` | Scale factor for `MESH` shape | double | -- | No | 1.0 |
| `color` | Color of the shape in RGB format [r, g, b] | array[3] | -- | No | [-1, -1, -1] |

### Joints

Joints are connection between two bodies and constrain their relative motion.
They can be represented through constraints (ideal kinematic joints) or through stiff compliance (bushings).
Currently supported joint `type` are: `LOCK`, `REVOLUTE`, `SPHERICAL`, `PRISMATIC`, `UNIVERSAL`, `POINT_LINE`, `POINT_PLANE`.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Joint type | string | `LOCK`,<br> `REVOLUTE`,<br>`SPHERICAL`,<br>`PRISMATIC`,<br>`UNIVERSAL`,<br>`POINT_LINE`,<br>`POINT_PLANE` | Yes | -- |
| `name` | Unique identifier for the joint | string | -- | Yes | -- |
| `body1` | Name of the first body to connect | string | -- | Yes | -- |
| `body2` | Name of the second body to connect | string | -- | Yes | -- |
| `location` | Joint location | array[3] | -- | Yes | -- |
| `axis` | Axis of motion for revolute/prismatic joints | array[3] | -- | Yes | -- |
| `axis1` | First axis for universal joints | array[3] | -- | Yes | -- |
| `axis2` | Second axis for universal joints | array[3] | -- | Yes | -- |
| `bushing_data` | Bushing compliance data; if not present, the joint is kinematic | object | -- | No | no bushing |

The `bushing_data` models compliance behavior along the joint's constrained degrees of freedom (i.e., relaxations of the rigid constraints for an ideal kinematic joint). 
Note that a `joint` of type `PRISMATIC`, `POINT_LINE` or `POINT_PLANE` is prohibited.

For the constrained DOF, one can specify stiffness and damping coefficients, 

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `stiffness_linear` | Linear stiffness coefficient for "constrained" translational DOFs | double | -- | Yes | -- |
| `damping_linear` | Linear damping coefficient for "constrained" translational DOFs | double | -- | Yes | -- |
| `stiffness_rotational` | Rotational stiffness coefficient for "constrained" rotational DOFs | double | -- | Yes | -- |
| `damping_rotational` | Rotational damping coefficient for "constrained" rotational DOFs | double | -- | Yes | -- |
| `DOF` | Degree of freedom specific properties | object | -- | No | all 0.0 |

Optionally, one can apply compliance to the unconstrained DOF using a nested `DOF` object, for example:

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

If the `DOF` object is not present, the linear and rotational stiffness and damping in the direction of the unconstrained DOF of the bushing are all set to 0.0.


### Constraints
Constraints connect two bodies and constrain their relative motion. Supported constraint `type` are: `DISTANCE`, `REVOLUTE-SPHERICAL`, `REVOLUTE-TRANSLATIONAL`. 

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Constraint type | string | `DISTANCE`,<br>`REVOLUTE-SPHERICAL`,<br>`REVOLUTE-TRANSLATIONAL` | Yes | -- |
| `name` | Unique identifier for the constraint | string | -- | Yes | -- |
| `body1` | Name of the first body to connect | string | -- | Yes | -- |
| `body2` | Name of the second body to connect | string | -- | Yes | -- |
| `point1` | Point on body1 expressed in global frame | array[3] | -- | Yes | -- |
| `point2` | Point on body2 expressed in global frame | array[3] | -- | Yes | -- |

### Passive spring-damper force elements

There are two types of spring-damper elements, translational (TSDA) and rotational (RSDA).

#### Translational spring-dampers

TSDA elements apply forces on the connected bodies.
In the YAML specification file, they are listed in an array `tsdas` containing objects with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Unique identifier for the force element | string | -- | Yes | -- |
| `body1` | Name of the first body to connect | string | -- | Yes | -- |
| `body2` | Name of the second body to connect | string | -- | Yes | -- |
| `point1` | Point on body1 expressed in global frame [x,y,z] | array[3] | -- | Yes | -- |
| `point2` | Point on body2 expressed in global frame [x,y,z] | array[3] | -- | Yes | -- |
| `free_length` | TSDA free length | double | -- | Yes | -- |
| `preload` | preload force | double | -- | No | 0 |
| `minimum_length` | Minimum TSDA length | double | -- | No | -- |
| `maximum_length` | Maximum TSDA length | double | -- | No | -- |
| `visualization` | Visualization properties of the TSDA element | object | -- | No | no visualization |


For linear spring-dampers, use the `spring_coefficient` and `damping_coefficient` properties. 
For example:

```yaml
tsdas:
  - name: linear_spring_damper
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

For nonlinear behavior, use `spring_curve_data` and/or `damping_curve_data` and/or the pair `deformation`/`map_data`. 
These properties allow specification of spring-damper characteristics as tabular data (which will be linearly interpolated by Chrono at run-time). 
For example:

```yaml
tsdas:
  - name: nonlinear_spring_damper
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

Optionally, a `visualization` object specifies rendering of the TSDA element:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Type of visualization geometry | string | `SEGMENT`,`SPRING` | YES | -- |
| `color` | RGB color of the element `[r, g, b]` | array[3] | -- | No | [0, 0, 0] |
| `radius` | Radius of the visualized TSDA geometry (if `SPRING`) | double | -- | No | 0.05 |
| `resolution` | Number of subdivisions along one coil turn (if `SPRING`) | integer | -- | No | 65 |
| `turns` | Number of coil turns (if `SPRING`) | integer | -- | No | 5 |

#### Rotational spring-dampers

Rotational spring-damper elements (RSDA) apply torques between the two connected bodies. 
In the YAML peicifcation file, they are listed in an array `rsdas` containing objects with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Unique identifier for the element | string | -- | Yes | -- |
| `body1` | Name of the first body to connect | string | -- | Yes | -- |
| `body2` | Name of the second body to connect | string | -- | Yes | -- |
| `location` | RSDA location expressed in global frame [x,y,z] | array[3] | -- | No | [0, 0, 0] |
| `axis` | RSDA axis expressed in global frame [x,y,z] | array[3] | -- | Yes | -- |
| `free_angle` | RSDA free angle | double | -- | Yes | -- |
| `preload` | preload torque | double | -- | No | 0 |
| `spring_coefficient`  | Linear spring coefficient | double | -- | Yes for linear spring | -- |
| `damping_coefficient` | Linear damping coefficient | double | -- | Yes for linear damper | -- |
| `spring_curve_data`   | Nonlinear spring curve data [[angle, torque], ...] | array, see above | -- | Yes for nonlinear spring | -- |
| `damping_curve_data`  | Nonlinear damping curve data [[angular velocity, torque], ...] | array, see above | -- | Yes for nonlinear damper | -- |

Linear or non-linear RSDA torques are specified similarly to corresponding linear or non-linear forces for a TSDA.


### Body Loads

Body loads represent external forces or torques applied to specific bodies in the simulation. 

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Unique identifier for the body load | string | -- | Yes | -- |
| `type` | Type of body load | string | `FORCE`, `TORQUE` | Yes | -- |
| `body` | Name of the body to which the load is applied | string | -- | Yes | -- |
| `load` | Load vector for force or torque | array[3] | -- | Yes | -- |
| `local_load` | Whether `load` is applied in local body frame (true) or global frame (false) | boolean | -- | Yes | -- |
| `point` | (`FORCE` only) Location of the applied `load` | array[3] | -- | Yes for `FORCE` | -- |
| `local_point` | (`FORCE` only) Whether `point` is specified in local body frame (true) or global frame (false) | boolean | -- | Yes for `FORCE` | -- |

### Motors
Motors are used to apply forces or torques to bodies. Supported motor `type` are: `LINEAR`, `ROTATION`. `LINEAR` motors apply forces along a specified `axis`, while `ROTATION` motors apply torques.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `name` | Unique identifier for the motor | string | -- | Yes | -- |
| `type` | Type of motor | string | `LINEAR`,<br>`ROTATION` | Yes | -- |
| `body1` | Name of the first body to connect | string | -- | Yes | -- |
| `body2` | Name of the second body to connect | string | -- | Yes | -- |
| `location` | Motor location expressed in the global frame at [x,y,z] | array[3] | -- | Yes | -- |
| `axis` | Motor axis expressed in the global frame at [x,y,z] | array[3] | -- | Yes | -- |
| `actuation_type` | Type of actuation | string | `POSITION`,<br>`SPEED`,<br>`FORCE` | Yes | -- |
| `actuation_function` | Function of time defining the actuation | object | -- | Yes | -- |
| `guide` | Guide constraint for linear motors | string | `FREE`,<br>`PRISMATIC`,<br>`SPHERICAL` | No | `PRISMATIC` |
| `spindle` | Spindle constraint for rotational motors | string | `FREE`,<br>`REVOLUTE`,<br>`CYLINDRICAL` | No | `REVOLUTE` |

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

All function types can include an optional `repeat` field to periodically replicate the function. 
For example:
```yaml
repeat:
  start: 1.0
  width: 2.0
  shift: 3.0
```
## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/mbs_model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/mbs_model.schema.yaml
