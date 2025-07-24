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

Joints connect two bodies and constrain their relative motion. They can be represented through constraints (kinematic joints) or through stiff compliance (bushings).

| Property | Description | Required |
|----------|-------------|----------|
| `type` | Joint type (revolute, prismatic, spherical, etc.) | Yes |
| `name` | Unique identifier for the joint | Yes |
| `body1` | Name of the first body to connect | Yes |
| `body2` | Name of the second body to connect | Yes |
| `bushing_data` | Bushing compliance data; if not present, the joint is kinematic | No |

### Passive spring-damper force elements

Spring-damper elements apply forces between bodies.

| Property | Description | Required |
|----------|-------------|----------|
| `type` | Spring-damper type type (TSDA, RSDA) | Yes |
| `name` | Unique identifier for the force element | Yes |
| `body1` | Name of the first body to connect | Yes |
| `body2` | Name of the second body to connect | Yes |

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

### Motors


## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/model.schema.yaml
