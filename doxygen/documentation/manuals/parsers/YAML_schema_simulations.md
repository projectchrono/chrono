YAML schema for Chrono simulation specification {#YAML_schema_simulations}
=======================================

A YAML file `simulation.yaml` defines the main parameters needed to run a Chrono simulation. It includes:

1. **Time Settings**: How long to run the simulation and at what resolution
2. **Contact Settings**: How to handle collisions and contacts
3. **Solver Settings**: How to solve the equations of motion
4. **Visualization Settings**: How to display the simulation

## File Structure

A simulation file contains these main sections:

### Required Parameters

| Property | Description | Type |
|----------|-------------|------|
| `time_step` | Integration timestep in seconds | number |
| `contact_method` | Contact method for collision detection and response | string (SMC/NSC) |

### Optional Parameters

| Property | Description | Type |
|----------|-------------|------|
| `end_time` | Total simulation time in seconds | number |
| `enforce_realtime` | Whether to enforce real-time simulation | boolean |
| `gravity` | Gravitational acceleration vector [x, y, z] | array[3] |

### Integrator, solver, and visualization settings

| Property | Description | Type |
|----------|-------------|------|
| `integrator` | Integrator type and parameters | object |
| `solver` | (DVI or linear) solver type and parameters | object |
| `visualization` | Run-time visualization settings | object |

### Integrator types and parameters

The integrator can be of one of the following types:
- `EULER_IMPLICIT_LINEARIZED`
- `EULER_IMPLICIT_PROJECTED`
- `EULER_IMPLICIT`
- `HHT`

### Solver types and parameters

The solver can be one of the following types:
- `BARZILAI_BORWEIN`
- `PSOR`
- `APGD`
- `MINRES`
- `GMRES`
- `BICGSTAB`
- `PARDISO`
- `MUMPS`
- `SPARSE_LU`
- `SPARSE_QR`

### Run-time visualization parameters

If an entry with key `visualization` is present, run-time visualization will be enabled.
The following optional parameters can be set:

| Property | Description | Type |
|----------|-------------|------|
| `render_fps` | Target frames per second for visualization | number |
| `enable_shadows` | Whether or not shadows are enabled in the visualization system | boolean |
| `camera` | Camera vertical, location, and look-at point | object |


## Example

Below is an example of a simulation configuration:

```yaml
# Basic MBS simulation

contact_method: SMC

time_step: 1e-4
end_time: 100
enforce_realtime: true

integrator:
    type: Euler_implicit_linearized

solver:
    type: Barzilai_Borwein
    max_iterations: 100
    overrelaxation_factor: 1.0
    sharpness_factor: 1.0

visualization:
    render_fps: 120
    enable_shadows: true
    camera:
        vertical: Z
        location: [9, -4, 1]
        target: [2, 0, 0]
```

## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/simulation.schema.yaml
