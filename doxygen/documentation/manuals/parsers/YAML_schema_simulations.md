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
| `gravity` | Gravitational acceleration vector [x, y, z] in m/s^2 | array[3] |
| `integrator_type` | Type of numerical integrator | string |
| `solver_type` | Type of linear solver | string |
| `render` | Whether to enable visualization | boolean |
| `render_fps` | Target frames per second for visualization | number |
| `camera_vertical` | Vertical axis for camera orientation | string (Y/Z) |

### Integrator Types

The `integrator_type` can be one of:
- `EULER_IMPLICIT_LINEARIZED`
- `EULER_IMPLICIT_PROJECTED`
- `EULER_IMPLICIT`
- `HHT`

### Solver Types

The `solver_type` can be one of:
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

## Example

Below is an example of a simulation configuration:

```yaml
# project_name.simulation.yaml

# Required parameters
time_step: 0.01
contact_method: SMC

# Optional parameters
end_time: 40.0
enforce_realtime: false
gravity: [0, 0, -9.8]
integrator_type: EULER_IMPLICIT
solver_type: BARZILAI_BORWEIN

# Visualization settings
render: true
render_fps: 120
camera_vertical: Z
```

## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/simulation.schema.yaml
