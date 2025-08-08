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

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `enforce_realtime` | Whether to enforce real-time simulation | boolean | false |
| `end_time` | Total simulation time in seconds | number | -1 for infinite simulation |
| `gravity` | Gravitational acceleration vector [x, y, z] | array[3] | [0, 0, -9.8] |

### Integrator, solver, and visualization settings

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `integrator` | Integrator type and parameters | object | `EULER_IMPLICIT_LINEARIZED` |
| `solver` | (DVI or linear) solver type and parameters | object | `BARZILAIBORWEIN` |
| `visualization` | Run-time visualization settings | object | `false` |

### Integrator types and parameters

The integrator can be of one of the following types:
- `EULER_IMPLICIT_LINEARIZED`
- `EULER_IMPLICIT_PROJECTED`
- `EULER_IMPLICIT`
- `HHT`

Each integrator can support the following settings depending on the integrator type:

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `rel_tolerance` | Relative tolerance (HHT and implicit Euler) | number | 1e-4 |
| `abs_tolerance_states` | Absolute tolerance for state variables (HHT and implicit Euler) | number | 1e-4 |
| `abs_tolerance_multipliers` | Absolute tolerance for Lagrange multipliers (HHT and implicit Euler) | number | 1e2 |
| `max_iterations` | Maximum number of non-linear iteration for implicit integrators.| number | 50 |
| `use_stepsize_control` | Whether to use internal step-size control  | boolean | false |
| `use_modified_newton` | Whether to use a modified Newton iteration | boolean | false |


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

Each solver supports different configuration parameters depending on the solver type: 

#### Iterative DVI Solvers (BARZILAI_BORWEIN, APGD, PSOR)

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `max_iterations`| Maximum number of iterations | number | 100 |
| `overrelaxation_factor` | Overrelaxation factor for improved convergence | number | 1.0 |
| `sharpness_factor` | Sharpness factor for solver response tuning | number | 1.0 |

#### Iterative Krylov Linear Solvers (BICGSTAB, MINRES, GMRES)

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `max_iterations` | Maximum number of iterations | number  | 100 |
| `tolerance` | Residual tolerance for convergence | number  | 0.0 |
| `enable_diagonal_preconditioner` | Enable diagonal preconditioner to accelerate convergence | boolean | fasle |

#### Direct Sparse Linear Solvers (SPARSE_LU, SPARSE_QR, PARDISO_MKL, MUMPS)

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `lock_sparsity_pattern`| Keep matrix sparsity pattern unchanged | boolean | false |
| `use_sparsity_pattern_learner` | Evaluate matrix sparsity pattern in a pre-processing stage (for `SPARSE_LU` and `SPARESE_QR` only) | boolean | true |

### Run-time visualization parameters

If an entry with key `visualization` is present, run-time visualization will be enabled.
The following optional parameters can be set:

| Property | Description | Type |
|----------|-------------|------|
| `render_fps` | Target frames per second for visualization | number |
| `enable_shadows` | Whether or not shadows are enabled in the visualization system | boolean |
| `camera` | Camera vertical, location, and look-at point | object |


#### Camera Settings

The `camera` object specifies the initial view configuration for run-time visualization. All fields are optional; if omitted, defaults will be used.

| Property | Description | Type | Default |
|----------|-------------|------| ------- |
| `vertical` | Vertical axis for camera orientation| string (`Y` or `Z`)  | `Z` |
| `location` | Initial amera location [x, y, z]| array[3]| [0, -1, 0]|
| `target` | Initial camera look-at point [x, y, z] | array[3]|[0,  0, 0]|

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
