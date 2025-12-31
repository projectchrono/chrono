YAML schema for Chrono MBS simulation specification {#YAML_schema_mbs_simulations}
=======================================

A Chrono YAML MBS simulation file defines the parameters needed to run a Chrono simulation. It consists of two main objects:
- The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- The `simulation` object that contains simulation methods, solver and integrator settings, and visualization options.

## Simulation specification

The `simulation` object in a Chrono YAML simulation specification file defines:
    1. **Time Settings**: How long to run the simulation and at what resolution
    2. **Contact Settings**: How to handle collisions and contacts
    3. **Solver Settings**: How to solve the equations of motion
    4. **Visualization Settings**: How to display the simulation

<div class="ce-info">
Enum values for various object types in the simulation description YAML file can be provided using any desired case.
For example, the "Euler implicit" integrator can be specified using any of `euler_implicit`, `EULER_IMPLICIT`, `Euler_implicit`, `EUler_imPLICIt`, etc.
</div>

### Required Parameters

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `time_step` | Integration timestep in seconds | double | -- | Yes | -- |
| `contact_method` | Contact method for collision detection and response | string | `SMC`,`NSC` | Yes | -- |

### Optional Parameters

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `enforce_realtime` | Whether to enforce real-time simulation | boolean | -- | No | false |
| `end_time` | Total simulation time in seconds | double | -- | No | -1 for infinite simulation |
| `gravity` | Gravitational acceleration vector [x, y, z] | array[3] | -- | No | [0, 0, -9.8] |

### Integrator, solver, and visualization settings

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `integrator` | Integrator type and parameters | object | `EULER_IMPLICIT_LINEARIZED`,<br>`EULER_IMPLICIT_PROJECTED`,<br>`EULER_IMPLICIT`,<br>`HHT` | No | `EULER_IMPLICIT_LINEARIZED`  |
| `solver` | (DVI or linear) solver type and parameters | object |  `BARZILAI_BORWEIN`,<br>`PSOR`,<br>`APGD`,<br>`MINRES`,<br>`GMRES`,<br>`BICGSTAB`,<br>`PARDISO`,<br>`MUMPS`,<br>`SPARSE_LU`,<br>`SPARSE_QR` | No | `BARZILAI_BORWEIN` |
| `visualization` | Run-time visualization settings | object | -- | No | `false` |

### Integrator types and parameters

Each integrator can support the following settings depending on the integrator type:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `rel_tolerance` | Relative tolerance (HHT and implicit Euler) | double | -- | No | 1e-4 |
| `abs_tolerance_states` | Absolute tolerance for state variables (HHT and implicit Euler) | double | -- | No | 1e-4 |
| `abs_tolerance_multipliers` | Absolute tolerance for Lagrange multipliers (HHT and implicit Euler) | double | -- | No | 1e2 |
| `max_iterations` | Maximum number of non-linear iteration for implicit integrators | integer | -- | No | 50 |
| `use_stepsize_control` | Whether to use internal step-size control (HHT) | boolean | -- | No | false |
| `use_modified_newton` | Whether to use a modified Newton iteration (HHT) | boolean | -- | No | false |


### Solver types and parameters

Each solver supports different configuration parameters depending on the solver type: 

#### Iterative DVI Solvers (BARZILAI_BORWEIN, APGD, PSOR)

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `max_iterations`| Maximum number of iterations | integer | -- | No | 100 |
| `overrelaxation_factor` | Overrelaxation factor for improved convergence | double | -- | No | 1.0 |
| `sharpness_factor` | Sharpness factor for solver response tuning | double | -- | No | 1.0 |

#### Iterative Krylov Linear Solvers (BICGSTAB, MINRES, GMRES)

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `max_iterations` | Maximum number of iterations | integer | -- | No | 100 |
| `tolerance` | Residual tolerance for convergence | double | -- | No | 0.0 |
| `enable_diagonal_preconditioner` | Enable diagonal preconditioner to accelerate convergence | boolean | -- | No | false |

#### Direct Sparse Linear Solvers (SPARSE_LU, SPARSE_QR, PARDISO_MKL, MUMPS)

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `lock_sparsity_pattern`| Keep matrix sparsity pattern unchanged | boolean | -- | No | false |
| `use_sparsity_pattern_learner` | Evaluate matrix sparsity pattern in a pre-processing stage (for `SPARSE_LU` and `SPARSE_QR` only) | boolean | -- | No | true |

### Run-time visualization parameters

If an entry with key `visualization` is present, run-time visualization will be enabled.
The following optional parameters can be set:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Type of visualization | string | `MODEL_FILE`,`PRIMITIVES`,<br>`COLLISION`,`NONE` | No | `MODEL_FILE` |
| `render_fps` | Target frames per second for visualization | integer | -- | No | 120 |
| `enable_shadows` | Whether or not shadows are enabled in the visualization system | boolean | -- | No | true |
| `camera` | Camera vertical, location, and look-at point | object | -- | No | see `camera` object below |


#### Camera Settings

The `camera` object specifies the initial view configuration for run-time visualization. All fields are optional; if omitted, defaults will be used.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `vertical` | Vertical axis for camera orientation | string | `Y`,`Z` | No | `Z` |
| `location` | Initial camera location [x, y, z] | array[3] | -- | No | [0, -1, 0] |
| `target` | Initial camera look-at point [x, y, z] | array[3] | -- | No | [0, 0, 0] |

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

The YAML model specification file must follow the ``data/yaml/schema/mbs_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/mbs_simulation.schema.yaml
