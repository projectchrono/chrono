YAML schema for Chrono MBS solver specification {#YAML_schema_mbs_solver}
=======================================

A Chrono YAML MBS solver file defines the parameters needed to run a Chrono simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The `contact_method` specifying the formulation for contact force generation.
- [required] The `integrator` object specifying the type and settgins for the time integrator.
- [required] The `solver` object specifying the type and settgins for the (linear or DVI) solver.

## Contact formulation

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `contact_method` | Contact method for collision detection and response | string | `SMC`,`NSC` | Yes | -- |

## Integrator types and parameters

Each integrator can support the following settings depending on the integrator type:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Integrator type | enum | `EULER_IMPLICIT_LINEARIZED`,<br>`EULER_IMPLICIT_PROJECTED`,<br>`EULER_IMPLICIT`,<br>`HHT` | Yes | `EULER_IMPLICIT_LINEARIZED`  |
| `rel_tolerance` | Relative tolerance (HHT and implicit Euler) | double | -- | No | 1e-4 |
| `time_step` | Integration timestep in seconds | double | -- | Yes | -- |
| `abs_tolerance_states` | Absolute tolerance for state variables (HHT and implicit Euler) | double | -- | No | 1e-4 |
| `abs_tolerance_multipliers` | Absolute tolerance for Lagrange multipliers (HHT and implicit Euler) | double | -- | No | 1e2 |
| `max_iterations` | Maximum number of non-linear iteration for implicit integrators | integer | -- | No | 50 |
| `use_stepsize_control` | Whether to use internal step-size control (HHT) | boolean | -- | No | false |
| `use_modified_newton` | Whether to use a modified Newton iteration (HHT) | boolean | -- | No | false |


## Solver types and parameters

Each solver supports different configuration parameters depending on the solver type: 

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | (DVI or linear) solver type | enum |  `BARZILAI_BORWEIN`,<br>`PSOR`,<br>`APGD`,<br>`MINRES`,<br>`GMRES`,<br>`BICGSTAB`,<br>`PARDISO`,<br>`MUMPS`,<br>`SPARSE_LU`,<br>`SPARSE_QR` | Yes | `BARZILAI_BORWEIN` |

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

## Run-time visualization parameters

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

Below is an example of an MBS solver configuration:

\include data/yaml/mbs/solver_mbs.yaml

## YAML schema

The YAML MBS solver specification file must follow the ``data/yaml/schema/mbs_solver.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/mbs_solver.schema.yaml
