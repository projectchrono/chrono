YAML schema for Chrono MBS solver specification {#YAML_schema_mbs_solver}
=======================================

A Chrono YAML MBS solver file defines the parameters needed to run a Chrono simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The `contact_method` specifying the formulation for contact force generation.
- [optional] The `integrator` object specifying the type and settings for the time integrator.
  If omitted, an `EULER_IMPLICIT_LINEARIZED` integrator with a time step of 1e-3 is used.
- [optional] The `solver` object specifying the type and settings for the (linear or DVI) solver.
  If omitted, a `BARZILAI_BORWEIN` solver with default settings is used.

Note that a solver file specifies neither output nor run-time visualization settings; those belong in the
[MBS simulation file](@ref YAML_schema_mbs_simulation).

## Contact formulation

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `contact_method` | Contact method for collision detection and response | string | `SMC`,`NSC` | Yes | -- |

## Integrator types and parameters

Each integrator can support the following settings depending on the integrator type:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Integrator type | enum | `EULER_IMPLICIT_LINEARIZED`,<br>`EULER_IMPLICIT_PROJECTED`,<br>`EULER_IMPLICIT`,<br>`HHT` | Yes | -- |
| `rel_tolerance` | Relative tolerance (HHT and implicit Euler) | double | -- | No | 1e-4 |
| `time_step` | Integration timestep in seconds | double | -- | Yes | -- |

Both `type` and `time_step` are required if the `integrator` object is present.
| `abs_tolerance_states` | Absolute tolerance for state variables (HHT and implicit Euler) | double | -- | No | 1e-4 |
| `abs_tolerance_multipliers` | Absolute tolerance for Lagrange multipliers (HHT and implicit Euler) | double | -- | No | 1e2 |
| `max_iterations` | Maximum number of non-linear iterations for implicit integrators | integer | -- | No | 50 |
| `use_stepsize_control` | Whether to use internal step-size control (HHT) | boolean | -- | No | false |
| `use_modified_newton` | Whether to use a modified Newton iteration (HHT) | boolean | -- | No | false |


## Solver types and parameters

Each solver supports different configuration parameters depending on the solver type: 

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | (DVI or linear) solver type | enum |  `BARZILAI_BORWEIN`,<br>`PSOR`,<br>`APGD`,<br>`MINRES`,<br>`GMRES`,<br>`BICGSTAB`,<br>`PARDISO`,<br>`MUMPS`,<br>`SPARSE_LU`,<br>`SPARSE_QR` | Yes | -- |

`type` is required if the `solver` object is present.
The `PARDISO` and `MUMPS` solvers require the corresponding optional Chrono module to be enabled.

#### Iterative DVI Solvers (BARZILAI_BORWEIN, APGD, PSOR)

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `max_iterations`| Maximum number of iterations | integer | -- | No | 100 |
| `overrelaxation_factor` | Overrelaxation factor for improved convergence | double | -- | No | 1.0 |
| `sharpness_factor` | Sharpness factor for solver response tuning | double | -- | No | 1.0 |
| `enable_diagonal_preconditioner` | Enable diagonal preconditioner to accelerate convergence | boolean | -- | No | false |
| `warm_start` | Warm start the solver from the previous solution | boolean | -- | No | false |

#### Iterative Krylov Linear Solvers (BICGSTAB, MINRES, GMRES)

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `max_iterations` | Maximum number of iterations | integer | -- | No | 100 |
| `tolerance` | Residual tolerance for convergence | double | -- | No | 0.0 |
| `enable_diagonal_preconditioner` | Enable diagonal preconditioner to accelerate convergence | boolean | -- | No | false |
| `warm_start` | Warm start the solver from the previous solution | boolean | -- | No | false |

#### Direct Sparse Linear Solvers (SPARSE_LU, SPARSE_QR, PARDISO, MUMPS)

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `lock_sparsity_pattern`| Keep matrix sparsity pattern unchanged | boolean | -- | No | false |
| `use_sparsity_pattern_learner` | Evaluate matrix sparsity pattern in a pre-processing stage (for `SPARSE_LU` and `SPARSE_QR` only) | boolean | -- | No | true |

## Example

Below is an example of an MBS solver configuration:

\include data/yaml/mbs/solver_mbs.yaml

## YAML schema

The YAML MBS solver specification file must follow the ``data/yaml/schema/mbs_solver.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/mbs_solver.schema.yaml
