YAML schema for Chrono::FSI-SPH solver specification {#YAML_schema_fsisph_solver}
=======================================

A Chrono YAML SPH solver file defines the parameters needed to run a Chrono::FSI-SPH simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- The `sph` object specifying SPH method parameters.
- The `kernel` object specifying the SPH kernel definition.
- The `discretization` object specifying parameters for the SPH discretization.
- The `boundary_conditions` object specifying the method and parameters for treating fluid-solid boundary conditions.
- The `integration` object specifying the type and parameters for the time integrator.
- The `proximity_search` object specifying parameters for the proximity (neighbor) search algorithm.
- The `particle_shifting` object specifying the method and parameters for the particle shifting algorithm.
- The `viscosity` object specifying the method and parameters for viscosity treatment.


All objects listed below are optional; any object or property that is omitted keeps its default value.
Note that an SPH solver file specifies neither output nor run-time visualization settings; those belong in the
[FSI-SPH simulation file](@ref YAML_schema_fsisph_simulation).

## SPH method specification

The `sph` object collects the base parameters of the weakly compressible SPH formulation.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `eos_type` | Equation of state | enum | `ISOTHERMAL`,`TAIT` | No | `ISOTHERMAL` |
| `use_delta_sph` | Whether to use delta-SPH density diffusion | boolean | -- | No | `true` |
| `delta_sph_coefficient` | Delta-SPH coefficient | double | -- | No | 0.1 |
| `max_velocity` | Maximum expected velocity, used for scaling | double | -- | No | 1.0 |
| `min_distance_coefficient` | Minimum inter-particle distance, as a fraction of the kernel radius | double | -- | No | 0.01 |
| `density_reinit_steps` | Number of steps between density re-initializations | integer | -- | No | 2e8 |
| `use_density_based_projection` | Use density-based projection (`IMPLICIT_SPH` only) | boolean | -- | No | `false` |
| `free_surface_threshold` | Divergence threshold used to identify free-surface particles (CRM only) | double | -- | No | 2.0 |

`free_surface_threshold` is compared against the divergence of the position field; particles with divergence below the
threshold are treated as free-surface particles.

## Kernel specification

The `kernel` object defines the SPH smoothing kernel and the particle resolution.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `kernel_type` | SPH smoothing kernel | enum | `QUADRATIC`,`CUBIC_SPLINE`,<br>`QUINTIC_SPLINE`,`WENDLAND` | No | `CUBIC_SPLINE` |
| `initial_spacing` | Initial particle spacing | double | -- | No | 0.01 |
| `d0_multiplier` | Kernel length multiplier; the kernel length is `d0_multiplier * initial_spacing` | double | -- | No | 1.2 |

## SPH discretization specification

The `discretization` object selects the consistent (corrected) forms of the SPH differential operators.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `use_consistent_gradient_discretization` | Use the G matrix in the SPH gradient approximation | boolean | -- | No | `false` |
| `use_consistent_laplacian_discretization` | Use the L matrix in the SPH Laplacian approximation | boolean | -- | No | `false` |

## Boundary condition treatment

The `boundary_conditions` object controls how fluid-solid boundaries are enforced.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `boundary_method` | Boundary condition enforcement method | enum | `ADAMI`,`HOLMES` | No | `ADAMI` |
| `num_bce_layers` | Number of BCE marker layers, on boundaries and on solids | integer | -- | No | 3 |

## Integrator specification

The `integration` object defines the fluid solver time step and integration scheme.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `time_step` | Fluid solver time step size in seconds | double | -- | Yes | -- |
| `integration_scheme` | SPH integration scheme | enum | `EULER`,`RK2`,`VERLET`,<br>`SYMPLECTIC`,`IMPLICIT_SPH` | No | `RK2` |
| `use_variable_time_step` | Whether to adapt the time step during the simulation | boolean | -- | No | `false` |

Note that `time_step` is required whenever the `integration` object is present.

## Proximity search treatment

The `proximity_search` object controls how often neighbor lists are rebuilt.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `num_proximity_search_steps` | Number of steps between updates of the neighbor lists | integer | -- | No | 1 |

## Particle shifting treatment

The `particle_shifting` object selects the particle shifting algorithm and its coefficients.
Which coefficients are relevant depends on the selected `shifting_method`.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `shifting_method` | Particle shifting method | enum | `NONE`,`PPST`,`XSPH`,<br>`PPST_XSPH`,`DIFFUSION`,<br>`DIFFUSION_XSPH` | No | `XSPH` |
| `shifting_xsph_eps` | XSPH coefficient (`XSPH`, `PPST_XSPH`, `DIFFUSION_XSPH`) | double | -- | No | 0.5 |
| `shifting_ppst_push` | PPST pushing coefficient (`PPST`, `PPST_XSPH`) | double | -- | No | 3.0 |
| `shifting_ppst_pull` | PPST pulling coefficient (`PPST`, `PPST_XSPH`) | double | -- | No | 1.0 |
| `shifting_beta_implicit` | Shifting coefficient used by the implicit solver (`IMPLICIT_SPH` scheme) | double | -- | No | 1.0 |
| `shifting_diffusion_A` | Diffusion-based shifting coefficient A (`DIFFUSION`, `DIFFUSION_XSPH`) | double | 1 to 6 | No | 1.0 |
| `shifting_diffusion_AFSM` | Diffusion-based shifting coefficient AFSM (`DIFFUSION`, `DIFFUSION_XSPH`) | double | -- | No | 3.0 |
| `shifting_diffusion_AFST` | Diffusion-based shifting coefficient AFST (`DIFFUSION`, `DIFFUSION_XSPH`) | double | -- | No | 2.0 |

## Viscosity treatment

The `viscosity` object selects the viscosity model and its coefficient.

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `viscosity_method` | Viscosity treatment method | enum | `LAMINAR`,<br>`ARTIFICIAL_UNILATERAL`,<br>`ARTIFICIAL_BILATERAL` | No | `ARTIFICIAL_UNILATERAL` |
| `artificial_viscosity` | Artificial viscosity coefficient (both `ARTIFICIAL_*` methods) | double | -- | No | 0.02 |

The `LAMINAR` method uses the physical viscosity given by the `fluid_properties` entry of the
[SPH model file](@ref YAML_schema_fsisph_model) instead.

## Example

Below is an example of an SPH solver configuration:

\include data/yaml/fsi/dam_break/solver_sph.yaml

## YAML schema

The YAML SPH solver specification file must follow the ``data/yaml/schema/fsisph_solver.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsisph_solver.schema.yaml
